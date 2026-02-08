# app.py
from __future__ import annotations
from fastapi import FastAPI, Body
from fastapi.responses import Response, JSONResponse
import numpy as np
from scipy.ndimage import distance_transform_edt, gaussian_filter
from skimage.measure import marching_cubes
import trimesh
import time

app = FastAPI()

# -----------------------------
# utils: build csys
# -----------------------------
def rpy_deg_to_R(rpy):
    # roll-pitch-yaw (Z-Y-X or X-Y-Z どっち？問題を避けるため、ここは最初 WCS only を想定)
    # 今回 rpy=(0,0,0) なので恒等でOK。拡張するなら順序を規定して実装する。
    return np.eye(3, dtype=np.float64)

def world_to_local(p_world: np.ndarray, origin: np.ndarray, R: np.ndarray):
    # p_local = R^T (p_world - origin)
    return (p_world - origin) @ R

# -----------------------------
# 2D signed distance to polygon in (z,r)
# polygon: Nx2 array [ [z,r], ... ] closed or open ok
# returns signed distance (negative inside)
# -----------------------------
def point_in_poly_2d(q: np.ndarray, poly: np.ndarray) -> bool:
    # ray casting in (z,r) plane; treat z as x, r as y
    x, y = q[0], q[1]
    inside = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        # check edge intersects ray to +inf in x
        cond = ((y0 > y) != (y1 > y))
        if cond:
            xinters = (x1 - x0) * (y - y0) / (y1 - y0 + 1e-30) + x0
            if xinters > x:
                inside = not inside
    return inside

def dist_point_to_segment_2d(q: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    t = np.dot(q - a, ab) / (np.dot(ab, ab) + 1e-30)
    t = np.clip(t, 0.0, 1.0)
    proj = a + t * ab
    return float(np.linalg.norm(q - proj))

def point_in_poly_2d_vectorized(q_array: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """ベクトル化版: q_array: (N, 2) array -> (N,) bool array"""
    x, y = q_array[:, 0], q_array[:, 1]
    inside = np.zeros(len(q_array), dtype=bool)
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        cond = ((y0 > y) != (y1 > y))
        if np.any(cond):
            xinters = (x1 - x0) * (y[cond] - y0) / (y1 - y0 + 1e-30) + x0
            inside[cond] ^= (xinters > x[cond])
    return inside

def sdf_polygon_2d_vectorized(q_array: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """ベクトル化版: q_array: (N, 2) array -> (N,) float array"""
    N = len(q_array)
    dmin = np.full(N, 1e30, dtype=np.float64)
    n = len(poly)
    
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        ab = b - a
        ab_dot = np.dot(ab, ab) + 1e-30
        
        # 各点から線分への距離
        diff = q_array - a
        t = np.dot(diff, ab) / ab_dot
        t = np.clip(t, 0.0, 1.0)
        proj = a + t[:, np.newaxis] * ab
        dist = np.linalg.norm(q_array - proj, axis=1)
        dmin = np.minimum(dmin, dist)
    
    inside = point_in_poly_2d_vectorized(q_array, poly)
    return np.where(inside, -dmin, dmin)

def sdf_polygon_2d(q: np.ndarray, poly: np.ndarray) -> float:
    # unsigned min distance to edges
    dmin = 1e30
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        dmin = min(dmin, dist_point_to_segment_2d(q, a, b))
    inside = point_in_poly_2d(q, poly)
    return -dmin if inside else dmin

# -----------------------------
# build axisymmetric polygon from turn_od_profile
# profile points are [{"z":..., "radius":...}, ...]
# returns polygon in (z,r) as float64
# -----------------------------
def build_turn_od_polygon(profile: list[dict]) -> np.ndarray:
    pts = np.array([[float(p["z"]), float(p["radius"])] for p in profile], dtype=np.float64)
    # sort by z just in case
    pts = pts[np.argsort(pts[:, 0])]
    z0 = pts[0, 0]
    z1 = pts[-1, 0]
    # close with axis (r=0)
    poly = np.vstack([
        [z0, 0.0],
        pts,
        [z1, 0.0],
    ])
    return poly

# -----------------------------
# 3D SDF evaluator for this request
# (only supports: stock cylinder + turn_od_profile)
# -----------------------------
def sdf_stock_cylinder(p: np.ndarray, dia: float, h: float):
    # ベクトル化対応: p が (3,) なら float、(N,3) なら (N,) float配列を返す
    if p.ndim == 1:
        # スカラー版
        r = np.hypot(p[0], p[1])
        R = dia * 0.5
        dr = r - R
        dz0 = -p[2]
        dz1 = p[2] - h
        outside = np.maximum.reduce([dr, dz0, dz1])
        inside = np.minimum.reduce([dr, dz0, dz1])
        return float(outside if outside > 0 else inside)
    else:
        # ベクトル版: p.shape = (N, 3)
        r = np.hypot(p[:, 0], p[:, 1])
        R = dia * 0.5
        dr = r - R
        dz0 = -p[:, 2]
        dz1 = p[:, 2] - h
        outside = np.maximum.reduce([dr, dz0, dz1])
        inside = np.minimum.reduce([dr, dz0, dz1])
        return np.where(outside > 0, outside, inside)

def make_sdf_eval(req: dict):
    stock = req["stock"]
    assert stock["type"] == "cylinder"
    dia = float(stock["params"]["dia"])
    h = float(stock["params"]["h"])

    feats = req.get("features", [])
    turn = None
    for f in feats:
        if f.get("feature_type") == "turn_od_profile":
            turn = f
            break

    if turn is None:
        # no feature -> stock only
        def eval_sdf(p_world):
            return sdf_stock_cylinder(p_world, dia, h)
        return eval_sdf, (dia, h)

    poly = build_turn_od_polygon(turn["params"]["profile"])

    def eval_sdf(p_world):
        # ベクトル化対応版
        sdf_cyl = sdf_stock_cylinder(p_world, dia, h)
        
        if p_world.ndim == 1:
            r = float(np.hypot(p_world[0], p_world[1]))
            z = float(p_world[2])
            q = np.array([z, r], dtype=np.float64)
            sdf_turn = sdf_polygon_2d(q, poly)
            return max(sdf_cyl, sdf_turn)
        else:
            # ベクトル版
            r = np.hypot(p_world[:, 0], p_world[:, 1])
            z = p_world[:, 2]
            q_array = np.stack([z, r], axis=1)
            sdf_turn = sdf_polygon_2d_vectorized(q_array, poly)
            return np.maximum(sdf_cyl, sdf_turn)

    return eval_sdf, (dia, h)

# ==============================================
# Sample A: SDF smooth with Gaussian filter
# (推薦：最も効果的で実装が簡単)
# ==============================================
def export_stl_smooth_sdf_gaussian(eval_sdf, bounds_min, bounds_max, pitch, 
                                    sigma=1.0, level_offset=0.0):
    """
    Method A: SDF平滑化後にMarching Cubes
    - sigma: ガウシアンフィルタのシグマ（ピクセル単位）
             推奨値: 0.8〜1.5
    - level_offset: 平滑化後のレベル値
             寸法調整用：外形が縮むなら負、膨らむなら正
             ざっくり: ±(0.3〜0.8)*pitch を試す
    """
    print("\n[STL生成開始] Method A: ガウシアン平滑化版")
    t0 = time.time()
    
    # build grid
    print(f"グリッド構築: bounds_min={bounds_min}, bounds_max={bounds_max}, pitch={pitch}")
    size = (bounds_max - bounds_min) / pitch
    nx, ny, nz = np.ceil(size).astype(int) + 1
    print(f"  グリッドサイズ: nx={nx}, ny={ny}, nz={nz}")

    xs = bounds_min[0] + np.arange(nx) * pitch
    ys = bounds_min[1] + np.arange(ny) * pitch
    zs = bounds_min[2] + np.arange(nz) * pitch

    # sample
    print("[numpy計算] SDF値をサンプリング中...")
    t1 = time.time()
    x_grid, y_grid, z_grid = np.meshgrid(xs, ys, zs, indexing='ij')
    points = np.stack([x_grid.ravel(), y_grid.ravel(), z_grid.ravel()], axis=1)
    sdf_flat = eval_sdf(points)
    sdf = sdf_flat.reshape((nx, ny, nz)).astype(np.float32)
    print(f"SDF計算完了: {time.time() - t1:.2f}秒")

    # ★ Gaussian smooth (3D)
    print(f"[平滑化] ガウシアンフィルタ適用: sigma={sigma}")
    t_smooth = time.time()
    sdf_smooth = gaussian_filter(sdf, sigma=sigma)
    print(f"  平滑化完了: {time.time() - t_smooth:.2f}秒")

    # Marching Cubes with level offset
    print(f"[STL生成] Marching Cubesで等面を抽出: level={level_offset}")
    t2 = time.time()
    sdf_zyx = np.transpose(sdf_smooth, (2,1,0))
    verts, faces, normals, _ = marching_cubes(sdf_zyx, level=level_offset, 
                                               spacing=(pitch, pitch, pitch))
    print(f"  頂点数: {len(verts)}, 面数: {len(faces)}")
    print(f"Marching Cubes完了: {time.time() - t2:.2f}秒")
    
    # Convert from (z,y,x) to world (x,y,z)
    v = np.zeros_like(verts)
    v[:, 0] = verts[:, 2] + bounds_min[0]
    v[:, 1] = verts[:, 1] + bounds_min[1]
    v[:, 2] = verts[:, 0] + bounds_min[2]

    print("[STL生成] メッシュ作成中...")
    mesh = trimesh.Trimesh(vertices=v, faces=faces, process=False)

    # keep largest component
    print("[STL生成] メッシュ修復中...")
    t3 = time.time()
    parts = mesh.split(only_watertight=False)
    if len(parts) > 1:
        print(f"  複数のコンポーネント検出: {len(parts)}個 -> 最大のものを選択")
        mesh = max(parts, key=lambda m: m.volume if m.is_volume else m.area)

    mesh.merge_vertices()
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()
    print(f"  基本的な修復完了")

    # try:
    #     import pymeshfix
    #     print(f"  pymeshfixで詳細修復中...")
    #     mf = pymeshfix.MeshFix(mesh.vertices, mesh.faces)
    #     mf.repair(joincomp=True, remove_smallest_components=True)
    #     mesh = trimesh.Trimesh(vertices=mf.points, faces=mf.faces, process=True)
    #     print(f"  pymeshfix修復完了")
    # except Exception as e:
    #     print(f"  pymeshfix修復スキップ: {e}")

    print(f"メッシュ修復完了: {time.time() - t3:.2f}秒")
    print(f"[STL生成完了] 総処理時間: {time.time() - t0:.2f}秒\n")
    
    return mesh.export(file_type="stl")


# ==============================================
# Sample B: Re-distance SDF + smooth
# (より正確：SDFが「距離っぽいけど厳密でない」場合)
# ==============================================
def export_stl_redist_smooth(eval_sdf, bounds_min, bounds_max, pitch, 
                              sigma=1.0, level_offset=0.0):
    """
    Method B: 再距離化 → 平滑化 → Marching Cubes
    
    eval_sdfが「距離SDF」でなく「単なる符号付き場」の場合、
    distance_transform_edt で正確な距離SDF に再計算してから平滑化
    
    - sigma: ガウシアンフィルタのシグマ
    - level_offset: MC後のレベル値調整
    """
    print("\n[STL生成開始] Method B: 再距離化+平滑化版")
    t0 = time.time()
    
    # build grid
    print(f"グリッド構築: bounds_min={bounds_min}, bounds_max={bounds_max}, pitch={pitch}")
    size = (bounds_max - bounds_min) / pitch
    nx, ny, nz = np.ceil(size).astype(int) + 1
    print(f"  グリッドサイズ: nx={nx}, ny={ny}, nz={nz}")

    xs = bounds_min[0] + np.arange(nx) * pitch
    ys = bounds_min[1] + np.arange(ny) * pitch
    zs = bounds_min[2] + np.arange(nz) * pitch

    # sample original SDF
    print("[numpy計算] SDF値をサンプリング中...")
    t1 = time.time()
    x_grid, y_grid, z_grid = np.meshgrid(xs, ys, zs, indexing='ij')
    points = np.stack([x_grid.ravel(), y_grid.ravel(), z_grid.ravel()], axis=1)
    sdf_flat = eval_sdf(points)
    sdf = sdf_flat.reshape((nx, ny, nz)).astype(np.float32)
    print(f"SDF計算完了: {time.time() - t1:.2f}秒")

    # ★ Re-distance using distance_transform_edt
    print("[再距離化] occupancy を取得...")
    t_redist = time.time()
    
    # occupancy: True = inside (sdf < 0), False = outside
    occupancy = (sdf < 0)
    
    # distance_transform_edt: inside距離
    dist_inside = distance_transform_edt(occupancy) * pitch
    
    # outside距離（逆領域）
    dist_outside = distance_transform_edt(~occupancy) * pitch
    
    # 符号付き距離SDF を再構築
    sdf_redist = np.where(occupancy, -dist_inside, dist_outside).astype(np.float32)
    print(f"  再距離化完了: {time.time() - t_redist:.2f}秒")

    # ★ Gaussian smooth
    print(f"[平滑化] ガウシアンフィルタ適用: sigma={sigma}")
    t_smooth = time.time()
    sdf_smooth = gaussian_filter(sdf_redist, sigma=sigma)
    print(f"  平滑化完了: {time.time() - t_smooth:.2f}秒")

    # Marching Cubes
    print(f"[STL生成] Marching Cubesで等面を抽出: level={level_offset}")
    t2 = time.time()
    sdf_zyx = np.transpose(sdf_smooth, (2,1,0))
    verts, faces, normals, _ = marching_cubes(sdf_zyx, level=level_offset, 
                                               spacing=(pitch, pitch, pitch))
    print(f"  頂点数: {len(verts)}, 面数: {len(faces)}")
    print(f"Marching Cubes完了: {time.time() - t2:.2f}秒")
    
    # Convert from (z,y,x) to world (x,y,z)
    v = np.zeros_like(verts)
    v[:, 0] = verts[:, 2] + bounds_min[0]
    v[:, 1] = verts[:, 1] + bounds_min[1]
    v[:, 2] = verts[:, 0] + bounds_min[2]

    print("[STL生成] メッシュ作成中...")
    mesh = trimesh.Trimesh(vertices=v, faces=faces, process=False)

    # keep largest component
    print("[STL生成] メッシュ修復中...")
    t3 = time.time()
    parts = mesh.split(only_watertight=False)
    if len(parts) > 1:
        print(f"  複数のコンポーネント検出: {len(parts)}個 -> 最大のものを選択")
        mesh = max(parts, key=lambda m: m.volume if m.is_volume else m.area)

    mesh.merge_vertices()
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()
    print(f"  基本的な修復完了")

    try:
        import pymeshfix
        print(f"  pymeshfixで詳細修復中...")
        mf = pymeshfix.MeshFix(mesh.vertices, mesh.faces)
        mf.repair(joincomp=True, remove_smallest_components=True)
        mesh = trimesh.Trimesh(vertices=mf.points, faces=mf.faces, process=True)
        print(f"  pymeshfix修復完了")
    except Exception as e:
        print(f"  pymeshfix修復スキップ: {e}")

    print(f"メッシュ修復完了: {time.time() - t3:.2f}秒")
    print(f"[STL生成完了] 総処理時間: {time.time() - t0:.2f}秒\n")
    
    return mesh.export(file_type="stl")

# -----------------------------
# sample SDF grid and mesh it
# -----------------------------
def export_stl_from_sdf(eval_sdf, bounds_min, bounds_max, pitch):
    print("\n[STL生成開始]")
    t0 = time.time()
    
    # build grid
    print(f"グリッド構築: bounds_min={bounds_min}, bounds_max={bounds_max}, pitch={pitch}")
    size = (bounds_max - bounds_min) / pitch
    nx, ny, nz = np.ceil(size).astype(int) + 1
    print(f"  グリッドサイズ: nx={nx}, ny={ny}, nz={nz} (総セル数: {nx*ny*nz})")

    xs = bounds_min[0] + np.arange(nx) * pitch
    ys = bounds_min[1] + np.arange(ny) * pitch
    zs = bounds_min[2] + np.arange(nz) * pitch

    # sample
    print("[numpy計算] SDF値をサンプリング中...")
    t1 = time.time()
    
    # 全グリッド座標を一度に生成してベクトル化処理
    x_grid, y_grid, z_grid = np.meshgrid(xs, ys, zs, indexing='ij')
    points = np.stack([x_grid.ravel(), y_grid.ravel(), z_grid.ravel()], axis=1)
    
    # ベクトル化eval_sdfで一括評価
    sdf_flat = eval_sdf(points)
    sdf = sdf_flat.reshape((nx, ny, nz)).astype(np.float32)
    
    print(f"SDF計算完了: {time.time() - t1:.2f}秒")

    # marching cubes expects array indexed as (z,y,x) or similar depending; we adapt:
    # We'll transpose to (z,y,x) for skimage
    print("[STL生成] Marching Cubesで等面を抽出中...")
    t2 = time.time()
    sdf_zyx = np.transpose(sdf, (2,1,0))
    verts, faces, normals, _ = marching_cubes(sdf_zyx, level=0.0, spacing=(pitch, pitch, pitch))
    print(f"  頂点数: {len(verts)}, 面数: {len(faces)}")
    print(f"Marching Cubes完了: {time.time() - t2:.2f}秒")
    
    # verts are in (z,y,x) space; convert to world (x,y,z)
    # marching_cubes gives verts as (z,y,x) in same order as spacing
    v = np.zeros_like(verts)
    v[:, 0] = verts[:, 2] + bounds_min[0]  # x
    v[:, 1] = verts[:, 1] + bounds_min[1]  # y
    v[:, 2] = verts[:, 0] + bounds_min[2]  # z

    print("[STL生成] メッシュ作成中...")
    mesh = trimesh.Trimesh(vertices=v, faces=faces, process=False)

    # keep largest component
    print("[STL生成] メッシュ修復中...")
    t3 = time.time()
    parts = mesh.split(only_watertight=False)
    if len(parts) > 1:
        print(f"  複数のコンポーネント検出: {len(parts)}個 -> 最大のものを選択")
        mesh = max(parts, key=lambda m: m.volume if m.is_volume else m.area)

    # basic repair
    mesh.merge_vertices()
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()
    print(f"  基本的な修復完了")

    # stronger repair if available
    try:
        import pymeshfix
        print(f"  pymeshfixで詳細修復中...")
        mf = pymeshfix.MeshFix(mesh.vertices, mesh.faces)
        mf.repair(joincomp=True, remove_smallest_components=True)
        # 現行API: points/faces
        mesh = trimesh.Trimesh(vertices=mf.points, faces=mf.faces, process=True)
        print(f"  pymeshfix修復完了")
    except Exception as e:
        print(f"  pymeshfix修復スキップ: {e}")

    print(f"メッシュ修復完了: {time.time() - t3:.2f}秒")
    print(f"[STL生成完了] 総処理時間: {time.time() - t0:.2f}秒\n")
    
    return mesh.export(file_type="stl")

# -----------------------------
# endpoint
# -----------------------------
@app.post("/sdf/run")
def sdf_run(req: dict = Body(...)):
    units = req.get("units", "mm")
    if units != "mm":
        return JSONResponse({"error": "only mm supported in this minimal implementation"}, status_code=400)

    output_mode = req.get("output_mode", "stl")
    if output_mode not in ("stl", "sdf_grid", "both"):
        return JSONResponse({"error": "output_mode must be stl|sdf_grid|both"}, status_code=400)

    # choose pitch (if not provided)
    pitch = float(req.get("pitch_mm", 0.5))  # 0.5mm default: adjust as needed

    eval_sdf, (dia, h) = make_sdf_eval(req)

    # bounds from stock, with padding
    pad = 2.0 * pitch
    R = dia * 0.5
    bounds_min = np.array([-R - pad, -R - pad, 0.0 - pad], dtype=np.float64)
    bounds_max = np.array([ R + pad,  R + pad, h + pad], dtype=np.float64)

    # export stl
    #stl_bytes = export_stl_from_sdf(eval_sdf, bounds_min, bounds_max, pitch)
    #stl_bytes = export_stl_redist_smooth(eval_sdf, bounds_min, bounds_max, pitch, sigma=1.0, level_offset=0.0)
    stl_bytes = export_stl_smooth_sdf_gaussian(eval_sdf, bounds_min, bounds_max, pitch, sigma=1.0, level_offset=0.0)
    return Response(content=stl_bytes, media_type="application/vnd.ms-pki.stl")
