// DexelTurnPoc.cs
// POC: 単方向Dexel（Z-dexel）で「旋削ODプロファイル」を反映し、STL(バイナリ)を書き出す雰囲気コード。
// 目的：コードの“流れ”を見る用。最適化/厳密メッシュはまだ（Phase1相当）。
//
// - 入力 JSON を読み取り
// - 円柱ストックを Z-dexel グリッド（XY格子、各列にZ区間）として保持
// - turn_od_profile で列の区間を更新（ここでは「最終形状」へ直接反映。工具CLは未使用）
// - Dexelから「半径(z)」をサンプルして、回転体メッシュ(STL)を生成
//
// 制約：法線不要（STL法線は0で埋める）
// 注意：本コードは「旋削OD」前提で、形状は回転対称としてメッシュ化します。
//       （ミーリング一般形状は別の抽出が必要）

using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace DexelTurnPoc
{
    public static class Program
    {
        public static void Main()
        {
            // ---- 入力(JSON) ----
            string json = @"
{
  ""units"": ""mm"",
  ""origin"": ""world"",
  ""stock"": {
    ""type"": ""cylinder"",
    ""params"": { ""dia"": 50.0, ""h"": 80.0 }
  },
  ""csys_list"": [
    {
      ""name"": ""WCS"",
      ""role"": ""world"",
      ""origin"": { ""x"": 0.0, ""y"": 0.0, ""z"": 0.0 },
      ""rpy_deg"": { ""r"": 0.0, ""p"": 0.0, ""y"": 0.0 }
    }
  ],
  ""features"": [
    {
      ""feature_type"": ""turn_od_profile"",
      ""id"": ""L3_F1_TURN_OD_PROFILE"",
      ""params"": {
        ""csys_id"": ""WCS"",
        ""profile"": [
          { ""z"": 0.0,  ""radius"": 25.0 },
          { ""z"": 20.0, ""radius"": 25.0 },
          { ""z"": 20.0, ""radius"": 20.0 },
          { ""z"": 40.0, ""radius"": 20.0 },
          { ""z"": 40.0, ""radius"": 15.0 },
          { ""z"": 80.0, ""radius"": 15.0 }
        ]
      }
    }
  ],
  ""output_mode"": ""stl"",
  ""file_template_solid"": ""case3_{step:02d}_{name}_solid.stl"",
  ""dry_run"": false,
  ""pitch_mm"": 0.5
}
";

            var opts = new JsonSerializerOptions
            {
                PropertyNameCaseInsensitive = true
            };
            var input = JsonSerializer.Deserialize<InputRoot>(json, opts)
                        ?? throw new InvalidOperationException("JSON parse failed");

            double pitch = input.PitchMm <= 0 ? 0.5 : input.PitchMm;

            // ---- ストック(円柱) ----
            if (!string.Equals(input.Stock.Type, "cylinder", StringComparison.OrdinalIgnoreCase))
                throw new NotSupportedException("POC supports only cylinder stock.");

            double stockR = input.Stock.Params.Dia * 0.5;
            double stockH = input.Stock.Params.H;

            // ---- Dexel grid を生成（Z-dexel）----
            // AABB: [-R, +R] x [-R, +R] x [0, H]
            var grid = DexelGrid.CreateForCylinder(stockR, stockH, pitch);

            // ---- Feature適用（turn_od_profile）----
            foreach (var f in input.Features)
            {
                if (string.Equals(f.FeatureType, "turn_od_profile", StringComparison.OrdinalIgnoreCase))
                {
                    var profile = f.Params.Profile;
                    if (profile == null || profile.Count < 2)
                        throw new InvalidOperationException("turn_od_profile.profile is empty");

                    // ここが「除去加工の反映（Dexel更新）」：各列のZ区間をプロファイルに合わせて更新
                    ApplyTurnOdProfile(grid, stockR, stockH, profile);
                }
                else
                {
                    Console.WriteLine($"[skip] feature_type={f.FeatureType} (POC only turn_od_profile)");
                }
            }

            // ---- Dexel → 回転体メッシュ → STL ----
            // “雰囲気”として、Zのサンプル点で「そのzでの外径半径」をDexelから推定し、回転体を三角形化します。
            // 角分割は粗め。必要なら上げる。
            int radialSegments = 96;   // 360/96 = 3.75deg
            var mesh = MeshBuilder.BuildRevolvedMeshFromDexel(grid, zSamples: BuildZSlices(input), radialSegments);

            string outPath = $"case3_01_{SanitizeName(input.Features.First().Id)}_solid.stl";
            using (var fs = File.Create(outPath))
                StlBinaryWriter.Write(fs, mesh.Triangles);

            Console.WriteLine($"Wrote STL: {Path.GetFullPath(outPath)}");
            Console.WriteLine($"Triangles: {mesh.Triangles.Count}");
        }

        private static string SanitizeName(string s)
        {
            foreach (char c in Path.GetInvalidFileNameChars())
                s = s.Replace(c, '_');
            return s;
        }

        // Zサンプル：段差を正しく表現するため、同一Zで複数の半径がある場合は
        // わずかなオフセット（-ε, +ε）を加えてサンプル点を分離
        private static List<double> BuildZSlices(InputRoot input)
        {
            var f = input.Features.FirstOrDefault(x => x.FeatureType == "turn_od_profile");
            if (f?.Params?.Profile == null || f.Params.Profile.Count == 0)
                return new List<double> { 0.0, input.Stock.Params.H };

            var profile = f.Params.Profile.OrderBy(p => p.Z).ToList();
            var zs = new List<double>();
            const double epsilon = 1e-6;

            for (int i = 0; i < profile.Count; i++)
            {
                double z = profile[i].Z;
                
                // 同一Zで異なる半径がある場合（段差）、z-ε と z+ε として分離
                bool hasPrevious = i > 0 && Math.Abs(profile[i - 1].Z - z) < 1e-9;
                bool hasNext = i < profile.Count - 1 && Math.Abs(profile[i + 1].Z - z) < 1e-9;

                if (hasNext)
                {
                    // 次の点も同じZなら、このZ値を z-ε として記録（段差の手前）
                    if (!zs.Contains(z - epsilon))
                        zs.Add(z - epsilon);
                }
                else if (hasPrevious)
                {
                    // 前の点が同じZなら、このZ値を z+ε として記録（段差の奥）
                    if (!zs.Contains(z + epsilon))
                        zs.Add(z + epsilon);
                }
                else
                {
                    // 段差でない通常のポイント
                    zs.Add(z);
                }
            }

            // 重複除去してソート
            zs = zs.Distinct().OrderBy(z => z).ToList();

            // 安全：0とHは必ず含める
            double h = input.Stock.Params.H;
            if (!zs.Contains(0.0)) zs.Insert(0, 0.0);
            if (!zs.Contains(h)) zs.Add(h);

            return zs;
        }

        // --- 旋削ODプロファイルをDexelに反映 ---
        // ここでは「最終形状」を列ごとに直接構築します。
        // （工具CLがある場合は、各ステップでAABB→列更新→dirtyメッシュ、に置き換える）
        private static void ApplyTurnOdProfile(DexelGrid grid, double stockR, double stockH, List<ProfilePoint> profile)
        {
            // プロファイルを Z昇順で保持（同一Zで複数点は“段差”）
            var sorted = profile.OrderBy(p => p.Z).ToList();

            // 半径関数 r(z) を「区分線形」で評価（ただし入力は同一Zの段差を含むので、そのままの接続でOK）
            double RadiusAt(double z)
            {
                z = Math.Clamp(z, 0.0, stockH);
                // “段差”対応：同一Zがあれば最後の点を採用（旋削は通常、後の半径が適用されるイメージ）
                for (int i = sorted.Count - 1; i >= 0; --i)
                    if (Math.Abs(sorted[i].Z - z) < 1e-9)
                        return sorted[i].Radius;

                // 区間探索（POCなので線形でOK。最適化は二分探索）
                for (int i = 0; i < sorted.Count - 1; i++)
                {
                    var a = sorted[i];
                    var b = sorted[i + 1];
                    if (a.Z <= z && z <= b.Z)
                    {
                        double t = (z - a.Z) / Math.Max(1e-12, (b.Z - a.Z));
                        return a.Radius + (b.Radius - a.Radius) * t;
                    }
                }
                // 範囲外は端点
                if (z < sorted[0].Z) return sorted[0].Radius;
                return sorted[^1].Radius;
            }

            // 各列（x,y固定）に対して：
            // その列の半径 rxy に対し、材料が残るのは「rxy <= RadiusAt(z)」を満たす z区間。
            // 旋削ODは外側を削るので、通常は z=0から連続（単一区間）になりやすいが、一般化して区間構築。
            for (int iy = 0; iy < grid.Ny; iy++)
            {
                for (int ix = 0; ix < grid.Nx; ix++)
                {
                    double x = grid.XMin + (ix + 0.5) * grid.Pitch;
                    double y = grid.YMin + (iy + 0.5) * grid.Pitch;
                    double rxy = Math.Sqrt(x * x + y * y);

                    // ストック外は空
                    if (rxy > stockR)
                    {
                        grid.Columns[ix, iy].SetEmpty();
                        continue;
                    }

                    // Z方向にサンプルして区間を近似（POC）
                    // ※本気なら「プロファイル線分と rxy の交点z」を解析的に求めて区間化
                    double dz = grid.Pitch; // 0.5mm
                    int nz = Math.Max(1, (int)Math.Ceiling(grid.ZMax / dz));
                    List<Interval> intervals = new();

                    bool inside = false;
                    double zStart = 0;

                    for (int k = 0; k <= nz; k++)
                    {
                        double z = Math.Min(grid.ZMax, k * dz);
                        double rAllowed = Math.Min(stockR, RadiusAt(z));
                        bool nowInside = rxy <= rAllowed + 1e-9;

                        if (!inside && nowInside)
                        {
                            inside = true;
                            zStart = z;
                        }
                        else if (inside && !nowInside)
                        {
                            inside = false;
                            double zEnd = z;
                            if (zEnd > zStart + 1e-9)
                                intervals.Add(new Interval(zStart, zEnd));
                        }
                    }
                    if (inside)
                    {
                        double zEnd = grid.ZMax;
                        if (zEnd > zStart + 1e-9)
                            intervals.Add(new Interval(zStart, zEnd));
                    }

                    grid.Columns[ix, iy].SetIntervals(intervals);
                }
            }
        }
    }

    // =============================
    // Data models for JSON
    // =============================

    public sealed class InputRoot
    {
        [JsonPropertyName("units")] public string Units { get; set; } = "mm";
        [JsonPropertyName("origin")] public string Origin { get; set; } = "world";
        [JsonPropertyName("stock")] public Stock Stock { get; set; } = new();
        [JsonPropertyName("csys_list")] public List<Csys> CsysList { get; set; } = new();
        [JsonPropertyName("features")] public List<Feature> Features { get; set; } = new();
        [JsonPropertyName("output_mode")] public string OutputMode { get; set; } = "stl";
        [JsonPropertyName("file_template_solid")] public string FileTemplateSolid { get; set; } = "case_{step:02d}_{name}_solid.stl";
        [JsonPropertyName("dry_run")] public bool DryRun { get; set; } = false;
        [JsonPropertyName("pitch_mm")] public double PitchMm { get; set; } = 0.5;
    }

    public sealed class Stock
    {
        [JsonPropertyName("type")] public string Type { get; set; } = "cylinder";
        [JsonPropertyName("params")] public StockParams Params { get; set; } = new();
    }

    public sealed class StockParams
    {
        [JsonPropertyName("dia")] public double Dia { get; set; }
        [JsonPropertyName("h")] public double H { get; set; }
    }

    public sealed class Csys
    {
        [JsonPropertyName("name")] public string Name { get; set; } = "WCS";
        [JsonPropertyName("role")] public string Role { get; set; } = "world";
        [JsonPropertyName("origin")] public Vec3 Origin { get; set; } = new();
        [JsonPropertyName("rpy_deg")] public RpyDeg RpyDeg { get; set; } = new();
    }

    public sealed class Vec3 { [JsonPropertyName("x")] public double X { get; set; } [JsonPropertyName("y")] public double Y { get; set; } [JsonPropertyName("z")] public double Z { get; set; } }
    public sealed class RpyDeg { [JsonPropertyName("r")] public double R { get; set; } [JsonPropertyName("p")] public double P { get; set; } [JsonPropertyName("y")] public double Y { get; set; } }

    public sealed class Feature
    {
        [JsonPropertyName("feature_type")] public string FeatureType { get; set; } = "";
        [JsonPropertyName("id")] public string Id { get; set; } = "";
        [JsonPropertyName("params")] public FeatureParams Params { get; set; } = new();
    }

    public sealed class FeatureParams
    {
        [JsonPropertyName("csys_id")] public string CsysId { get; set; } = "WCS";
        [JsonPropertyName("profile")] public List<ProfilePoint>? Profile { get; set; }
    }

    public sealed class ProfilePoint
    {
        [JsonPropertyName("z")] public double Z { get; set; }
        [JsonPropertyName("radius")] public double Radius { get; set; }
    }

    // =============================
    // Dexel data structure (Z-dexel)
    // =============================

    public readonly struct Interval
    {
        public readonly double Z0;
        public readonly double Z1;
        public Interval(double z0, double z1) { Z0 = z0; Z1 = z1; }
    }

    public sealed class DexelColumn
    {
        // POC: interval list. 多くは1本になるはず
        public List<Interval> Intervals { get; private set; } = new();

        public void SetEmpty() => Intervals = new List<Interval>(0);

        public void SetIntervals(List<Interval> intervals)
        {
            // ここでマージ等も可能（POCは省略）
            Intervals = intervals;
        }

        public bool ContainsZ(double z)
        {
            foreach (var it in Intervals)
                if (it.Z0 <= z && z <= it.Z1) return true;
            return false;
        }
    }

    public sealed class DexelGrid
    {
        public double Pitch { get; private set; }
        public int Nx { get; private set; }
        public int Ny { get; private set; }
        public double XMin { get; private set; }
        public double YMin { get; private set; }
        public double ZMax { get; private set; } // ZMin=0固定

        public DexelColumn[,] Columns { get; private set; } = default!;

        private DexelGrid() { }

        public static DexelGrid CreateForCylinder(double radius, double height, double pitch)
        {
            double xmin = -radius, xmax = radius;
            double ymin = -radius, ymax = radius;

            int nx = (int)Math.Ceiling((xmax - xmin) / pitch);
            int ny = (int)Math.Ceiling((ymax - ymin) / pitch);

            var g = new DexelGrid
            {
                Pitch = pitch,
                Nx = nx,
                Ny = ny,
                XMin = xmin,
                YMin = ymin,
                ZMax = height,
                Columns = new DexelColumn[nx, ny]
            };

            // 初期化：円柱内部なら [0,H]、外は空
            for (int iy = 0; iy < ny; iy++)
                for (int ix = 0; ix < nx; ix++)
                {
                    double x = xmin + (ix + 0.5) * pitch;
                    double y = ymin + (iy + 0.5) * pitch;
                    double r = Math.Sqrt(x * x + y * y);

                    var col = new DexelColumn();
                    if (r <= radius + 1e-9)
                        col.SetIntervals(new List<Interval> { new Interval(0.0, height) });
                    else
                        col.SetEmpty();

                    g.Columns[ix, iy] = col;
                }

            return g;
        }

        // Dexelから「あるzでの外径半径」を粗く推定（POC：走査）
        public double EstimateOuterRadiusAtZ(double z)
        {
            double maxR = 0.0;
            for (int iy = 0; iy < Ny; iy++)
                for (int ix = 0; ix < Nx; ix++)
                {
                    var col = Columns[ix, iy];
                    if (!col.ContainsZ(z)) continue;

                    double x = XMin + (ix + 0.5) * Pitch;
                    double y = YMin + (iy + 0.5) * Pitch;
                    double r = Math.Sqrt(x * x + y * y);
                    if (r > maxR) maxR = r;
                }
            return maxR;
        }
    }

    // =============================
    // Mesh + STL writer
    // =============================

    public readonly struct Triangle
    {
        public readonly Vector3 A;
        public readonly Vector3 B;
        public readonly Vector3 C;
        public Triangle(Vector3 a, Vector3 b, Vector3 c) { A = a; B = b; C = c; }
    }

    public sealed class Mesh
    {
        public List<Triangle> Triangles { get; } = new();
    }

    public static class MeshBuilder
    {
        // Dexelから半径(z)を推定して、回転体として三角形化
        public static Mesh BuildRevolvedMeshFromDexel(DexelGrid grid, List<double> zSamples, int radialSegments)
        {
            var mesh = new Mesh();

            // zSamples を昇順、重複除去
            var zs = zSamples.Distinct().OrderBy(z => z).ToList();
            if (zs.Count < 2) zs = new List<double> { 0.0, grid.ZMax };

            // 各zで半径を推定（POC：全列走査）
            var radii = new double[zs.Count];
            for (int i = 0; i < zs.Count; i++)
                radii[i] = grid.EstimateOuterRadiusAtZ(zs[i]);

            // 回転体の外皮（側面）
            for (int i = 0; i < zs.Count - 1; i++)
            {
                double z0 = zs[i], z1 = zs[i + 1];
                double r0 = radii[i], r1 = radii[i + 1];

                // 段差を縦面で出したい場合、同一zを複製してリングを作る等が必要。
                // POCでは「サンプル点が段差位置を含む」前提で、素直に接続。
                AddRevolvedStrip(mesh, z0, r0, z1, r1, radialSegments);
            }

            // キャップ（z=0 と z=ZMax）
            // 半径が0ならスキップ
            AddCap(mesh, z: zs[0], radius: radii[0], radialSegments, isTop: false);
            AddCap(mesh, z: zs[^1], radius: radii[^1], radialSegments, isTop: true);

            return mesh;
        }

        private static void AddRevolvedStrip(Mesh mesh, double z0, double r0, double z1, double r1, int n)
        {
            for (int k = 0; k < n; k++)
            {
                double a0 = 2.0 * Math.PI * (k / (double)n);
                double a1 = 2.0 * Math.PI * ((k + 1) / (double)n);

                var p00 = new Vector3((float)(r0 * Math.Cos(a0)), (float)(r0 * Math.Sin(a0)), (float)z0);
                var p01 = new Vector3((float)(r0 * Math.Cos(a1)), (float)(r0 * Math.Sin(a1)), (float)z0);
                var p10 = new Vector3((float)(r1 * Math.Cos(a0)), (float)(r1 * Math.Sin(a0)), (float)z1);
                var p11 = new Vector3((float)(r1 * Math.Cos(a1)), (float)(r1 * Math.Sin(a1)), (float)z1);

                // quad -> two triangles
                // winding は適当（法線不要）
                mesh.Triangles.Add(new Triangle(p00, p10, p11));
                mesh.Triangles.Add(new Triangle(p00, p11, p01));
            }
        }

        private static void AddCap(Mesh mesh, double z, double radius, int n, bool isTop)
        {
            if (radius <= 1e-9) return;
            var center = new Vector3(0, 0, (float)z);

            for (int k = 0; k < n; k++)
            {
                double a0 = 2.0 * Math.PI * (k / (double)n);
                double a1 = 2.0 * Math.PI * ((k + 1) / (double)n);

                var p0 = new Vector3((float)(radius * Math.Cos(a0)), (float)(radius * Math.Sin(a0)), (float)z);
                var p1 = new Vector3((float)(radius * Math.Cos(a1)), (float)(radius * Math.Sin(a1)), (float)z);

                if (isTop)
                    mesh.Triangles.Add(new Triangle(center, p1, p0));
                else
                    mesh.Triangles.Add(new Triangle(center, p0, p1));
            }
        }
    }

    public static class StlBinaryWriter
    {
        // STL binary: 80-byte header + uint32 triCount + (50 bytes per tri)
        // normal(12 bytes) + v0(12) + v1(12) + v2(12) + attr(2)
        public static void Write(Stream stream, List<Triangle> tris)
        {
            Span<byte> header = stackalloc byte[80];
            header.Clear();
            WriteAscii(header, "DexelTurnPoc");
            stream.Write(header);

            Span<byte> u32 = stackalloc byte[4];
            BinaryPrimitives.WriteUInt32LittleEndian(u32, (uint)tris.Count);
            stream.Write(u32);

            Span<byte> buf = stackalloc byte[50];

            foreach (var t in tris)
            {
                buf.Clear();
                // normal: 0,0,0 （法線不要）
                WriteVec3(buf.Slice(12, 12), t.A);
                WriteVec3(buf.Slice(24, 12), t.B);
                WriteVec3(buf.Slice(36, 12), t.C);
                // attr (2 bytes) already zero
                stream.Write(buf);
            }
        }

        private static void WriteVec3(Span<byte> dst12, Vector3 v)
        {
            BinaryPrimitives.WriteSingleLittleEndian(dst12.Slice(0, 4), v.X);
            BinaryPrimitives.WriteSingleLittleEndian(dst12.Slice(4, 4), v.Y);
            BinaryPrimitives.WriteSingleLittleEndian(dst12.Slice(8, 4), v.Z);
        }

        private static void WriteAscii(Span<byte> dst, string s)
        {
            var bytes = System.Text.Encoding.ASCII.GetBytes(s);
            int n = Math.Min(dst.Length, bytes.Length);
            bytes.AsSpan(0, n).CopyTo(dst);
        }
    }
}
