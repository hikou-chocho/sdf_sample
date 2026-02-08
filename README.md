# SDF Sample - 旋削形状3Dメッシュ生成

このプロジェクトは、Signed Distance Function (SDF) を使用して回転体（旋削加工形状）の3Dメッシュを生成するシステムです。FastAPI WebAPIとC#の両方の実装を提供しています。

## 概要

旋削加工のプロファイル（断面形状）から、Marching Cubesアルゴリズムを用いてSTL形式の3Dメッシュを自動生成します。ガウシアンフィルタによる平滑化処理により、高品質なメッシュ出力が可能です。

### 主な機能

- 軸対称形状の3Dメッシュ生成
- SDFベースの形状表現
- Marching Cubesアルゴリズムによるメッシュ抽出
- ガウシアンフィルタによる表面平滑化
- STL形式でのエクスポート
- メッシュ自動修復機能

## プロジェクト構成

```
sdf_sample/
├── app.py                      # FastAPI Webサーバー (メイン実装)
├── generate_tritabe.py         # Marching Cubesテーブル生成ツール
├── prereq.txt                  # Pythonパッケージ依存関係
├── case3_profile.json          # サンプルプロファイルデータ
├── tritabe.cs                  # C#用三角化テーブル
└── csharp/
    └── ConsoleApp1/            # C#実装版
        ├── ConsoleApp1.sln
        ├── Program.cs
        ├── TiledMarchingCubes.cs
        ├── ContiguousNd3.cs
        └── ...
```

## 必要要件

### Python版

- Python 3.8以上
- 以下のパッケージ（`prereq.txt`参照）：
  - fastapi
  - uvicorn
  - numpy
  - scipy
  - scikit-image
  - trimesh
  - pymeshfix

### C#版

- .NET 8.0以上

## インストール

### Python版のセットアップ

```powershell
# 仮想環境の作成（推奨）
python -m venv venv
.\venv\Scripts\Activate.ps1

# パッケージのインストール
pip install -r prereq.txt
```

### C#版のセットアップ

```powershell
cd csharp\ConsoleApp1
dotnet restore
dotnet build
```

## 使用方法

### Python WebAPI (app.py)

#### サーバーの起動

```powershell
uvicorn app:app --reload
```

デフォルトでは `http://localhost:8000` でサーバーが起動します。

#### APIエンドポイント

**POST /sdf/run**

旋削プロファイルからSTLメッシュを生成します。

リクエスト例:

```json
{
  "units": "mm",
  "output_mode": "stl",
  "pitch_mm": 0.5,
  "stock": {
    "type": "cylinder",
    "params": {
      "dia": 50.0,
      "h": 100.0
    }
  },
  "features": [
    {
      "feature_type": "turn_od_profile",
      "params": {
        "profile": [
          {"z": 10, "radius": 25},
          {"z": 50, "radius": 20},
          {"z": 90, "radius": 25}
        ]
      }
    }
  ]
}
```

パラメータ:
- `units`: 単位（現在は"mm"のみサポート）
- `output_mode`: 出力モード（"stl", "sdf_grid", "both"）
- `pitch_mm`: グリッド解像度（推奨: 0.3～1.0mm）
- `stock`: 素材形状の定義
- `features`: 加工形状の配列

レスポンス: STLファイルのバイナリデータ

#### APIドキュメント

FastAPIの自動生成ドキュメントは以下で確認できます：
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

### C#版

```powershell
cd csharp\ConsoleApp1
dotnet run
```

## 技術詳細

### SDFベースの形状表現

Signed Distance Function (SDF) は、空間内の各点が形状表面からどれだけ離れているかを符号付き距離で表現します：
- 負の値：形状内部
- ゼロ：形状表面
- 正の値：形状外部

### Marching Cubesアルゴリズム

3D空間をボクセルグリッドに分割し、各立方体の8頂点のSDF値から、表面を通過する三角形メッシュを生成します。

### 平滑化処理

ガウシアンフィルタをSDF値に適用することで、滑らかな表面を生成します：
- `sigma`: フィルタのシグマ値（推奨: 0.8～1.5）
- `level_offset`: ISO表面のオフセット値（寸法調整用）

### メッシュ修復

生成されたメッシュは以下の処理で自動修復されます：
1. 最大コンポーネントの抽出
2. 頂点のマージ
3. 未参照頂点の削除
4. 法線の修正
5. pymeshfixによる詳細修復（利用可能な場合）

## パフォーマンス

処理時間の目安（標準的なPC環境）：
- グリッド構築と SDF サンプリング: ~1-3秒
- ガウシアン平滑化: ~0.5-1秒
- Marching Cubes: ~1-2秒
- メッシュ修復: ~0.5-1秒

**合計: 3～7秒程度**

解像度（`pitch_mm`）を細かくすると精度が向上しますが、処理時間も増加します。

## トラブルシューティング

### メッシュが荒い
- `pitch_mm` を小さく設定（例: 0.3mm）
- `sigma` を調整して平滑化を強化

### 寸法が合わない
- `level_offset` パラメータで調整
- 外形が縮む場合: 負の値を試す
- 外形が膨らむ場合: 正の値を試す

### パフォーマンスが遅い
- `pitch_mm` を大きく設定（例: 1.0mm）
- グリッドサイズを確認（出力ログに表示）

## 開発

### ユーティリティスクリプト

**generate_tritabe.py**

Marching Cubesアルゴリズムで使用する三角化テーブルをC#形式で生成します。

```powershell
python generate_tritabe.py
```

## ライセンス

（ライセンス情報をここに記載）

## 参考文献

- Paul Bourke's Marching Cubes Implementation
- [Marching Cubesアルゴリズム](http://paulbourke.net/geometry/polygonise/)
- [Signed Distance Functions](https://iquilezles.org/articles/distfunctions/)

## 貢献

プルリクエストやissueの報告を歓迎します。

---

**更新日**: 2026年2月
