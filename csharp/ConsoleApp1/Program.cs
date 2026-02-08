using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.IO;
using System.Diagnostics;

namespace ConsoleApp1
{
    // Vector2 and Vector3 are from System.Numerics (standard library)

    /// <summary>
    /// ストック定義（円筒のみ V1 対応）
    /// </summary>
    public class Stock
    {
        [JsonPropertyName("type")]
        public string Type { get; set; } = "cylinder";

        [JsonPropertyName("params")]
        public StockParams Params { get; set; } = new();
    }

    public class StockParams
    {
        [JsonPropertyName("dia")]
        public float Dia { get; set; }

        [JsonPropertyName("h")]
        public float H { get; set; }
    }

    /// <summary>
    /// 旋盤加工用のプロファイル（z, r）
    /// </summary>
    public class Profile
    {
        public List<Vector2> Points { get; set; } = new List<Vector2>();

        public Profile(params (float z, float r)[] points)
        {
            Points = points.Select(p => new Vector2(p.z, p.r)).ToList();
        }

        public Profile()
        {
        }
    }

    /// <summary>
    /// Feature: 旋削（turn_od_profile）
    /// </summary>
    public class TurnOdProfile
    {
        [JsonPropertyName("feature_type")]
        public string FeatureType { get; set; } = "turn_od_profile";

        [JsonPropertyName("id")]
        public string Id { get; set; } = "";

        [JsonPropertyName("params")]
        public TurnOdParams Params { get; set; } = new();
    }

    public class TurnOdParams
    {
        [JsonPropertyName("csys_id")]
        public string CsysId { get; set; } = "WCS";

        [JsonPropertyName("profile")]
        public List<ProfilePoint> Profile { get; set; } = new();
    }

    public class ProfilePoint
    {
        [JsonPropertyName("z")]
        public float Z { get; set; }

        [JsonPropertyName("radius")]
        public float Radius { get; set; }
    }

    /// <summary>
    /// JSON 入力スキーマ（case3_profile.json）
    /// </summary>
    public class SdfRequest
    {
        [JsonPropertyName("units")]
        public string Units { get; set; } = "mm";

        [JsonPropertyName("stock")]
        public Stock Stock { get; set; } = new();

        [JsonPropertyName("features")]
        public List<TurnOdProfile> Features { get; set; } = new();

        [JsonPropertyName("pitch_mm")]
        public float PitchMm { get; set; } = 0.5f;

        [JsonPropertyName("output_mode")]
        public string OutputMode { get; set; } = "stl";
    }

    /// <summary>
    /// SDF（Signed Distance Field）計算エンジン
    /// </summary>
    public class SdfCalculator
    {
        /// <summary>
        /// 2D点がポリゴン内にあるかを判定（ray casting）
        /// </summary>
        private static bool PointInPoly2D(Vector2 q, List<Vector2> poly)
        {
            bool inside = false;
            int n = poly.Count;

            for (int i = 0; i < n; i++)
            {
                Vector2 p0 = poly[i];
                Vector2 p1 = poly[(i + 1) % n];

                if ((p0.Y > q.Y) != (p1.Y > q.Y))
                {
                    float xinters = (p1.X - p0.X) * (q.Y - p0.Y) / (p1.Y - p0.Y + 1e-10f) + p0.X;
                    if (q.X < xinters)
                    {
                        inside = !inside;
                    }
                }
            }

            return inside;
        }

        /// <summary>
        /// 点から線分までの距離を計算
        /// </summary>
        private static float DistPointToSegment2D(Vector2 q, Vector2 a, Vector2 b)
        {
            Vector2 ab = b - a;
            float abDot = Vector2.Dot(ab, ab) + 1e-10f;
            float t = Vector2.Dot(q - a, ab) / abDot;
            t = Math.Max(0, Math.Min(1, t));

            Vector2 proj = a + ab * t;
            return (q - proj).Length();
        }

        /// <summary>
        /// 平面プロファイルに対するSDF計算（2D）
        /// </summary>
        public static float ComputeSDF2D(Vector2 q, Profile profile)
        {
            float dmin = float.MaxValue;
            int n = profile.Points.Count;

            for (int i = 0; i < n; i++)
            {
                Vector2 a = profile.Points[i];
                Vector2 b = profile.Points[(i + 1) % n];
                dmin = Math.Min(dmin, DistPointToSegment2D(q, a, b));
            }

            bool inside = PointInPoly2D(q, profile.Points);
            return inside ? -dmin : dmin;
        }

        /// <summary>
        /// 軸対称体の3D SDF計算（回転対称プロファイルから）
        /// </summary>
        public static float ComputeSDF3D_Axisymmetric(Vector3 p, Profile profile)
        {
            // 点をローカル座標に変換
            float z = p.Z;
            float r = (float)Math.Sqrt(p.X * p.X + p.Y * p.Y);

            // z-r 平面でのSDF計算
            Vector2 q = new Vector2(z, r);
            return ComputeSDF2D(q, profile);
        }

        /// <summary>
        /// ストック（円筒）のみのSDF計算
        /// 点がストック内：負、ストック外：正
        /// </summary>
        public static float ComputeStockSDF(Vector3 p, float dia, float h)
        {
            float r = (float)Math.Sqrt(p.X * p.X + p.Y * p.Y);
            float R = dia * 0.5f;
            
            // 点が円筒内にあるかチェック
            bool insideZ = p.Z >= 0 && p.Z <= h;
            bool insideR = r <= R;
            
            if (insideZ && insideR)
            {
                // 点はストック内 → 負値を返す（ストック境界までの距離）
                float distToR = R - r;      // 半径方向の距離
                float distToZ0 = p.Z;        // 底面までの距離
                float distToZ1 = h - p.Z;    // 上面までの距離
                float distToBoundary = Math.Min(Math.Min(distToR, distToZ0), distToZ1);
                return -distToBoundary;
            }
            else
            {
                // 点はストック外 → 正値を返す
                float distR = r - R;  // 半径オーバー分
                float distZ0 = -p.Z;  // 下側オーバー分
                float distZ1 = p.Z - h;  // 上側オーバー分
                
                distR = Math.Max(0, distR);
                distZ0 = Math.Max(0, distZ0);
                distZ1 = Math.Max(0, distZ1);
                
                if (distR > 0 && (distZ0 > 0 || distZ1 > 0))
                {
                    // コーナー部分
                    return (float)Math.Sqrt(distR * distR + Math.Max(distZ0, distZ1) * Math.Max(distZ0, distZ1));
                }
                else if (distR > 0)
                {
                    // 側面からはみ出し
                    return distR;
                }
                else
                {
                    // Z方向からはみ出し
                    return Math.Max(distZ0, distZ1);
                }
            }
        }

        /// <summary>
        /// 複数点の SDF を一括評価（ベクトル化版）
        /// </summary>
        public static void EvalSdfBatch(Vector3[] points, float[] output, 
            Func<Vector3, float> evalFunc)
        {
            if (points.Length == 0) return;
            
            for (int i = 0; i < points.Length; i++)
            {
                output[i] = evalFunc(points[i]);
            }
        }
    }

    /// <summary>
    /// エントリポイント
    /// </summary>
    public class Program
    {
        public static void Main(string[] args)
        {
            var sw = Stopwatch.StartNew();

            try
            {
                if (args.Length < 1)
                {
                    PrintUsage();
                    return;
                }

                string inputFile = args[0];
                string outputFile = args.Length > 1 ? args[1] : Path.ChangeExtension(inputFile, ".stl");
                float pitchMm = 0.5f;
                int threads = Environment.ProcessorCount;

                // Parse CLI args
                for (int i = 2; i < args.Length; i++)
                {
                    if (args[i] == "--pitch" && i + 1 < args.Length)
                    {
                        float.TryParse(args[i + 1], out pitchMm);
                        i++;
                    }
                    else if (args[i] == "--threads" && i + 1 < args.Length)
                    {
                        int.TryParse(args[i + 1], out threads);
                        i++;
                    }
                }

                Console.WriteLine("=== C# SDF to STL Converter ===\n");
                Console.WriteLine($"Input:  {inputFile}");
                Console.WriteLine($"Output: {outputFile}");
                Console.WriteLine($"Pitch:  {pitchMm} mm");
                Console.WriteLine($"Threads: {threads}\n");

                // Load JSON
                Console.WriteLine("[1/6] JSON 読み込み中...");
                var jsonText = File.ReadAllText(inputFile);
                var options = new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.CamelCase };
                var req = JsonSerializer.Deserialize<SdfRequest>(jsonText, options);
                
                if (req == null)
                {
                    Console.WriteLine("ERROR: Failed to parse JSON");
                    return;
                }

                Console.WriteLine($"  Stock: {req.Stock.Type} (dia={req.Stock.Params.Dia}, h={req.Stock.Params.H})");
                Console.WriteLine($"  Features: {req.Features.Count}\n");

                // Create SDF evaluator
                Console.WriteLine("[2/6] SDF エバリュエーター生成中...");
                var evalSdf = MakeSdfEvaluator(req);

                // Generate bounds
                float R = req.Stock.Params.Dia * 0.5f;
                float pad = 2.0f * pitchMm;
                var boundsMin = new Vector3(-R - pad, -R - pad, -pad);
                var boundsMax = new Vector3(R + pad, R + pad, req.Stock.Params.H + pad);

                Console.WriteLine($"  Bounds: [{boundsMin.X:F1}, {boundsMin.Y:F1}, {boundsMin.Z:F1}] to " +
                    $"[{boundsMax.X:F1}, {boundsMax.Y:F1}, {boundsMax.Z:F1}]\n");

                // Generate STL
                Console.WriteLine("[3/6] グリッド生成中...");
                sw.Restart();
                var gridBuildSw = Stopwatch.StartNew();
                var stlData = GenerateStl(evalSdf, boundsMin, boundsMax, pitchMm, threads);
                gridBuildSw.Stop();
                Console.WriteLine($"  完了: {gridBuildSw.ElapsedMilliseconds}ms\n");

                // Write STL
                Console.WriteLine("[4/6] STL ファイル書き込み中...");
                File.WriteAllBytes(outputFile, stlData);
                Console.WriteLine($"  File written: {new FileInfo(outputFile).Length} bytes\n");

                Console.WriteLine("[5/6] 処理完了");
                Console.WriteLine($"総実行時間: {sw.ElapsedMilliseconds}ms");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"ERROR: {ex.Message}");
                Console.WriteLine(ex.StackTrace);
            }
        }

        private static void PrintUsage()
        {
            Console.WriteLine("Usage: ConsoleApp1 <input.json> [output.stl] [--pitch 0.5] [--threads 4]");
            Console.WriteLine("\nExample:");
            Console.WriteLine("  ConsoleApp1 case3_profile.json output.stl --pitch 0.5 --threads 8");
        }

        private static Func<Vector3, float> MakeSdfEvaluator(SdfRequest req)
        {
            float dia = req.Stock.Params.Dia;
            float h = req.Stock.Params.H;

            // Find turn_od_profile feature
            var turnFeature = req.Features.FirstOrDefault(f => f.FeatureType == "turn_od_profile");

            if (turnFeature == null)
            {
                // Stock only
                return p => SdfCalculator.ComputeStockSDF(p, dia, h);
            }

            // Build profile from JSON
            var profile = new Profile();
            foreach (var pt in turnFeature.Params.Profile)
            {
                profile.Points.Add(new Vector2(pt.Z, pt.Radius));
            }

            // Close the profile: add axis return and bottom closure
            if (profile.Points.Count > 0)
            {
                // Last point should return to axis (z, 0)
                var lastPt = profile.Points[profile.Points.Count - 1];
                profile.Points.Add(new Vector2(lastPt.X, 0)); // Return to axis

                // Return to start z-position on axis
                var firstPt = profile.Points[0];
                if (lastPt.X != firstPt.X)  // If not already at start z
                {
                    profile.Points.Add(new Vector2(firstPt.X, 0)); // Move to start z on axis
                }

                // Close back to first point
                profile.Points.Add(firstPt);  // Explicit closure
            }

            // Debug: Print profile
            Console.WriteLine($"  Profile points loaded: {profile.Points.Count}");
            for (int i = 0; i < profile.Points.Count; i++)
            {
                Console.WriteLine($"    [{i}] z={profile.Points[i].X:F1}, r={profile.Points[i].Y:F1}");
            }

            // Test SDF at a few points
            Console.WriteLine("  Test SDF values:");
            var testPoints = new[] 
            { 
                new Vector3(20, 0, 10),  // r=20, z=10 (should be inside at step 1)
                new Vector3(25, 0, 10),  // r=25, z=10 (on boundary at z=0-20)
                new Vector3(22, 0, 30),  // r=22, z=30 (between step 1 and step 2)
                new Vector3(25, 0, 0),   // r=25, z=0 (on start profile)
            };
            foreach( var tp in testPoints)
            {
                float r = (float)Math.Sqrt(tp.X * tp.X + tp.Y * tp.Y);
                float sdfStock = SdfCalculator.ComputeStockSDF(tp, dia, h);
                float sdfTurn = SdfCalculator.ComputeSDF3D_Axisymmetric(tp, profile);
                float sdfComposite = Math.Max(sdfStock, sdfTurn);  // Intersection: A AND B → max(A,B)
                Console.WriteLine($"    p=({tp.X:F1}, {tp.Y:F1}, {tp.Z:F1}): r={r:F1}, sdfStock={sdfStock:F2}, sdfTurn={sdfTurn:F2}, composite={sdfComposite:F2}");
            }

            // Return composite SDF: max(stock, turn_profile) = keep material if BOTH are negative
            // Point is inside if: sdfStock < 0 AND sdfTurn < 0
            return p =>
            {
                float sdfStock = SdfCalculator.ComputeStockSDF(p, dia, h);
                float sdfTurn = SdfCalculator.ComputeSDF3D_Axisymmetric(p, profile);
                return Math.Max(sdfStock, sdfTurn);
            };
        }

        private static byte[] GenerateStl(Func<Vector3, float> evalSdf, 
            Vector3 boundsMin, Vector3 boundsMax, float pitch, int maxThreads)
        {
            // 1. Generate grid coordinates
            var size = (boundsMax - boundsMin) / pitch;
            int nx = (int)Math.Ceiling(size.X) + 1;
            int ny = (int)Math.Ceiling(size.Y) + 1;
            int nz = (int)Math.Ceiling(size.Z) + 1;

            Console.WriteLine($"    Grid: {nx} × {ny} × {nz} = {(long)nx * ny * nz} points");

            // 2. Sample SDF grid
            float[] sdfGrid = new float[nx * ny * nz];
            var gridXs = Enumerable.Range(0, nx).Select(i => boundsMin.X + i * pitch).ToArray();
            var gridYs = Enumerable.Range(0, ny).Select(i => boundsMin.Y + i * pitch).ToArray();
            var gridZs = Enumerable.Range(0, nz).Select(i => boundsMin.Z + i * pitch).ToArray();

            Console.WriteLine($"    Sampling SDF...");
            var sampleSw = Stopwatch.StartNew();

            // Parallel sampling by x-slices
            var opt = new ParallelOptions { MaxDegreeOfParallelism = maxThreads };
            Parallel.For(0, nx, opt, x =>
            {
                for (int y = 0; y < ny; y++)
                {
                    for (int z = 0; z < nz; z++)
                    {
                        var p = new Vector3(gridXs[x], gridYs[y], gridZs[z]);
                        int idx = x + nx * (y + ny * z);
                        sdfGrid[idx] = evalSdf(p);
                    }
                }
            });

            sampleSw.Stop();
            Console.WriteLine($"    SDF sampling done: {sampleSw.ElapsedMilliseconds}ms");

            // 2.5 Apply Gaussian smoothing (sigma=1.0f)
            Console.WriteLine($"    Applying Gaussian filter (sigma=1.0)...");
            var filterSw = Stopwatch.StartNew();
            float[] sdfGridSmoothed = new float[nx * ny * nz];
            GaussianFilter.ApplySeparableGaussian3D(sdfGrid, nx, ny, nz, sigma: 1.0f, sdfGridSmoothed);
            filterSw.Stop();
            Console.WriteLine($"    Gaussian filter done: {filterSw.ElapsedMilliseconds}ms");

            // 3. Marching Cubes
            Console.WriteLine($"    Running Marching Cubes...");
            var mcSw = Stopwatch.StartNew();

            TiledMarchingCubes.BuildMeshTiled(
                sdfGridSmoothed, nx, ny, nz,
                iso: 0.0f,
                out List<Vector3> positions,
                out List<int> indices,
                maxDegreeOfParallelism: maxThreads);

            mcSw.Stop();
            Console.WriteLine($"    MC done: {mcSw.ElapsedMilliseconds}ms");
            Console.WriteLine($"    Vertices: {positions.Count}, Triangles: {indices.Count / 3}");

            // 4. Convert from grid space to world space
            var worldPositions = new Vector3[positions.Count];
            for (int i = 0; i < positions.Count; i++)
            {
                var gp = positions[i];
                worldPositions[i] = new Vector3(
                    boundsMin.X + gp.X,
                    boundsMin.Y + gp.Y,
                    boundsMin.Z + gp.Z
                );
            }

            // 5. Export to Binary STL
            return ExportBinaryStl(worldPositions, indices);
        }

        private static byte[] ExportBinaryStl(Vector3[] vertices, List<int> indices)
        {
            int triangleCount = indices.Count / 3;
            int fileSize = 80 + 4 + triangleCount * 50; // header + count + triangles

            using (var ms = new System.IO.MemoryStream(fileSize))
            {
                using (var bw = new System.IO.BinaryWriter(ms))
                {
                    // Header (80 bytes)
                    byte[] header = new byte[80];
                    var headerText = "Binary STL generated by C# SDF Converter";
                    System.Text.Encoding.ASCII.GetBytes(headerText, 0, 
                        Math.Min(headerText.Length, 80), header, 0);
                    bw.Write(header);

                    // Triangle count
                    bw.Write(triangleCount);

                    // Triangles
                    for (int i = 0; i < triangleCount; i++)
                    {
                        int i0 = indices[i * 3];
                        int i1 = indices[i * 3 + 1];
                        int i2 = indices[i * 3 + 2];

                        Vector3 v0 = vertices[i0];
                        Vector3 v1 = vertices[i1];
                        Vector3 v2 = vertices[i2];

                        // Normal
                        Vector3 e1 = v1 - v0;
                        Vector3 e2 = v2 - v0;
                        Vector3 normal = Vector3.Normalize(Vector3.Cross(e1, e2));

                        bw.Write(normal.X);
                        bw.Write(normal.Y);
                        bw.Write(normal.Z);

                        bw.Write(v0.X);
                        bw.Write(v0.Y);
                        bw.Write(v0.Z);

                        bw.Write(v1.X);
                        bw.Write(v1.Y);
                        bw.Write(v1.Z);

                        bw.Write(v2.X);
                        bw.Write(v2.Y);
                        bw.Write(v2.Z);

                        bw.Write((ushort)0); // Attribute byte count
                    }
                }

                return ms.ToArray();
            }
        }
    }
}