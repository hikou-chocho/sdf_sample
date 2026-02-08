using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System;
using System.Numerics;

namespace ConsoleApp1
{

    // NOTE: Usage cautions (SDF-focused)
    // 注意: 使用上の注意 (SDF用途)
    // 1) Always contiguous memory (float[]). No stride/view/non-contiguous slices.
    //    常に連続メモリ前提。stride/view/非連続スライスは非対応。
    //    Transpose or sparse slices require explicit copies.
    //    転置や間引きスライスはコピーが必要。
    // 2) Avoid intermediate arrays. Prefer out params or in-place ops to reduce memory bandwidth.
    //    中間配列は作らず out / in-place でメモリ帯域を節約。
    // 3) 3D indexer is convenient but slow; hot loops should use AsSpan() with a single for-loop.
    //    3D indexer は遅いのでホットパスは AsSpan() + 単一ループで回す。
    // 4) Do not build meshgrid-like arrays. Compute coordinates on the fly in nested loops.
    //    meshgrid は作らず、座標はネストループ内で都度計算。
    // 5) SIMD-friendly design: consider vectorized EvalSdf for inner loops.
    //    内側ループは SIMD 化 (ベクトル版 EvalSdf など) を検討。
    // 6) Memory grows fast: 512^3 float is ~512MB; multiple buffers quickly hit GBs.
    //    512^3 float は約512MB。複数バッファですぐ数GB。
    // 7) Parallel.For over x is safe if each thread writes disjoint indices only.
    //    Parallel.For(x) は各スレッドが別領域に書く前提なら安全。
    // 8) float is recommended for SDF; double doubles memory and halves SIMD width.
    //    SDFは基本 float 推奨。double はメモリ2倍/SIMD幅半減。
    // 9) reshape/ravel style features are intentionally minimal; AsSpan() is the flat view.
    //    reshape/ravel 相当は最小限。フラットは AsSpan()。
    // 10) Not NumPy-compatible: no broadcasting, fancy indexing, or non-contiguous views.
    //     NumPy互換ではない (broadcasting/高度indexing/非連続ビューは不可)。
    // Most important: keep "one kernel, one loop" to avoid temporary buffers.
    // 最重要: 「1カーネル1ループ」で中間バッファを避ける。
    public sealed class ContiguousNd3
    {
        // shape
        public int Nx { get; }
        public int Ny { get; }
        public int Nz { get; }
        public int Length => Nx * Ny * Nz;

        // flat buffer (C-order: x fastest? ここでは z fastest にして [x,y,z] -> ((x*Ny)+y)*Nz+z)
        // 好みで変えてOK。重要なのは一貫性と連続性。
        public float[] Buffer { get; }

        public ContiguousNd3(int nx, int ny, int nz)
        {
            if (nx <= 0 || ny <= 0 || nz <= 0) throw new ArgumentOutOfRangeException();
            Nx = nx; Ny = ny; Nz = nz;
            Buffer = new float[Length];
        }

        // 3D indexer（ホットパスでは使わない想定）
        public float this[int x, int y, int z]
        {
            get => Buffer[ToIndex(x, y, z)];
            set => Buffer[ToIndex(x, y, z)] = value;
        }

        public int ToIndex(int x, int y, int z)
        {
            // bounds check（必要なら外す/DebugのみでもOK）
            if ((uint)x >= (uint)Nx || (uint)y >= (uint)Ny || (uint)z >= (uint)Nz)
                throw new IndexOutOfRangeException();

            // idx = (x*Ny + y)*Nz + z
            return ((x * Ny) + y) * Nz + z;
        }

        public Span<float> AsSpan() => Buffer.AsSpan();

        // ---- Ops（SDF用途の基本）----

        // dst = min(a, b)
        public static void Min(ContiguousNd3 a, ContiguousNd3 b, ContiguousNd3 dst)
        {
            RequireSameShape(a, b);
            RequireSameShape(a, dst);

            var sa = a.AsSpan();
            var sb = b.AsSpan();
            var sd = dst.AsSpan();

            int n = sa.Length;
            int w = Vector<float>.Count;
            int i = 0;

            // SIMD fast path
            for (; i <= n - w; i += w)
            {
                var va = new Vector<float>(sa.Slice(i, w));
                var vb = new Vector<float>(sb.Slice(i, w));
                Vector.Min(va, vb).CopyTo(sd.Slice(i, w));
            }

            // tail
            for (; i < n; i++)
                sd[i] = sa[i] < sb[i] ? sa[i] : sb[i];
        }

        // dst = a + b
        public static void Add(ContiguousNd3 a, ContiguousNd3 b, ContiguousNd3 dst)
        {
            RequireSameShape(a, b);
            RequireSameShape(a, dst);

            var sa = a.AsSpan();
            var sb = b.AsSpan();
            var sd = dst.AsSpan();

            int n = sa.Length;
            int w = Vector<float>.Count;
            int i = 0;

            for (; i <= n - w; i += w)
            {
                var va = new Vector<float>(sa.Slice(i, w));
                var vb = new Vector<float>(sb.Slice(i, w));
                (va + vb).CopyTo(sd.Slice(i, w));
            }

            for (; i < n; i++)
                sd[i] = sa[i] + sb[i];
        }

        // in-place: a = sqrt(a)
        public static void SqrtInPlace(ContiguousNd3 a)
        {
            var s = a.AsSpan();
            for (int i = 0; i < s.Length; i++)
                s[i] = MathF.Sqrt(s[i]);
        }

        // fill
        public void Fill(float value) => AsSpan().Fill(value);

        // optional: get a Z-slice span copy (デバッグ用、コピー前提)
        public float[] ExtractSliceZ(int z)
        {
            if ((uint)z >= (uint)Nz) throw new ArgumentOutOfRangeException(nameof(z));
            var slice = new float[Nx * Ny];
            int p = 0;
            for (int x = 0; x < Nx; x++)
                for (int y = 0; y < Ny; y++)
                    slice[p++] = Buffer[((x * Ny) + y) * Nz + z];
            return slice;
        }

        private static void RequireSameShape(ContiguousNd3 a, ContiguousNd3 b)
        {
            if (a.Nx != b.Nx || a.Ny != b.Ny || a.Nz != b.Nz)
                throw new ArgumentException("shape mismatch");
        }
    }

}
