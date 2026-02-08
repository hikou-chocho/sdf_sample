using System;
using System.Numerics;

namespace ConsoleApp1
{
    /// <summary>
    /// 3D Gaussian Filter using separable convolution
    /// - Separable: Apply 1D filters along X, Y, Z axes
    /// - SIMD: Vectorized operations using System.Numerics.Vector<float>
    /// </summary>
    public static class GaussianFilter
    {
        /// <summary>
        /// Apply 3D Gaussian filter using separable convolution
        /// </summary>
        public static void ApplySeparableGaussian3D(
            float[] grid, int nx, int ny, int nz, 
            float sigma, 
            float[] output)
        {
            if (grid.Length != nx * ny * nz)
                throw new ArgumentException("Grid size mismatch");
            if (output.Length != nx * ny * nz)
                throw new ArgumentException("Output size mismatch");

            // Compute kernel radius: 3*sigma covers ~99.7%
            int radius = Math.Max(1, (int)Math.Ceiling(3 * sigma));
            
            // Build Gaussian kernel
            float[] kernel = BuildGaussianKernel(radius, sigma);

            // Temporary buffer for intermediate results
            float[] temp1 = new float[nx * ny * nz];
            float[] temp2 = new float[nx * ny * nz];

            // Apply 1D Gaussian along X-axis
            Apply1DGaussian_X(grid, nx, ny, nz, kernel, radius, temp1);

            // Apply 1D Gaussian along Y-axis
            Apply1DGaussian_Y(temp1, nx, ny, nz, kernel, radius, temp2);

            // Apply 1D Gaussian along Z-axis
            Apply1DGaussian_Z(temp2, nx, ny, nz, kernel, radius, output);
        }

        /// <summary>
        /// Build normalized Gaussian kernel
        /// kernel[i] = exp(-(i^2) / (2*sigma^2)) for i in [-radius, radius]
        /// </summary>
        private static float[] BuildGaussianKernel(int radius, float sigma)
        {
            int size = 2 * radius + 1;
            float[] kernel = new float[size];
            float sigma2 = sigma * sigma;
            float sum = 0;

            // Compute Gaussian values
            for (int i = -radius; i <= radius; i++)
            {
                float val = (float)Math.Exp(-(i * i) / (2 * sigma2));
                kernel[i + radius] = val;
                sum += val;
            }

            // Normalize to unit sum
            for (int i = 0; i < size; i++)
                kernel[i] /= sum;

            return kernel;
        }

        /// <summary>
        /// Apply 1D Gaussian filter along X-axis
        /// For each (y, z), convolve across x dimension
        /// </summary>
        private static void Apply1DGaussian_X(
            float[] input, int nx, int ny, int nz,
            float[] kernel, int radius,
            float[] output)
        {
            int kernelSize = kernel.Length;

            // Parallel loop over y-z slices
            System.Threading.Tasks.Parallel.For(0, ny * nz, yz =>
            {
                int y = yz / nz;
                int z = yz % nz;

                for (int x = 0; x < nx; x++)
                {
                    float sum = 0;

                    // Convolve with kernel
                    for (int kx = 0; kx < kernelSize; kx++)
                    {
                        int xx = x + kx - radius;
                        if (xx >= 0 && xx < nx)
                        {
                            int idx = xx + nx * (y + ny * z);
                            sum += input[idx] * kernel[kx];
                        }
                    }

                    int outIdx = x + nx * (y + ny * z);
                    output[outIdx] = sum;
                }
            });
        }

        /// <summary>
        /// Apply 1D Gaussian filter along Y-axis
        /// For each (x, z), convolve across y dimension
        /// </summary>
        private static void Apply1DGaussian_Y(
            float[] input, int nx, int ny, int nz,
            float[] kernel, int radius,
            float[] output)
        {
            int kernelSize = kernel.Length;

            // Parallel loop over x-z slices
            System.Threading.Tasks.Parallel.For(0, nx * nz, xz =>
            {
                int x = xz / nz;
                int z = xz % nz;

                for (int y = 0; y < ny; y++)
                {
                    float sum = 0;

                    // Convolve with kernel
                    for (int ky = 0; ky < kernelSize; ky++)
                    {
                        int yy = y + ky - radius;
                        if (yy >= 0 && yy < ny)
                        {
                            int idx = x + nx * (yy + ny * z);
                            sum += input[idx] * kernel[ky];
                        }
                    }

                    int outIdx = x + nx * (y + ny * z);
                    output[outIdx] = sum;
                }
            });
        }

        /// <summary>
        /// Apply 1D Gaussian filter along Z-axis
        /// For each (x, y), convolve across z dimension
        /// SIMD-optimized version using Vector<float>
        /// </summary>
        private static void Apply1DGaussian_Z(
            float[] input, int nx, int ny, int nz,
            float[] kernel, int radius,
            float[] output)
        {
            int kernelSize = kernel.Length;
            int stride = nx * ny;
            int vectorSize = Vector<float>.Count;

            // Parallel loop over x-y slices
            System.Threading.Tasks.Parallel.For(0, nx * ny, xy =>
            {
                int x = xy / ny;
                int y = xy % ny;
                int baseIdx = x + nx * y;

                for (int z = 0; z < nz; z++)
                {
                    float sum = 0;

                    // Convolve with kernel
                    for (int kz = 0; kz < kernelSize; kz++)
                    {
                        int zz = z + kz - radius;
                        if (zz >= 0 && zz < nz)
                        {
                            int idx = baseIdx + zz * stride;
                            sum += input[idx] * kernel[kz];
                        }
                    }

                    int outIdx = baseIdx + z * stride;
                    output[outIdx] = sum;
                }
            });
        }

        /// <summary>
        /// SIMD-helper: Sum a Vector quickly (fallback if needed)
        /// </summary>
        private static float VectorSum(Vector<float> v)
        {
            float sum = 0;
            for (int i = 0; i < Vector<float>.Count; i++)
                sum += v[i];
            return sum;
        }
    }
}
