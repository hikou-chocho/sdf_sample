using System.Numerics;

namespace ConsoleApp1
{

    public static class TiledMarchingCubes
    {
        // タイル（セル空間）のサイズ
        public const int TileCx = 64;
        public const int TileCy = 64;
        public const int TileCz = 64;

        /// <summary>
        /// voxel: サイズ (nx, ny, nz) の 1D 配列。等方スケールなので座標は (x,y,z) をそのまま使う。
        /// 出力: WPF用 (Positions, Indices)。タイル境界の共有はしない（重複OK）。
        /// </summary>
        public static void BuildMeshTiled(
            float[] voxel, int nx, int ny, int nz,
            float iso,
            out List<Vector3> positions,
            out List<int> indices,
            int maxDegreeOfParallelism = -1)
        {
            if (nx < 2 || ny < 2 || nz < 2) throw new ArgumentException("Voxel dims must be >= 2.");
            int cx = nx - 1, cy = ny - 1, cz = nz - 1; // cell dims

            int tilesX = (cx + TileCx - 1) / TileCx;
            int tilesY = (cy + TileCy - 1) / TileCy;
            int tilesZ = (cz + TileCz - 1) / TileCz;
            int tileCount = tilesX * tilesY * tilesZ;

            // タイル結果を格納（後で順次マージするので並列中に共有リストへ触らない）
            var tileResults = new TileMesh[tileCount];

            var opt = new ParallelOptions();
            if (maxDegreeOfParallelism > 0) opt.MaxDegreeOfParallelism = maxDegreeOfParallelism;

            Parallel.For(0, tileCount, opt, ti =>
            {
                int tz = ti / (tilesX * tilesY);
                int rem = ti - tz * (tilesX * tilesY);
                int ty = rem / tilesX;
                int tx = rem - ty * tilesX;

                int x0 = tx * TileCx;
                int y0 = ty * TileCy;
                int z0 = tz * TileCz;

                int x1 = Math.Min(x0 + TileCx, cx);
                int y1 = Math.Min(y0 + TileCy, cy);
                int z1 = Math.Min(z0 + TileCz, cz);

                tileResults[ti] = BuildTile(voxel, nx, ny, nz, iso, x0, y0, z0, x1, y1, z1);
            });

            // --- マージ（重複OKなので単純連結でOK） ---
            positions = new List<Vector3>(EstimateTotalVertices(tileResults));
            indices = new List<int>(EstimateTotalIndices(tileResults));

            int baseVertex = 0;
            foreach (var tm in tileResults)
            {
                if (tm == null) continue;
                positions.AddRange(tm.Positions);
                for (int i = 0; i < tm.Indices.Count; i++)
                    indices.Add(tm.Indices[i] + baseVertex);

                baseVertex += tm.Positions.Count;
            }
        }

        // タイル生成（タイル内は辺共有して頂点を減らす）
        private static TileMesh BuildTile(
            float[] v, int nx, int ny, int nz, float iso,
            int x0, int y0, int z0, int x1, int y1, int z1)
        {
            int tcx = x1 - x0; // tile cell dims (<=64)
            int tcy = y1 - y0;
            int tcz = z1 - z0;

            // タイル内の “辺→頂点index” キャッシュ（タイルローカル。境界共有はしない）
            // X辺: tcx * (tcy+1) * (tcz+1)
            // Y辺: (tcx+1) * tcy * (tcz+1)
            // Z辺: (tcx+1) * (tcy+1) * tcz
            int xEdgeCount = tcx * (tcy + 1) * (tcz + 1);
            int yEdgeCount = (tcx + 1) * tcy * (tcz + 1);
            int zEdgeCount = (tcx + 1) * (tcy + 1) * tcz;

            var xEdge = new int[xEdgeCount];
            var yEdge = new int[yEdgeCount];
            var zEdge = new int[zEdgeCount];
            Array.Fill(xEdge, -1);
            Array.Fill(yEdge, -1);
            Array.Fill(zEdge, -1);

            var mesh = new TileMesh();

            // セル走査（グローバルセル座標 x,y,z）
            for (int z = z0; z < z1; z++)
                for (int y = y0; y < y1; y++)
                    for (int x = x0; x < x1; x++)
                    {
                        // 8頂点の値（ボクセル座標）
                        float v0 = V(v, nx, ny, nz, x, y, z);
                        float v1 = V(v, nx, ny, nz, x + 1, y, z);
                        float v2 = V(v, nx, ny, nz, x + 1, y + 1, z);
                        float v3 = V(v, nx, ny, nz, x, y + 1, z);
                        float v4 = V(v, nx, ny, nz, x, y, z + 1);
                        float v5 = V(v, nx, ny, nz, x + 1, y, z + 1);
                        float v6 = V(v, nx, ny, nz, x + 1, y + 1, z + 1);
                        float v7 = V(v, nx, ny, nz, x, y + 1, z + 1);

                        int cubeIndex = 0;
                        if (v0 < iso) cubeIndex |= 1;
                        if (v1 < iso) cubeIndex |= 2;
                        if (v2 < iso) cubeIndex |= 4;
                        if (v3 < iso) cubeIndex |= 8;
                        if (v4 < iso) cubeIndex |= 16;
                        if (v5 < iso) cubeIndex |= 32;
                        if (v6 < iso) cubeIndex |= 64;
                        if (v7 < iso) cubeIndex |= 128;

                        int edgeMask = McTables.EdgeTable[cubeIndex];
                        if (edgeMask == 0) continue;

                        // タイルローカルセル座標（0..tcx-1 等）
                        int lx = x - x0;
                        int ly = y - y0;
                        int lz = z - z0;

                        // セルの12辺に対応する頂点index（生成 or 再利用）
                        Span<int> evi = stackalloc int[12];
                        for (int i = 0; i < 12; i++) evi[i] = -1;

                        // edge 0: v0-v1 (X) at (x,y,z)
                        if ((edgeMask & 1) != 0)
                            evi[0] = GetOrCreateX(mesh, xEdge, iso, v0, v1,
                                new Vector3(x, y, z), new Vector3(x + 1, y, z),
                                XEdgeId(lx, ly, lz, tcx, tcy));

                        // edge 1: v1-v2 (Y) at (x+1,y,z)
                        if ((edgeMask & 2) != 0)
                            evi[1] = GetOrCreateY(mesh, yEdge, iso, v1, v2,
                                new Vector3(x + 1, y, z), new Vector3(x + 1, y + 1, z),
                                YEdgeId(lx + 1, ly, lz, tcx, tcy));

                        // edge 2: v2-v3 (X) at (x,y+1,z)  (note values: v3->v2 along +X)
                        if ((edgeMask & 4) != 0)
                            evi[2] = GetOrCreateX(mesh, xEdge, iso, v3, v2,
                                new Vector3(x, y + 1, z), new Vector3(x + 1, y + 1, z),
                                XEdgeId(lx, ly + 1, lz, tcx, tcy));

                        // edge 3: v3-v0 (Y) at (x,y,z) (values: v0->v3 along +Y)
                        if ((edgeMask & 8) != 0)
                            evi[3] = GetOrCreateY(mesh, yEdge, iso, v0, v3,
                                new Vector3(x, y, z), new Vector3(x, y + 1, z),
                                YEdgeId(lx, ly, lz, tcx, tcy));

                        // edge 4: v4-v5 (X) at (x,y,z+1)
                        if ((edgeMask & 16) != 0)
                            evi[4] = GetOrCreateX(mesh, xEdge, iso, v4, v5,
                                new Vector3(x, y, z + 1), new Vector3(x + 1, y, z + 1),
                                XEdgeId(lx, ly, lz + 1, tcx, tcy));

                        // edge 5: v5-v6 (Y) at (x+1,y,z+1)
                        if ((edgeMask & 32) != 0)
                            evi[5] = GetOrCreateY(mesh, yEdge, iso, v5, v6,
                                new Vector3(x + 1, y, z + 1), new Vector3(x + 1, y + 1, z + 1),
                                YEdgeId(lx + 1, ly, lz + 1, tcx, tcy));

                        // edge 6: v6-v7 (X) at (x,y+1,z+1) (values: v7->v6 along +X)
                        if ((edgeMask & 64) != 0)
                            evi[6] = GetOrCreateX(mesh, xEdge, iso, v7, v6,
                                new Vector3(x, y + 1, z + 1), new Vector3(x + 1, y + 1, z + 1),
                                XEdgeId(lx, ly + 1, lz + 1, tcx, tcy));

                        // edge 7: v7-v4 (Y) at (x,y,z+1) (values: v4->v7 along +Y)
                        if ((edgeMask & 128) != 0)
                            evi[7] = GetOrCreateY(mesh, yEdge, iso, v4, v7,
                                new Vector3(x, y, z + 1), new Vector3(x, y + 1, z + 1),
                                YEdgeId(lx, ly, lz + 1, tcx, tcy));

                        // edge 8: v0-v4 (Z) at (x,y,z)
                        if ((edgeMask & 256) != 0)
                            evi[8] = GetOrCreateZ(mesh, zEdge, iso, v0, v4,
                                new Vector3(x, y, z), new Vector3(x, y, z + 1),
                                ZEdgeId(lx, ly, lz, tcx, tcy));

                        // edge 9: v1-v5 (Z) at (x+1,y,z)
                        if ((edgeMask & 512) != 0)
                            evi[9] = GetOrCreateZ(mesh, zEdge, iso, v1, v5,
                                new Vector3(x + 1, y, z), new Vector3(x + 1, y, z + 1),
                                ZEdgeId(lx + 1, ly, lz, tcx, tcy));

                        // edge 10: v2-v6 (Z) at (x+1,y+1,z)
                        if ((edgeMask & 1024) != 0)
                            evi[10] = GetOrCreateZ(mesh, zEdge, iso, v2, v6,
                                new Vector3(x + 1, y + 1, z), new Vector3(x + 1, y + 1, z + 1),
                                ZEdgeId(lx + 1, ly + 1, lz, tcx, tcy));

                        // edge 11: v3-v7 (Z) at (x,y+1,z)
                        if ((edgeMask & 2048) != 0)
                            evi[11] = GetOrCreateZ(mesh, zEdge, iso, v3, v7,
                                new Vector3(x, y + 1, z), new Vector3(x, y + 1, z + 1),
                                ZEdgeId(lx, ly + 1, lz, tcx, tcy));

                        // triTable -> Indices
                        for (int t = 0; t < 16; t += 3)
                        {
                            int a = McTables.TriTable[cubeIndex, t];
                            if (a == -1) break;
                            int b = McTables.TriTable[cubeIndex, t + 1];
                            int c = McTables.TriTable[cubeIndex, t + 2];

                            mesh.Indices.Add(evi[a]);
                            mesh.Indices.Add(evi[b]);
                            mesh.Indices.Add(evi[c]);
                        }
                    }

            return mesh;
        }

        // --- Edge cache helpers (tile-local) ---

        private static int GetOrCreateX(TileMesh mesh, int[] xEdge, float iso, float a, float b, Vector3 pa, Vector3 pb, int id)
        {
            int idx = xEdge[id];
            if (idx >= 0) return idx;
            idx = mesh.Positions.Count;
            mesh.Positions.Add(Interp(iso, a, b, pa, pb));
            xEdge[id] = idx;
            return idx;
        }

        private static int GetOrCreateY(TileMesh mesh, int[] yEdge, float iso, float a, float b, Vector3 pa, Vector3 pb, int id)
        {
            int idx = yEdge[id];
            if (idx >= 0) return idx;
            idx = mesh.Positions.Count;
            mesh.Positions.Add(Interp(iso, a, b, pa, pb));
            yEdge[id] = idx;
            return idx;
        }

        private static int GetOrCreateZ(TileMesh mesh, int[] zEdge, float iso, float a, float b, Vector3 pa, Vector3 pb, int id)
        {
            int idx = zEdge[id];
            if (idx >= 0) return idx;
            idx = mesh.Positions.Count;
            mesh.Positions.Add(Interp(iso, a, b, pa, pb));
            zEdge[id] = idx;
            return idx;
        }

        // --- Linear edge ID encoders in the tile-local edge arrays ---
        // For X edge: defined at (ex, ey, ez) where ex in [0..tcx-1], ey in [0..tcy], ez in [0..tcz]
        private static int XEdgeId(int ex, int ey, int ez, int tcx, int tcy)
        {
            // size: tcx * (tcy+1) * (tcz+1)
            // id = ex + tcx * (ey + (tcy+1)*ez)
            return ex + tcx * (ey + (tcy + 1) * ez);
        }

        // For Y edge: defined at (ex, ey, ez) where ex in [0..tcx], ey in [0..tcy-1], ez in [0..tcz]
        private static int YEdgeId(int ex, int ey, int ez, int tcx, int tcy)
        {
            // size: (tcx+1) * tcy * (tcz+1)
            // id = ex + (tcx+1) * (ey + tcy*ez)
            return ex + (tcx + 1) * (ey + tcy * ez);
        }

        // For Z edge: defined at (ex, ey, ez) where ex in [0..tcx], ey in [0..tcy], ez in [0..tcz-1]
        private static int ZEdgeId(int ex, int ey, int ez, int tcx, int tcy)
        {
            // size: (tcx+1) * (tcy+1) * tcz
            // id = ex + (tcx+1) * (ey + (tcy+1)*ez)
            return ex + (tcx + 1) * (ey + (tcy + 1) * ez);
        }

        // --- voxel access (1D) ---
        private static int Vid(int nx, int ny, int x, int y, int z) => x + nx * (y + ny * z);

        private static float V(float[] v, int nx, int ny, int nz, int x, int y, int z)
        {
            // ここは範囲内アクセス前提（タイル走査が正しければOK）
            return v[Vid(nx, ny, x, y, z)];
        }

        private static Vector3 Interp(float iso, float v0, float v1, Vector3 p0, Vector3 p1)
        {
            float d = (v1 - v0);
            float t = Math.Abs(d) < 1e-12f ? 0.5f : (iso - v0) / d;
            if (t < 0f) t = 0f;
            if (t > 1f) t = 1f;
            return p0 + t * (p1 - p0);
        }

        private static int EstimateTotalVertices(TileMesh[] tiles)
        {
            long sum = 0;
            foreach (var t in tiles) if (t != null) sum += t.Positions.Count;
            return sum > int.MaxValue ? int.MaxValue : (int)sum;
        }

        private static int EstimateTotalIndices(TileMesh[] tiles)
        {
            long sum = 0;
            foreach (var t in tiles) if (t != null) sum += t.Indices.Count;
            return sum > int.MaxValue ? int.MaxValue : (int)sum;
        }
    }

}
