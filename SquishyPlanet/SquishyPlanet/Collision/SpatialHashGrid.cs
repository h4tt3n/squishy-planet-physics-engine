using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;

namespace SquishyPlanet.Collision
{
    internal class SpatialHashGrid
    {
        public readonly int NumCols;
        public readonly int NumRows;
        private readonly float _invCellSize;

        // An array of lists. This is the C# equivalent of your 'buckets' Map.
        // We pre-allocate all lists to avoid GC pressure.
        private readonly List<int>[] _buckets;

        public SpatialHashGrid(float width, float height, float cellSize)
        {
            _invCellSize = 1.0f / cellSize;
            NumCols = (int)Math.Floor(width * _invCellSize) + 1;
            NumRows = (int)Math.Floor(height * _invCellSize) + 1;

            int numBuckets = NumCols * NumRows;
            _buckets = new List<int>[numBuckets];

            // Pre-allocate all the lists
            for (int i = 0; i < numBuckets; i++)
            {
                // Pre-size lists with a small capacity (e.g., 16)
                // to reduce re-allocations for typical cells.
                _buckets[i] = new List<int>(16);
            }
        }

        /// <summary>
        /// Clears all buckets in parallel.
        /// </summary>
        public void Clear()
        {
            // List<T>.Clear() is fast and doesn't de-allocate memory.
            // This is safe to run in parallel.
            Parallel.ForEach(_buckets, bucket => bucket.Clear());
        }

        /// <summary>
        /// Gets the integer hash for a 2D position.
        /// </summary>
        public int GetHash(Vector2 pos)
        {
            int col = (int)Math.Floor(pos.X * _invCellSize);
            int row = (int)Math.Floor(pos.Y * _invCellSize);
            return col + row* NumCols;
        }

        /// <summary>
        /// Gets the bucket list for a given hash.
        /// NOT thread-safe for writing.
        /// </summary>
        public List<int> GetBucket(int hash)
        {
            // Safety check for out-of-bounds hashes
            if (hash < 0 || hash >= _buckets.Length)
            {
                return null; // Or throw
            }
            return _buckets[hash];
        }

        /// <summary>
        /// Gets all buckets for iteration.
        /// </summary>
        public IEnumerable<List<int>> GetBuckets()
        {
            return _buckets;
        }

        /// <summary>
        /// Calculates the start/end cell indices for an AABB.
        /// This is the C# equivalent of the logic in 'addSquare'.
        /// </summary>
        public void GetCellRange(Vector2 pos, float radius,
                                 out int minCol, out int maxCol,
                                 out int minRow, out int maxRow)
        {
            float minX = pos.X - radius;
            float maxX = pos.X + radius;
            float minY = pos.Y - radius;
            float maxY = pos.Y + radius;

            minCol = (int)Math.Floor(minX * _invCellSize);
            maxCol = (int)Math.Floor(maxX * _invCellSize);
            minRow = (int)Math.Floor(minY * _invCellSize);
            maxRow = (int)Math.Floor(maxY * _invCellSize);
        }
    }
}