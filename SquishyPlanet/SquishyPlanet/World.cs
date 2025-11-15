using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

using SquishyPlanet.Collision;
using SquishyPlanet.Constraints;
using SquishyPlanet.Objects;
using SquishyPlanet.Utility;
using SquishyPlanet.Forces;

namespace SquishyPlanet
{
    public class World
    {
        private readonly Particles _particles;
        private readonly DistanceConstraints _distanceConstraints;
        private readonly AngularConstraints _angularConstraints;

        private readonly ParticleParticleCollisions _particleParticleCollisions;

        private readonly NewtonianGravity _newtonianGravity;

        private readonly SpatialHashGrid _grid;
        private readonly HashSet<(int, int)> _collisionPairs;

        private readonly List<int> _allParticles;

        public readonly Factory Factory;
        public Vector2 Gravity { get; set; } = new(0, 98200.0f);
        public int NumIterations = 10;
        public int GridCellSize = 12;
        public int WorldWidth = 1280;
        public int WorldHeight = 720;

        public int NumParticles => _particles.NumObjects;
        public int NumDistanceConstraints => _distanceConstraints.NumObjects;

        public World(int maxParticles, int maxDistanceConstraints, int maxAngularConstraints, int maxParticleParticleCollisions)
        {
            _particles = new Particles(maxParticles);
            _distanceConstraints = new DistanceConstraints(maxDistanceConstraints, _particles);
            _angularConstraints = new AngularConstraints(maxAngularConstraints, _particles, _distanceConstraints);

            _newtonianGravity = new NewtonianGravity(_particles);

            _particleParticleCollisions = new ParticleParticleCollisions(maxParticleParticleCollisions, _particles);

            _grid = new SpatialHashGrid(WorldWidth, WorldHeight, GridCellSize);
            _collisionPairs = new HashSet<(int, int)>();

            _allParticles = new List<int>(maxParticles);

            Factory = new Factory(_particles, _distanceConstraints, _angularConstraints);
        }

        public void ComputeData(float invDt)
        {
            _distanceConstraints.ComputeData(invDt);
            _angularConstraints.ComputeData(invDt);
            _particleParticleCollisions.ComputeData(invDt);
        }

        public void Step(float dt)
        {
            UpdateSimulationLists();

            _newtonianGravity.Solve(_allParticles, _allParticles, dt);

            //ApplyForces(dt);

            BuildBroadphase();

            QueryBroadphase();

            ComputeData(1 / dt);

            _particleParticleCollisions.Prune();

            _distanceConstraints.ApplyWarmStart();
            _angularConstraints.ApplyWarmStart();
            _particleParticleCollisions.ApplyWarmStart();

            for (int i = 0; i < NumIterations; i++)
            {
                _angularConstraints.ApplyCorrectiveImpulse();
                _distanceConstraints.ApplyCorrectiveImpulse();
                _particleParticleCollisions.ApplyCorrectiveImpulse();
            }

            _particles.Step(dt);
        }

        private void ApplyForces(float dt)
        {
            Parallel.For(0, _particles.NumObjects, i =>
            {
                if (_particles.InvMass[i] > 0.0f)
                {
                    _particles.Velocity[i] += Gravity * dt;
                }
            });
        }

        private void BuildBroadphase()
        {
            _grid.Clear();

            // We collect all (id, hash) pairs concurrently.
            // This avoids locking on individual list.Add() calls.
            var entries = new ConcurrentBag<(int objectId, int cellHash)>();

            // Step 1: Add all particles to the grid in parallel
            Parallel.For(0, _particles.NumObjects, i =>
            {
                Vector2 pos = _particles.Position[i];
                float radius = _particles.Radius[i];
                int id = _particles.Id[i];

                _grid.GetCellRange(pos, radius,
                    out int minCol, out int maxCol,
                    out int minRow, out int maxRow);

                for (int col = minCol; col <= maxCol; col++)
                {
                    for (int row = minRow; row <= maxRow; row++)
                    {
                        int hash = col + row * _grid.NumCols;
                        entries.Add((id, hash));
                    }
                }
            });

            // Step 2: (TODO) Add constraints
            // You would add a similar Parallel.For for _distanceConstraints here
            // using your 'addLineSegmentWithRadius' logic to generate hashes.

            // Step 3: Merge all entries (sequentially)
            // This part is fast and just populates the pre-allocated lists.
            foreach (var (id, hash) in entries)
            {
                // GetBucket can return null if hash is out of bounds
                _grid.GetBucket(hash)?.Add(id);
            }
        }

        private void QueryBroadphase()
        {
            _collisionPairs.Clear();

            // Note: This part is NOT parallelized, but the 'create' step below is.
            // You could parallelize this loop using a ConcurrentDictionary
            // but a simple HashSet is often faster if the 'create' logic is heavy.
            foreach (var bucket in _grid.GetBuckets())
            {
                if (bucket.Count < 2) continue;

                // Standard O(n^2) check *within* the bucket
                for (int i = 0; i < bucket.Count; i++)
                {
                    for (int j = i + 1; j < bucket.Count; j++)
                    {
                        int idA = bucket[i];
                        int idB = bucket[j];

                        // Sort IDs to create a unique pair (A,B) == (B,A)
                        var pair = (idA < idB) ? (idA, idB) : (idB, idA);

                        // Add to HashSet. If Add() returns true, this is a new pair.
                        if (_collisionPairs.Add(pair))
                        {
                            // --- This is where your JS type logic goes ---

                            // We don't have ToolBox, so we just create particle-particle pairs.
                            // You would add your ObjectType logic here.

                            // TODO: Call your collision-creation methods
                            _particleParticleCollisions.Create(pair.Item1, pair.Item2);
                        }
                    }
                }
            }
        }

        private void UpdateSimulationLists()
        {
            _allParticles.Clear();

            // Loop through all active particles
            for (int i = 0; i < _particles.NumObjects; i++)
            {
                // Add the stable ID from the dense array
                _allParticles.Add(_particles.Id[i]);
            }
        }

        public ReadOnlySpan<Vector2> GetParticlePositions()
        {
            return _particles.Position.AsSpan(0, _particles.NumObjects);
        }

        public ReadOnlySpan<ColorRgb> GetParticleColors()
        {
            return _particles.Color.AsSpan(0, _particles.NumObjects);
        }

        public ReadOnlySpan<float> GetParticleRadii()
        {
            return _particles.Radius.AsSpan(0, _particles.NumObjects);
        }

        public ReadOnlySpan<int> GetConstraintParticleA_IDs()
        {
            return _distanceConstraints.ParticleA.AsSpan(0, _distanceConstraints.NumObjects);
        }

        public ReadOnlySpan<int> GetConstraintParticleB_IDs()
        {
            return _distanceConstraints.ParticleB.AsSpan(0, _distanceConstraints.NumObjects);
        }

        public ReadOnlySpan<float> GetConstraintRadii()
        {
            return _distanceConstraints.Radius.AsSpan(0, _distanceConstraints.NumObjects);
        }

        /// <summary>
        /// Gets a particle's current position using its stable ID.
        /// This is the crucial lookup needed for rendering constraints.
        /// </summary>
        public Vector2 GetParticlePositionById(int id)
        {
            // Safety check
            if (id < 0 || id >= _particles.MaxObjects)
            {
                return Vector2.Zero;
            }

            // Use the internal map to find the dense index
            int denseIndex = _particles.index[id];

            // Safety check
            if (denseIndex == -1 || denseIndex >= _particles.NumObjects)
            {
                return Vector2.Zero;
            }

            return _particles.Position[denseIndex];
        }
    }
}
