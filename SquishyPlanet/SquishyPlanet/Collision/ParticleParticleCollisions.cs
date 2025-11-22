using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;

using SquishyPlanet.Objects;

namespace SquishyPlanet.Collision
{
    /// <summary>
    /// Internal data store for persistent particle-particle collision contacts.
    /// Manages data in a Struct of Arrays (SoA) layout.
    /// Uses a Dictionary to map particle pairs to a contact index.
    /// </summary>
    internal class ParticleParticleCollisions
    {
        private const int IdDefaultValue = -1;
        private const float PropertyDefaultValue = 0.0f;
        private const float Buffer = 0.5f;

        // Dependency
        private readonly Particles _particles;

        public int NumObjects { get; private set; }
        public readonly int MaxObjects;

        // ID Management
        // Maps a unique 64-bit pair key to a dense 32-bit array index
        private readonly Dictionary<long, int> _index;
        public readonly long[] Id; // Stores the 64-bit key

        // Constraint Properties (SoA)
        public readonly int[] ParticleA; // Particle ID
        public readonly int[] ParticleB; // Particle ID
        public readonly float[] CStiffness;
        public readonly float[] CDamping;
        public readonly float[] CWarmstart;
        public readonly float[] CCorrection;
        public readonly float[] Distance;
        public readonly float[] RestImpulse;
        public readonly Vector2[] Unit;
        public readonly Vector2[] AccumulatedImpulse;
        public readonly float[] ReducedMass;

        public ParticleParticleCollisions(int maxObjects, Particles particles)
        {
            MaxObjects = maxObjects;
            NumObjects = 0;
            _particles = particles;

            _index = new Dictionary<long, int>(maxObjects);
            Id = new long[maxObjects];

            ParticleA = new int[maxObjects];
            ParticleB = new int[maxObjects];
            CStiffness = new float[maxObjects];
            CDamping = new float[maxObjects];
            CWarmstart = new float[maxObjects];
            CCorrection = new float[maxObjects];
            Distance = new float[maxObjects];
            RestImpulse = new float[maxObjects];
            Unit = new Vector2[maxObjects];
            AccumulatedImpulse = new Vector2[maxObjects];
            ReducedMass = new float[maxObjects];

            Clear();
        }

        public void Clear()
        {
            NumObjects = 0;
            _index.Clear();
            Id.AsSpan().Fill(IdDefaultValue);

            ParticleA.AsSpan().Clear();
            ParticleB.AsSpan().Clear();
            CStiffness.AsSpan().Clear();
            CDamping.AsSpan().Clear();
            CWarmstart.AsSpan().Clear();
            CCorrection.AsSpan().Clear();
            Distance.AsSpan().Clear();
            RestImpulse.AsSpan().Clear();
            Unit.AsSpan().Clear();
            AccumulatedImpulse.AsSpan().Clear();
            ReducedMass.AsSpan().Clear();
        }

        /// <summary>
        /// Generates a unique 64-bit key for a pair of 32-bit IDs.
        /// Assumes particleA < particleB.
        /// </summary>
        private static long GetKey(int particleA, int particleB)
        {
            return ((long)particleA << 32) | (uint)particleB;
        }

        public int? Create(int particleA, int particleB)
        {
            // Ensure A < B for consistent key generation
            if (particleA > particleB)
            {
                (particleA, particleB) = (particleB, particleA);
            }

            long createId = GetKey(particleA, particleB);

            // Check if constraint already exists
            if (_index.ContainsKey(createId)) { return null; }

            // Check if array is full
            if (NumObjects == MaxObjects) { return null; }

            // --- Narrowphase Check ---
            int pA = _particles.index[particleA];
            int pB = _particles.index[particleB];

            float sumRadii = _particles.Radius[pA] + _particles.Radius[pB];
            float creationDistSqr = (sumRadii + Buffer) * (sumRadii + Buffer);

            float distanceSquared = Vector2.DistanceSquared(
                _particles.Position[pA],
                _particles.Position[pB]);

            // Check if distance is too big
            if (distanceSquared > creationDistSqr) { return null; }
            // --- End Narrowphase Check ---

            float sumInvMass = _particles.InvMass[pA] + _particles.InvMass[pB];
            float reducedMass = (sumInvMass > 0) ? 1.0f / sumInvMass : 0.0f;

            int createIndex = NumObjects;

            Id[createIndex] = createId;
            ParticleA[createIndex] = particleA;
            ParticleB[createIndex] = particleB;

            CStiffness[createIndex] = 0.5f;
            CDamping[createIndex] = 1.0f;
            CWarmstart[createIndex] = 0.5f;
            CCorrection[createIndex] = 0.2f;

            ReducedMass[createIndex] = reducedMass;

            // Transient data will be set by ComputeData
            Distance[createIndex] = 0.0f;
            RestImpulse[createIndex] = 0.0f;
            Unit[createIndex] = Vector2.Zero;
            AccumulatedImpulse[createIndex] = Vector2.Zero;

            _index.Add(createId, createIndex);
            NumObjects++;

            return createIndex;
        }

        public bool Delete(long id)
        {
            if (!_index.TryGetValue(id, out int deleteIndex))
            {
                return false; // ID doesn't exist
            }

            int lastIndex = NumObjects - 1;
            long lastId = Id[lastIndex];

            // Perform swap-delete
            _CopyData(lastIndex, deleteIndex);

            // Update lookup map
            _index[lastId] = deleteIndex;
            _index.Remove(id);

            NumObjects--;
            return true;
        }

        /// <summary>
        /// Efficiently prunes contacts that were flagged by ComputeData.
        /// </summary>
        public void Prune()
        {
            // Loop backwards to safely perform swap-delete
            for (int i = NumObjects - 1; i >= 0; i--)
            {
                if (ReducedMass[i] == -1.0f)
                {
                    long key = Id[i];
                    Delete(key);
                }
            }
        }

        private void _CopyData(int sourceIndex, int destIndex)
        {
            Id[destIndex] = Id[sourceIndex];
            ParticleA[destIndex] = ParticleA[sourceIndex];
            ParticleB[destIndex] = ParticleB[sourceIndex];
            CStiffness[destIndex] = CStiffness[sourceIndex];
            CDamping[destIndex] = CDamping[sourceIndex];
            CWarmstart[destIndex] = CWarmstart[sourceIndex];
            CCorrection[destIndex] = CCorrection[sourceIndex];
            Distance[destIndex] = Distance[sourceIndex];
            RestImpulse[destIndex] = RestImpulse[sourceIndex];
            Unit[destIndex] = Unit[sourceIndex];
            AccumulatedImpulse[destIndex] = AccumulatedImpulse[sourceIndex];
            ReducedMass[destIndex] = ReducedMass[sourceIndex];
        }

        public void ApplyCorrectiveImpulse()
        {
            // Sequential Gauss-Seidel solver. CANNOT be parallelized.
            for (int i = 0; i < NumObjects; i++)
            {
                SolveImpulse(i);
            }
            for (int i = NumObjects - 1; i >= 0; i--)
            {
                SolveImpulse(i);
            }
        }

        private void SolveImpulse(int i)
        {
            // Only solve for penetrating contacts
            if (Distance[i] > 0) { return; }

            int pA = _particles.index[ParticleA[i]];
            int pB = _particles.index[ParticleB[i]];

            Vector2 deltaImpulse = _particles.Impulse[pB] - _particles.Impulse[pA];
            float projectedImpulse = Vector2.Dot(Unit[i], deltaImpulse);
            float impulseError = (projectedImpulse - RestImpulse[i]) * ReducedMass[i] * CCorrection[i];
            Vector2 correctiveImpulse = Unit[i] * -impulseError;

            _particles.Impulse[pA] -= correctiveImpulse * _particles.InvMass[pA];
            _particles.Impulse[pB] += correctiveImpulse * _particles.InvMass[pB];

            AccumulatedImpulse[i] += correctiveImpulse;
        }

        public void ApplyWarmStart()
        {
            for (int i = 0; i < NumObjects; i++)
            {
                float projectedImpulse = Vector2.Dot(Unit[i], AccumulatedImpulse[i]);
                AccumulatedImpulse[i] = Vector2.Zero;

                if (projectedImpulse < 0.0) { continue; }

                int pA = _particles.index[ParticleA[i]];
                int pB = _particles.index[ParticleB[i]];

                Vector2 warmstartImpulse = Unit[i] * projectedImpulse * CWarmstart[i];

                _particles.Impulse[pA] -= warmstartImpulse * _particles.InvMass[pA];
                _particles.Impulse[pB] += warmstartImpulse * _particles.InvMass[pB];
            }
        }

        public void ComputeData(float invDt)
        {
            Parallel.For(0, NumObjects, i =>
            {
                int pA = _particles.index[ParticleA[i]];
                int pB = _particles.index[ParticleB[i]];

                Vector2 deltaPos = _particles.Position[pB] - _particles.Position[pA];
                float distanceSquared = deltaPos.LengthSquared();
                float sumRadii = _particles.Radius[pA] + _particles.Radius[pB];
                float sumRadiiBuffer = sumRadii + Buffer;

                if (distanceSquared > sumRadiiBuffer * sumRadiiBuffer)
                {
                    // Flag for pruning
                    ReducedMass[i] = -1.0f;
                    return; // continue
                }

                if (distanceSquared > sumRadii * sumRadii)
                {
                    // Colliding, but not penetrating
                    RestImpulse[i] = 0.0f;
                    Distance[i] = 1.0f; // Set > 0 to skip solver
                    return; // continue
                }

                float distance = MathF.Sqrt(distanceSquared);

                Distance[i] = distance - sumRadii; // Penetration depth (negative)
                Unit[i] = (distance > 0) ? deltaPos / distance : new Vector2(1, 0); // Default axis if stacked

                Vector2 deltaVel = _particles.Velocity[pB] - _particles.Velocity[pA];
                float velocityError = Vector2.Dot(Unit[i], deltaVel);

                RestImpulse[i] = -(Distance[i] * CStiffness[i] * invDt + velocityError * CDamping[i]);
            });
        }
    }
}