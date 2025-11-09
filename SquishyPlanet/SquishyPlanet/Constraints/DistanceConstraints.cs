using System;
using System.Numerics;
using System.Threading.Tasks;

namespace SquishyPlanet.Objects
{
    /// <summary>
    /// Internal data store for all distance constraints.
    /// Manages data in a Struct of Arrays (SoA) layout for high performance.
    /// </summary>
    internal class DistanceConstraints
    {
        private const int InvalidId = -1;

        // Dependency
        private readonly Particles _particles;

        public int NumObjects { get; private set; }
        public readonly int MaxObjects;

        // ID Management
        private int numFreeIds;
        private readonly int[] nextFreeId;
        private readonly int[] index; // Sparse lookup: index[id] -> denseIndex
        public readonly int[] Id;     // Packed array: Id[denseIndex] -> id

        // Constraint Properties (SoA)
        public readonly int[] ParticleA; // Particle ID
        public readonly int[] ParticleB; // Particle ID
        public readonly float[] CStiffness;
        public readonly float[] CDamping;
        public readonly float[] CWarmstart;
        public readonly float[] CCorrection;
        public readonly float[] Radius;
        public readonly float[] RestLength;
        public readonly float[] RestImpulse;
        public readonly Vector2[] Unit;
        public readonly Vector2[] AccumulatedImpulse;
        public readonly float[] ReducedMass;
        public readonly float[] InverseInertia;
        public readonly float[] AngularVelocity;

        public DistanceConstraints(int maxObjects, Particles particles)
        {
            MaxObjects = maxObjects;
            NumObjects = 0;
            numFreeIds = maxObjects;
            _particles = particles;

            // Allocate all memory
            nextFreeId = new int[maxObjects];
            index = new int[maxObjects];
            Id = new int[maxObjects];

            ParticleA = new int[maxObjects];
            ParticleB = new int[maxObjects];
            CStiffness = new float[maxObjects];
            CDamping = new float[maxObjects];
            CWarmstart = new float[maxObjects];
            CCorrection = new float[maxObjects];
            Radius = new float[maxObjects];
            RestLength = new float[maxObjects];
            RestImpulse = new float[maxObjects];
            Unit = new Vector2[maxObjects];
            AccumulatedImpulse = new Vector2[maxObjects];
            ReducedMass = new float[maxObjects];
            InverseInertia = new float[maxObjects];
            AngularVelocity = new float[maxObjects];

            Clear();
        }

        public void Clear()
        {
            NumObjects = 0;
            numFreeIds = MaxObjects;

            for (int i = 0; i < MaxObjects; i++)
            {
                nextFreeId[i] = (MaxObjects - 1) - i;
            }

            index.AsSpan().Fill(InvalidId);
            Id.AsSpan().Fill(InvalidId);

            ParticleA.AsSpan().Clear();
            ParticleB.AsSpan().Clear();
            CStiffness.AsSpan().Clear();
            CDamping.AsSpan().Clear();
            CWarmstart.AsSpan().Clear();
            CCorrection.AsSpan().Clear();
            Radius.AsSpan().Clear();
            RestLength.AsSpan().Clear();
            RestImpulse.AsSpan().Clear();
            Unit.AsSpan().Clear();
            AccumulatedImpulse.AsSpan().Clear();
            ReducedMass.AsSpan().Clear();
            InverseInertia.AsSpan().Clear();
            AngularVelocity.AsSpan().Clear();
        }

        public int Create(int particleAId, int particleBId, float radius)
        {
            if (NumObjects == MaxObjects)
            {
                return InvalidId; // Array is full
            }

            // Get index and id
            int createId = nextFreeId[numFreeIds - 1];
            int createIndex = NumObjects;

            numFreeIds--;

            // Get dense indices of particles
            // NOTE: We assume the factory has validated these IDs.
            int pA = _particles.index[particleAId];
            int pB = _particles.index[particleBId];

            // Calculate initial data
            Vector2 deltaPos = _particles.Position[pB] - _particles.Position[pA];
            float distance = deltaPos.Length();
            float sumInvMass = _particles.InvMass[pA] + _particles.InvMass[pB];
            float reducedMass = (sumInvMass > 0) ? 1.0f / sumInvMass : 0.0f;

            // Set ID and properties
            Id[createIndex] = createId;
            ParticleA[createIndex] = particleAId;
            ParticleB[createIndex] = particleBId;

            CStiffness[createIndex] = 1.0f;
            CDamping[createIndex] = 1.0f;
            CWarmstart[createIndex] = 1.0f;
            CCorrection[createIndex] = 1.0f;

            Radius[createIndex] = radius;
            RestLength[createIndex] = 30; // distance;
            ReducedMass[createIndex] = reducedMass;

            // Reset transient data
            RestImpulse[createIndex] = 0.0f;
            Unit[createIndex] = Vector2.Zero;
            AccumulatedImpulse[createIndex] = Vector2.Zero;
            InverseInertia[createIndex] = 0.0f;
            AngularVelocity[createIndex] = 0.0f;

            // Update lookup map
            index[createId] = createIndex;

            NumObjects++;
            return createId;
        }

        public bool Delete(int id)
        {
            if (id < 0 || id >= MaxObjects)
            {
                return false;
            }

            int deleteIndex = index[id];
            if (deleteIndex == InvalidId)
            {
                return false;
            }

            int lastIndex = NumObjects - 1;
            int lastId = Id[lastIndex];

            // Perform swap-delete
            _CopyData(lastIndex, deleteIndex);

            // Update lookup maps
            index[lastId] = deleteIndex;
            index[id] = InvalidId;

            NumObjects--;

            // Add freed ID back to stack
            nextFreeId[numFreeIds] = id;
            numFreeIds++;

            return true;
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
            Radius[destIndex] = Radius[sourceIndex];
            RestLength[destIndex] = RestLength[sourceIndex];
            RestImpulse[destIndex] = RestImpulse[sourceIndex];
            Unit[destIndex] = Unit[sourceIndex];
            AccumulatedImpulse[destIndex] = AccumulatedImpulse[sourceIndex];
            ReducedMass[destIndex] = ReducedMass[sourceIndex];
            InverseInertia[destIndex] = InverseInertia[sourceIndex];
            AngularVelocity[destIndex] = AngularVelocity[sourceIndex];
        }

        public void ApplyCorrectiveImpulse()
        {
            // Note: This is a Sequential Gauss-Seidel solver.
            // These loops CANNOT be parallelized as they have dependencies.
            for (int i = 0; i < NumObjects; i++)
            {
                _SolveImpulse(i);
            }

            for (int i = NumObjects - 1; i >= 0; i--)
            {
                _SolveImpulse(i);
            }
        }

        private void _SolveImpulse(int i)
        {
            int pA = _particles.index[ParticleA[i]];
            int pB = _particles.index[ParticleB[i]];

            float restImpulse = RestImpulse[i];
            float reducedMass = ReducedMass[i];
            Vector2 unit = Unit[i];

            Vector2 deltaImpulse = _particles.Impulse[pB] - _particles.Impulse[pA];

            float projectedImpulse = Vector2.Dot(unit, deltaImpulse);
            float impulseError = (projectedImpulse - restImpulse) * reducedMass * CCorrection[i];
            Vector2 correctiveImpulse = unit * -impulseError;

            _particles.Impulse[pA] -= correctiveImpulse * _particles.InvMass[pA];
            _particles.Impulse[pB] += correctiveImpulse * _particles.InvMass[pB];

            AccumulatedImpulse[i] += correctiveImpulse;
        }

        public void ApplyWarmStart()
        {
            Parallel.For(0, NumObjects, i =>
            {
                Vector2 unit = Unit[i];
                Vector2 accumulatedImpulse = AccumulatedImpulse[i];

                float projectedImpulse = Vector2.Dot(unit, accumulatedImpulse);

                AccumulatedImpulse[i] = Vector2.Zero; // Reset for next frame

                if (projectedImpulse < 0.0f) { return; } // continue;

                int pA = _particles.index[ParticleA[i]];
                int pB = _particles.index[ParticleB[i]];

                Vector2 warmstartImpulse = unit * projectedImpulse * CWarmstart[i];

                _particles.Impulse[pA] -= warmstartImpulse * _particles.InvMass[pA];
                _particles.Impulse[pB] += warmstartImpulse * _particles.InvMass[pB];
            });
        }

        public void ComputeData(float invDt)
        {
            Parallel.For(0, NumObjects, i =>
            {
                int pA = _particles.index[ParticleA[i]];
                int pB = _particles.index[ParticleB[i]];

                Vector2 deltaPos = _particles.Position[pB] - _particles.Position[pA];
                float distance = deltaPos.Length();
                float distanceSquared = deltaPos.LengthSquared();

                if (distance > 0.0f)
                {
                    Unit[i] = deltaPos / distance;
                }
                else
                {
                    Unit[i] = Vector2.Zero;
                }

                Vector2 deltaVel = _particles.Velocity[pB] - _particles.Velocity[pA];
                Vector2 unit = Unit[i];

                float distanceError = Vector2.Dot(unit, deltaPos) - RestLength[i];
                float velocityError = Vector2.Dot(unit, deltaVel);

                RestImpulse[i] = -(distanceError * CStiffness[i] * invDt + velocityError * CDamping[i]);

                float inertia = distanceSquared * ReducedMass[i];
                InverseInertia[i] = (inertia > 0.0f) ? 1.0f / inertia : 0.0f;
                AngularVelocity[i] = (deltaPos.X * deltaVel.Y - deltaPos.Y * deltaVel.X) * ReducedMass[i] * InverseInertia[i];
            });
        }
    }
}