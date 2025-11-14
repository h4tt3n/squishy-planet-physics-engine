using System;
using System.Numerics;
using System.Threading.Tasks;
using SquishyPlanet.Objects;

namespace SquishyPlanet.Constraints
{
    /// <summary>
    /// Internal data store for all angular spring constraints.
    /// Manages data in a Struct of Arrays (SoA) layout.
    /// </summary>
    internal class AngularConstraints
    {
        private const int InvalidId = -1;

        // Dependencies (injected)
        private readonly Particles _particles;
        private readonly DistanceConstraints _distanceConstraints;

        public int NumObjects { get; private set; }
        public readonly int MaxObjects;

        // ID Management
        private int numFreeIds;
        private readonly int[] nextFreeId;
        public readonly int[] index; // Sparse lookup: index[id] -> denseIndex
        public readonly int[] Id;     // Packed array: Id[denseIndex] -> id

        // Constraint Properties (SoA)
        public readonly int[] DistanceConstraintA; // ID of the first distance constraint
        public readonly int[] DistanceConstraintB; // ID of the second distance constraint
        public readonly float[] CStiffness;
        public readonly float[] CDamping;
        public readonly float[] CWarmstart;
        public readonly float[] CCorrection;
        public readonly Vector2[] Angle; // X = cos(angle), Y = sin(angle)
        public readonly Vector2[] RestAngle; // X = cos(angle), Y = sin(angle)
        public readonly float[] ReducedInertia;
        public readonly float[] RestImpulse;
        public readonly float[] AccumulatedImpulse;

        public AngularConstraints(int maxObjects, Particles particles, DistanceConstraints distanceConstraints)
        {
            MaxObjects = maxObjects;
            NumObjects = 0;
            numFreeIds = maxObjects;

            _particles = particles;
            _distanceConstraints = distanceConstraints;

            // Allocate all memory
            nextFreeId = new int[maxObjects];
            index = new int[maxObjects];
            Id = new int[maxObjects];

            DistanceConstraintA = new int[maxObjects];
            DistanceConstraintB = new int[maxObjects];
            CStiffness = new float[maxObjects];
            CDamping = new float[maxObjects];
            CWarmstart = new float[maxObjects];
            CCorrection = new float[maxObjects];
            Angle = new Vector2[maxObjects];
            RestAngle = new Vector2[maxObjects];
            ReducedInertia = new float[maxObjects];
            RestImpulse = new float[maxObjects];
            AccumulatedImpulse = new float[maxObjects];

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

            DistanceConstraintA.AsSpan().Clear();
            DistanceConstraintB.AsSpan().Clear();
            CStiffness.AsSpan().Clear();
            CDamping.AsSpan().Clear();
            CWarmstart.AsSpan().Clear();
            CCorrection.AsSpan().Clear();
            Angle.AsSpan().Clear();
            RestAngle.AsSpan().Clear();
            ReducedInertia.AsSpan().Clear();
            RestImpulse.AsSpan().Clear();
            AccumulatedImpulse.AsSpan().Clear();
        }

        public int Create(int distanceConstraintAId, int distanceConstraintBId)
        {
            if (NumObjects == MaxObjects)
            {
                return InvalidId; // Array is full
            }

            int createId = nextFreeId[numFreeIds - 1];
            int createIndex = NumObjects;

            numFreeIds--;

            Id[createIndex] = createId;
            DistanceConstraintA[createIndex] = distanceConstraintAId;
            DistanceConstraintB[createIndex] = distanceConstraintBId;

            CStiffness[createIndex] = 1.0f;
            CDamping[createIndex] = 1.0f;
            CWarmstart[createIndex] = 1.0f;
            CCorrection[createIndex] = 1.0f;

            int indexA = _distanceConstraints.index[distanceConstraintAId];
            int indexB = _distanceConstraints.index[distanceConstraintBId];

            Vector2 unitA = _distanceConstraints.Unit[indexA];
            Vector2 unitB = _distanceConstraints.Unit[indexB];

            // Calculate cosine (Dot product) and sine (2D Cross product)
            float cosAngle = Vector2.Dot(unitA, unitB);
            float sinAngle = (unitA.X * unitB.Y) - (unitA.Y * unitB.X);

            Angle[createIndex] = new Vector2(cosAngle, sinAngle);
            RestAngle[createIndex] = Angle[createIndex];

            ReducedInertia[createIndex] = 0.0f;
            RestImpulse[createIndex] = 0.0f;
            AccumulatedImpulse[createIndex] = 0.0f;

            index[createId] = createIndex;
            NumObjects++;

            return createId;
        }

        public bool Delete(int id)
        {
            if (id < 0 || id >= MaxObjects) return false;
            int deleteIndex = index[id];
            if (deleteIndex == InvalidId) return false;

            int lastIndex = NumObjects - 1;
            int lastId = Id[lastIndex];

            // Perform swap-delete
            _CopyData(lastIndex, deleteIndex);

            index[lastId] = deleteIndex;
            index[id] = InvalidId;

            NumObjects--;

            nextFreeId[numFreeIds] = id;
            numFreeIds++;

            return true;
        }

        private void _CopyData(int sourceIndex, int destIndex)
        {
            Id[destIndex] = Id[sourceIndex];
            DistanceConstraintA[destIndex] = DistanceConstraintA[sourceIndex];
            DistanceConstraintB[destIndex] = DistanceConstraintB[sourceIndex];
            CStiffness[destIndex] = CStiffness[sourceIndex];
            CDamping[destIndex] = CDamping[sourceIndex];
            CWarmstart[destIndex] = CWarmstart[sourceIndex];
            CCorrection[destIndex] = CCorrection[sourceIndex];
            Angle[destIndex] = Angle[sourceIndex];
            RestAngle[destIndex] = RestAngle[sourceIndex];
            ReducedInertia[destIndex] = ReducedInertia[sourceIndex];
            RestImpulse[destIndex] = RestImpulse[sourceIndex];
            AccumulatedImpulse[destIndex] = AccumulatedImpulse[sourceIndex];
        }

        public void ComputeData(float invDt)
        {
            Parallel.For(0, NumObjects, i =>
            {
                int indexA = _distanceConstraints.index[DistanceConstraintA[i]];
                int indexB = _distanceConstraints.index[DistanceConstraintB[i]];

                Vector2 unitA = _distanceConstraints.Unit[indexA];
                Vector2 unitB = _distanceConstraints.Unit[indexB];

                float cosAngle = Vector2.Dot(unitA, unitB);
                float sinAngle = (unitA.X * unitB.Y) - (unitA.Y * unitB.X);
                Angle[i] = new Vector2(cosAngle, sinAngle);

                Vector2 restAngle = RestAngle[i];
                Vector2 angle = Angle[i];

                // sin(rest - current)
                float angleError = (restAngle.X * angle.Y) - (restAngle.Y * angle.X);

                float angularVelocityError =
                    _distanceConstraints.AngularVelocity[indexB] - _distanceConstraints.AngularVelocity[indexA];

                float inverseInertiaSum =
                    _distanceConstraints.InverseInertia[indexA] + _distanceConstraints.InverseInertia[indexB];

                ReducedInertia[i] = (inverseInertiaSum > 0.0f) ? 1.0f / inverseInertiaSum : 0.0f;

                RestImpulse[i] = -(CStiffness[i] * angleError * invDt + CDamping[i] * angularVelocityError);
            });
        }

        public void ApplyCorrectiveImpulse()
        {
            // Sequential Gauss-Seidel solver. CANNOT be parallelized.
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
            int indexA = _distanceConstraints.index[DistanceConstraintA[i]];
            int indexB = _distanceConstraints.index[DistanceConstraintB[i]];

            int idAA = _distanceConstraints.ParticleA[indexA];
            int idAB = _distanceConstraints.ParticleB[indexA];
            int idBA = _distanceConstraints.ParticleA[indexB];
            int idBB = _distanceConstraints.ParticleB[indexB];

            int indexAA = _particles.index[idAA];
            int indexAB = _particles.index[idAB];
            int indexBA = _particles.index[idBA];
            int indexBB = _particles.index[idBB];

            Vector2 distanceA = _particles.Position[indexAB] - _particles.Position[indexAA];
            Vector2 distanceB = _particles.Position[indexBB] - _particles.Position[indexBA];

            Vector2 impulseA = _particles.Impulse[indexAB] - _particles.Impulse[indexAA];
            Vector2 impulseB = _particles.Impulse[indexBB] - _particles.Impulse[indexBA];

            float localImpulseA = (distanceA.X * impulseA.Y - distanceA.Y * impulseA.X) * _distanceConstraints.ReducedMass[indexA];
            float localImpulseB = (distanceB.X * impulseB.Y - distanceB.Y * impulseB.X) * _distanceConstraints.ReducedMass[indexB];

            float angularImpulseA = localImpulseA * _distanceConstraints.InverseInertia[indexA];
            float angularImpulseB = localImpulseB * _distanceConstraints.InverseInertia[indexB];

            float deltaImpulse = angularImpulseB - angularImpulseA;
            float impulseError = deltaImpulse - RestImpulse[i];
            float correctiveImpulse = -impulseError * ReducedInertia[i] * CCorrection[i];

            float newAngularImpulseA = correctiveImpulse * _distanceConstraints.InverseInertia[indexA];
            float newAngularImpulseB = correctiveImpulse * _distanceConstraints.InverseInertia[indexB];

            // Convert to linear perpendicular impulse (perp(v) = (-y, x))
            Vector2 newImpulseA = new Vector2(-distanceA.Y, distanceA.X) * (newAngularImpulseA * _distanceConstraints.ReducedMass[indexA]);
            Vector2 newImpulseB = new Vector2(-distanceB.Y, distanceB.X) * (newAngularImpulseB * _distanceConstraints.ReducedMass[indexB]);

            // Apply
            _particles.Impulse[indexAA] += newImpulseA * _particles.InvMass[indexAA];
            _particles.Impulse[indexAB] -= newImpulseA * _particles.InvMass[indexAB];
            _particles.Impulse[indexBA] -= newImpulseB * _particles.InvMass[indexBA];
            _particles.Impulse[indexBB] += newImpulseB * _particles.InvMass[indexBB];

            AccumulatedImpulse[i] += correctiveImpulse;
        }

        public void ApplyWarmStart()
        {
            Parallel.For(0, NumObjects, i =>
            {
                int indexA = _distanceConstraints.index[DistanceConstraintA[i]];
                int indexB = _distanceConstraints.index[DistanceConstraintB[i]];

                int idAA = _distanceConstraints.ParticleA[indexA];
                int idAB = _distanceConstraints.ParticleB[indexA];
                int idBA = _distanceConstraints.ParticleA[indexB];
                int idBB = _distanceConstraints.ParticleB[indexB];

                int indexAA = _particles.index[idAA];
                int indexAB = _particles.index[idAB];
                int indexBA = _particles.index[idBA];
                int indexBB = _particles.index[idBB];

                Vector2 distanceA = _particles.Position[indexAB] - _particles.Position[indexAA];
                Vector2 distanceB = _particles.Position[indexBB] - _particles.Position[indexBA];

                float warmstartImpulse = CWarmstart[i] * AccumulatedImpulse[i];
                AccumulatedImpulse[i] = 0.0f;

                float newAngularImpulseA = warmstartImpulse * _distanceConstraints.InverseInertia[indexA];
                float newAngularImpulseB = warmstartImpulse * _distanceConstraints.InverseInertia[indexB];

                Vector2 newImpulseA = new Vector2(-distanceA.Y, distanceA.X) * (newAngularImpulseA * _distanceConstraints.ReducedMass[indexA]);
                Vector2 newImpulseB = new Vector2(-distanceB.Y, distanceB.X) * (newAngularImpulseB * _distanceConstraints.ReducedMass[indexB]);

                // Apply
                _particles.Impulse[indexAA] += newImpulseA * _particles.InvMass[indexAA];
                _particles.Impulse[indexAB] -= newImpulseA * _particles.InvMass[indexAB];
                _particles.Impulse[indexBA] -= newImpulseB * _particles.InvMass[indexBA];
                _particles.Impulse[indexBB] += newImpulseB * _particles.InvMass[indexBB];
            });
        }
    }
}