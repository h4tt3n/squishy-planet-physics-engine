using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks; // For Parallel.For

using SquishyPlanet.Objects;

namespace SquishyPlanet.Forces
{
    internal class NewtonianGravity
    {
        private readonly Particles _particles;

        // This is the "second buffer" for writing impulses
        private readonly Vector2[] _impulseBuffer;

        public float GravitationalConstant { get; set; } = 1000; //6.674e-5f;
        private const float _epsilonSqr = 1.0f;

        public NewtonianGravity(Particles particles)
        {
            _particles = particles;
            // Allocate the temporary buffer once
            _impulseBuffer = new Vector2[_particles.MaxObjects];
        }

        /// <summary>
        /// Calculates and applies gravitational impulse between two groups of particles.
        /// </summary>
        public void Solve(List<int> groupA, List<int> groupB, float dt)
        {
            // O(N*M) calculation for two distinct groups
            if (groupA != groupB)
            {
                SolveDistinct(groupA, groupB, dt);
                return;
            }

            // O(N^2) calculation for a self-interacting group
            SolveNBodyParallel(groupA, dt);
        }

        /// <summary>
        /// O(N^2) parallel calculation for a single group.
        /// This is the "Map-Reduce" pattern you described.
        /// </summary>
        private void SolveNBodyParallel(List<int> group, float dt)
        {
            // Clear the buffer just for the active particles
            // (This is faster than clearing the whole array if NumObjects is small)
            Array.Clear(_impulseBuffer, 0, _particles.NumObjects);

            // 1. --- MAP (Parallel) ---
            // Each thread calculates the total force on *one* particle.
            Parallel.ForEach(group, idA =>
            {
                int pA = _particles.index[idA];
                float massA = _particles.Mass[pA];
                Vector2 posA = _particles.Position[pA];

                Vector2 totalImpulseOnA = Vector2.Zero;

                foreach (int idB in group)
                {
                    if (idA == idB) continue; // No self-gravity

                    int pB = _particles.index[idB];

                    // Calculate force A<-B
                    Vector2 deltaPos = _particles.Position[pB] - posA;
                    float distSqr = deltaPos.LengthSquared() + _epsilonSqr;
                    float dist = MathF.Sqrt(distSqr);
                    float forceMag = GravitationalConstant * (massA * _particles.Mass[pB]) / distSqr;
                    Vector2 impulse = (deltaPos / dist) * forceMag * dt;

                    totalImpulseOnA += impulse;
                }

                // Write the total result to the temporary buffer.
                // This is safe! Each thread only writes to its *own* index 'pA'.
                _impulseBuffer[pA] = totalImpulseOnA;
            });

            // 2. --- REDUCE (Parallel) ---
            // Apply all results from the temp buffer to the main impulse buffer.
            Parallel.ForEach(group, idA =>
            {
                int pA = _particles.index[idA];
                if (_particles.InvMass[pA] > 0)
                {
                    // --- THIS IS THE FIX ---
                    // Before: _particles.Impulse[pA] += _impulseBuffer[pA];
                    _particles.Impulse[pA] += _impulseBuffer[pA] * _particles.InvMass[pA];
                }
            });
        }

        /// <summary>
        /// O(N*M) sequential calculation for two distinct groups.
        /// This is harder to parallelize safely, so it remains sequential.
        /// </summary>
        private void SolveDistinct(List<int> groupA, List<int> groupB, float dt)
        {
            foreach (int idA in groupA)
            {
                int pA = _particles.index[idA];
                float massA = _particles.Mass[pA];
                float invMassA = _particles.InvMass[pA];
                Vector2 posA = _particles.Position[pA];

                foreach (int idB in groupB)
                {
                    int pB = _particles.index[idB];

                    Vector2 deltaPos = _particles.Position[pB] - posA;
                    float distSqr = deltaPos.LengthSquared() + _epsilonSqr;
                    float dist = MathF.Sqrt(distSqr);
                    float forceMag = GravitationalConstant * (massA * _particles.Mass[pB]) / distSqr;
                    Vector2 impulse = (deltaPos / dist) * forceMag * dt;

                    if (invMassA > 0)
                        _particles.Impulse[pA] += impulse * invMassA;
                    if (_particles.InvMass[pB] > 0)
                        _particles.Impulse[pB] -= impulse * _particles.InvMass[pB];
                }
            }
        }
    }
}