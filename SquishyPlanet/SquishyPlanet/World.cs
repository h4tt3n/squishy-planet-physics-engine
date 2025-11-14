using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using SquishyPlanet.Constraints;
using SquishyPlanet.Objects;
using SquishyPlanet.Utility;

namespace SquishyPlanet
{
    public class World
    {
        private readonly Particles _particles;
        private readonly DistanceConstraints _distanceConstraints;
        private readonly AngularConstraints _angularConstraints;

        public readonly Factory Factory;
        public Vector2 Gravity { get; set; } = new(0, 98200.0f);
        public int NumIterations = 10;

        public int NumParticles => _particles.NumObjects;
        public int NumDistanceConstraints => _distanceConstraints.NumObjects;

        public World(int maxParticles, int maxDistanceConstraints, int maxAngularConstraints)
        {
            _particles = new Particles(maxParticles);
            _distanceConstraints = new DistanceConstraints(maxDistanceConstraints, _particles);
            _angularConstraints = new AngularConstraints(maxAngularConstraints, _particles, _distanceConstraints);

            Factory = new Factory(_particles, _distanceConstraints, _angularConstraints);
        }

        public void ComputeData(float invDt)
        {
            _distanceConstraints.ComputeData(invDt);
            _angularConstraints.ComputeData(invDt);
        }

        public void Step(float dt)
        {
            _distanceConstraints.ComputeData(1 / dt);
            _angularConstraints.ComputeData(1 / dt);

            _distanceConstraints.ApplyWarmStart();
            _angularConstraints.ApplyWarmStart();

            for (int i = 0; i < NumIterations; i++)
            {
                _angularConstraints.ApplyCorrectiveImpulse();
                _distanceConstraints.ApplyCorrectiveImpulse();
            }

            ApplyForces(dt);

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
