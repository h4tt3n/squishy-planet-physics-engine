using SquishyPlanet.Objects;
using SquishyPlanet.Utility;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SquishyPlanet
{
    public class World
    {
        private readonly Particles _particles;

        public readonly Factory Factory;
        public Vector2 Gravity { get; set; } = new(0, 9.82f);

        public int NumParticles => _particles.NumObjects;

        public World(int maxParticles)
        {
            _particles = new Particles(maxParticles);

            Factory = new Factory(_particles);
        }

        public void Step(float dt)
        {
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
    }
}
