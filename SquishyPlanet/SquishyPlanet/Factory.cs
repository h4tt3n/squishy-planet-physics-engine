using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

using SquishyPlanet.Objects;
using SquishyPlanet.Utility;

namespace SquishyPlanet
{
    public class Factory
    {
        private readonly Particles _particles;
        private readonly DistanceConstraints _distanceConstraints;

        internal Factory(Particles particles, DistanceConstraints distanceConstraints)
        {
            _particles = particles;
            _distanceConstraints = distanceConstraints;
        }

        /// <summary>
        /// Creates a new particle in the simulation.
        /// </summary>
        /// <returns>The stable ID of the new particle, or -1 if full.</returns>
        public int CreateParticle(float objectType, Vector2 position, Vector2 velocity,
                                  float mass, float radius, ColorRgb color)
        {
            return _particles.Create(objectType, position, velocity, mass, radius, color);
        }

        /// <summary>
        /// Deletes a particle from the simulation.
        /// </summary>
        /// <returns>True if the particle was found and deleted.</returns>
        public bool DeleteParticle(int id)
        {
            return _particles.Delete(id);
        }

        public int CreateDistanceConstraint(int idA, int idB, float radius)
        {
            return _distanceConstraints.Create(idA, idB, radius);
        }

        public bool DeleteDistanceConstraint(int id)
        {
            return _distanceConstraints.Delete(id);
        }
    }
}
