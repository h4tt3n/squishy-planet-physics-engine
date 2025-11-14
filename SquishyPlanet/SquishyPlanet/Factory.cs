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
    public class Factory
    {
        private readonly Particles _particles;
        private readonly DistanceConstraints _distanceConstraints;
        private readonly AngularConstraints _angularConstraints;

        internal Factory(Particles particles, DistanceConstraints distanceConstraints, AngularConstraints angularConstraints)
        {
            _particles = particles;
            _distanceConstraints = distanceConstraints;
            _angularConstraints = angularConstraints;
        }

        public int CreateParticle(float objectType, Vector2 position, Vector2 velocity,
                                  float mass, float radius, ColorRgb color)
        {
            return _particles.Create(objectType, position, velocity, mass, radius, color);
        }
        
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
        public int CreateAngularConstraint(int distanceConstraintAId, int distanceConstraintBId)
        {
            return _angularConstraints.Create(distanceConstraintAId, distanceConstraintBId);
        }

        public bool DeleteAngularConstraint(int id)
        {
            return _angularConstraints.Delete(id);
        }
    }
}
