using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SquishyPlanet.Utility
{
    public enum ObjectType : byte
    {
        Particle = 1,
        Wheel = 2,
        Constraint = 4,
        FixedConstraint = 8,
        FluidParticle = 16,
        SoftBody = 32,
        FixedConstraintParticle = 64
    }
}
