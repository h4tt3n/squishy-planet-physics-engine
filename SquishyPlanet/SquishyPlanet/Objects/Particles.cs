using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

using SquishyPlanet.Utility;

namespace SquishyPlanet.Objects
{
    internal class Particles
    {
        private const int InvalidId = -1;

        public int NumObjects { get; private set; }
        public readonly int MaxObjects;

        // ID Management
        private int numFreeIds;
        private readonly int[] nextFreeId;
        public readonly int[] index;
        public readonly int[] Id;

        // Particle Properties
        public readonly float[] ObjectType;
        public readonly Vector2[] Position;
        public readonly Vector2[] RestPosition;
        public readonly Vector2[] Velocity;
        public readonly Vector2[] Impulse;
        public readonly float[] Mass;
        public readonly float[] InvMass;

        public readonly float[] Density;
        public readonly float[] SumDistances;
        public readonly float[] SumVelocities;
        public readonly float[] NumConstraints;

        public readonly float[] Radius;
        public readonly float[] InteractionRadius;
        public readonly ColorRgb[] Color;

        public Particles(int maxObjects)
        {
            MaxObjects = maxObjects;
            NumObjects = 0;
            numFreeIds = maxObjects;

            nextFreeId = new int[maxObjects];
            index = new int[maxObjects];
            Id = new int[maxObjects];

            ObjectType = new float[maxObjects];
            Position = new Vector2[maxObjects];
            RestPosition = new Vector2[maxObjects];
            Velocity = new Vector2[maxObjects];
            Impulse = new Vector2[maxObjects];
            Mass = new float[maxObjects];
            InvMass = new float[maxObjects];
            Density = new float[maxObjects];
            SumDistances = new float[maxObjects];
            SumVelocities = new float[maxObjects];
            NumConstraints = new float[maxObjects];
            Radius = new float[maxObjects];
            InteractionRadius = new float[maxObjects];
            Color = new ColorRgb[maxObjects];

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

            ObjectType.AsSpan().Clear();
            Position.AsSpan().Clear();
            RestPosition.AsSpan().Clear();
            Velocity.AsSpan().Clear();
            Impulse.AsSpan().Clear();
            Mass.AsSpan().Clear();
            InvMass.AsSpan().Clear();
            Density.AsSpan().Clear();
            SumDistances.AsSpan().Clear();
            SumVelocities.AsSpan().Clear();
            NumConstraints.AsSpan().Clear();
            Radius.AsSpan().Clear();
            InteractionRadius.AsSpan().Clear();
            Color.AsSpan().Clear();
        }

        public int Create(float objectType, Vector2 position, Vector2 velocity,
                          float mass, float radius, ColorRgb color)
        {
            if (NumObjects == MaxObjects)
            {
                return InvalidId;
            }

            // Get the next available ID and Index
            int createId = nextFreeId[numFreeIds - 1];
            int createIndex = NumObjects;

            numFreeIds--;

            // Update lookup maps
            index[createId] = createIndex;
            Id[createIndex] = createId;

            NumObjects++;

            // Set properties at the new index
            ObjectType[createIndex] = objectType;
            Position[createIndex] = position;
            RestPosition[createIndex] = position;
            Velocity[createIndex] = velocity;
            Impulse[createIndex] = Vector2.Zero;
            Mass[createIndex] = mass;
            InvMass[createIndex] = mass > 0 ? 1.0f / mass : 0.0f;

            Density[createIndex] = 0;
            SumDistances[createIndex] = 0;
            SumVelocities[createIndex] = 0;
            NumConstraints[createIndex] = 0;

            Radius[createIndex] = radius;
            InteractionRadius[createIndex] = radius + 0.5f;
            Color[createIndex] = color;

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

            // Get the ID and index of the last active particle
            int lastIndex = NumObjects - 1;
            int lastId = Id[lastIndex];

            Id[deleteIndex] = Id[lastIndex];
            ObjectType[deleteIndex] = ObjectType[lastIndex];
            Position[deleteIndex] = Position[lastIndex];
            RestPosition[deleteIndex] = RestPosition[lastIndex];
            Velocity[deleteIndex] = Velocity[lastIndex];
            Impulse[deleteIndex] = Impulse[lastIndex];
            Mass[deleteIndex] = Mass[lastIndex];
            InvMass[deleteIndex] = InvMass[lastIndex];
            Density[deleteIndex] = Density[lastIndex];
            SumDistances[deleteIndex] = SumDistances[lastIndex];
            SumVelocities[deleteIndex] = SumVelocities[lastIndex];
            NumConstraints[deleteIndex] = NumConstraints[lastIndex];
            Radius[deleteIndex] = Radius[lastIndex];
            InteractionRadius[deleteIndex] = InteractionRadius[lastIndex];
            Color[deleteIndex] = Color[lastIndex];

            // Update the lookup map
            index[lastId] = deleteIndex;

            index[id] = InvalidId;

            NumObjects--;

            // Add the now-free ID back to the stack
            nextFreeId[numFreeIds] = id;
            numFreeIds++;

            return true;
        }

        public void Step(float dt)
        {
            Parallel.For(0, NumObjects, i =>
            {
                if (InvMass[i] > 0)
                {
                    Velocity[i] += Impulse[i];
                    Position[i] += Velocity[i] * dt;
                }

                Impulse[i] = Vector2.Zero;
                Density[i] = 0.0f;
                SumVelocities[i] = 0.0f;
                SumDistances[i] = 0.0f;
            });
        }
    }
}
