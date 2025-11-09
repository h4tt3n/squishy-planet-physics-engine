using Xunit;
using SquishyPlanet;
using System.Numerics;
using SquishyPlanet.Utility;

namespace SquishyPlanet.Tests
{
    public class WorldTests
    {
        private readonly World _world;
        private readonly ColorRgb _testColor = new(0, 0, 0);

        public WorldTests()
        {
            // Arrange (common for all tests)
            _world = new World(maxParticles: 100, maxDistanceConstraints: 0);
        }

        [Fact]
        public void Step_WithGravity_ParticleMovesDown()
        {
            // Arrange
            _world.Gravity = new Vector2(0, 100f);
            int id = _world.Factory.CreateParticle(1, Vector2.Zero, Vector2.Zero, 1, 1, _testColor);

            // Act
            _world.Step(1.0f); // Step for 1 second

            // Assert
            var positions = _world.GetParticlePositions();

            // Per Symplectic Euler:
            // 1. vel = vel + (accel * dt) => 0 + (100 * 1.0) = 100
            // 2. pos = pos + (vel * dt)   => 0 + (100 * 1.0) = 100

            Assert.Equal(100f, positions[0].Y, 5);
        }

        [Fact]
        public void Step_StaticParticle_DoesNotMove()
        {
            // Arrange
            _world.Gravity = new Vector2(0, 100f);
            int id = _world.Factory.CreateParticle(1, Vector2.Zero, Vector2.Zero,
                mass: 0.0f, // Static particle
                radius: 1, color: _testColor);

            // Act
            _world.Step(1.0f);

            // Assert
            var positions = _world.GetParticlePositions();
            Assert.Equal(0f, positions[0].Y, 5);
        }
    }
}