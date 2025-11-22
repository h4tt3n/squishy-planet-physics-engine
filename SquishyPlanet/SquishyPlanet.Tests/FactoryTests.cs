using Xunit;
using SquishyPlanet;
using System.Numerics;
using SquishyPlanet.Utility;

namespace SquishyPlanet.Tests
{
    public class FactoryTests
    {
        private readonly World _world;
        private readonly ColorRgb _testColor = new(0, 0, 0);

        public FactoryTests()
        {
            _world = new World(maxParticles: 2, maxDistanceConstraints: 0, maxAngularConstraints: 0, maxParticleParticleCollisions: 0); // Small world for limit testing
        }

        [Fact]
        public void CreateParticle_WhenFull_ReturnsInvalidId()
        {
            // Arrange
            _world.Factory.CreateParticle(1, Vector2.Zero, Vector2.Zero, 1, 1, _testColor);
            _world.Factory.CreateParticle(1, Vector2.Zero, Vector2.Zero, 1, 1, _testColor);

            // Act: Try to create one more than max
            int invalidId = _world.Factory.CreateParticle(1, Vector2.Zero, Vector2.Zero, 1, 1, _testColor);

            // Assert
            Assert.Equal(-1, invalidId);
        }

        [Theory]
        [InlineData(-1, false)]  // Test deleting negative ID
        [InlineData(100, false)] // Test deleting out-of-bounds ID
        [InlineData(0, true)]    // Test deleting valid ID
        public void DeleteParticle_EdgeCases_ReturnsCorrectStatus(int idToDelete, bool expectedResult)
        {
            // Arrange
            _world.Factory.CreateParticle(1, Vector2.Zero, Vector2.Zero, 1, 1, _testColor); // This is ID 0

            // Act
            bool result = _world.Factory.DeleteParticle(idToDelete);

            // Assert
            Assert.Equal(expectedResult, result);
        }

        [Fact]
        public void DeleteParticle_DeletesCorrectParticle_ReturnsTrue()
        {
            // Arrange
            var pos1 = new Vector2(1, 1);
            var pos2 = new Vector2(2, 2);
            int id1 = _world.Factory.CreateParticle(1, pos1, Vector2.Zero, 1, 1, _testColor);
            int id2 = _world.Factory.CreateParticle(1, pos2, Vector2.Zero, 1, 1, _testColor);

            // Act
            bool result = _world.Factory.DeleteParticle(id1);

            // Assert
            Assert.True(result);
            Assert.Equal(1, _world.NumParticles);
            var positions = _world.GetParticlePositions();
            Assert.Equal(pos2.X, positions[0].X); // The remaining particle should be pos2
        }
    }
}