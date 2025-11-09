using Xunit;
using SquishyPlanet.Objects; // Required to access the internal Particles class
using SquishyPlanet.Utility;
using System.Numerics;

namespace SquishyPlanet.Tests
{
    /// <summary>
    /// These tests validate the internal logic of the Particles class directly.
    /// This requires 'InternalsVisibleTo' to be set in the SquishyPlanet project.
    /// </summary>
    public class ParticlesTests
    {
        // Define common test data to reduce clutter in tests
        private readonly Vector2 _testPos = Vector2.Zero;
        private readonly Vector2 _testVel = Vector2.Zero;
        private readonly ColorRgb _testColor = new(0, 0, 0);
        private const int InvalidId = -1;

        [Fact]
        public void Create_WhenFull_ReturnsInvalidIdAndDoesNotIncreaseCount()
        {
            // Arrange
            var particles = new Particles(maxObjects: 1); // Create a small pool
            // Fill the pool
            particles.Create(1, _testPos, _testVel, 1, 1, _testColor);

            // Act
            // Try to create one more than allowed
            int failedId = particles.Create(1, _testPos, _testVel, 1, 1, _testColor);

            // Assert
            Assert.Equal(1, particles.NumObjects); // Count should not have changed
            Assert.Equal(InvalidId, failedId);      // Method should return -1
        }

        [Fact]
        public void Delete_SwapAndPop_CorrectlyMovesLastParticle()
        {
            // Arrange
            var particles = new Particles(maxObjects: 3);
            var pos1 = new Vector2(1, 1);
            var pos2 = new Vector2(2, 2);
            var pos3 = new Vector2(3, 3);

            // Note: IDs are assigned from a stack (2, 1, 0)
            int id1 = particles.Create(1, pos1, _testVel, 1, 1, _testColor); // Index 0, ID 2
            int id2 = particles.Create(1, pos2, _testVel, 1, 1, _testColor); // Index 1, ID 1
            int id3 = particles.Create(1, pos3, _testVel, 1, 1, _testColor); // Index 2, ID 0

            // Act
            // Delete the particle at Index 1 (id2)
            // This should move particle at Index 2 (id3) into its slot
            bool result = particles.Delete(id2);

            // Assert
            Assert.True(result);
            Assert.Equal(2, particles.NumObjects); // Count decreased

            // Check that particle at index 1 now has the data from old particle 3
            Assert.Equal(pos3, particles.Position[1]);
            Assert.Equal(id3, particles.Id[1]);

            // The lookup map for the moved particle (id3) should be updated
            // We can't access 'index' directly as it's private, but we can test
            // its effect by trying to delete id3. It should now find it at index 1.
            Assert.True(particles.Delete(id3)); // This now deletes the particle at index 1
            Assert.Equal(1, particles.NumObjects);
        }

        [Fact]
        public void Create_AfterDelete_CorrectlyReusesFreedId()
        {
            // Arrange
            var particles = new Particles(maxObjects: 2);
            int id1 = particles.Create(1, _testPos, _testVel, 1, 1, _testColor); // ID 1
            int id2 = particles.Create(1, _testPos, _testVel, 1, 1, _testColor); // ID 0

            // Act
            particles.Delete(id1); // ID 1 is now on the free list
            int newId = particles.Create(1, _testPos, _testVel, 1, 1, _testColor); // Create new one

            // Assert
            // The new particle should have reused the last freed ID
            Assert.Equal(id1, newId);
            Assert.Equal(2, particles.NumObjects); // Back to full
        }

        [Theory]
        [InlineData(-1)]         // Negative ID
        [InlineData(0)]          // Valid ID, but not created
        [InlineData(100)]        // Out of bounds ID
        public void Delete_OnInvalidOrInactiveId_ReturnsFalse(int idToTest)
        {
            // Arrange
            var particles = new Particles(maxObjects: 10);

            // Act
            bool result = particles.Delete(idToTest);

            // Assert
            Assert.False(result);
        }

        [Fact]
        public void Delete_OnAlreadyDeletedId_ReturnsFalse()
        {
            // Arrange
            var particles = new Particles(maxObjects: 10);
            int id = particles.Create(1, _testPos, _testVel, 1, 1, _testColor); // Create ID 9

            // Act
            bool firstDelete = particles.Delete(id); // This one should work
            bool secondDelete = particles.Delete(id); // This one should fail

            // Assert
            Assert.True(firstDelete);
            Assert.False(secondDelete);
        }

        [Fact]
        public void Clear_ResetsNumObjectsAndAllowsCreation()
        {
            // Arrange
            var particles = new Particles(maxObjects: 10);
            particles.Create(1, _testPos, _testVel, 1, 1, _testColor);
            particles.Create(1, _testPos, _testVel, 1, 1, _testColor);
            Assert.Equal(2, particles.NumObjects); // Pre-check

            // Act
            particles.Clear();
            int id = particles.Create(1, _testPos, _testVel, 1, 1, _testColor);

            // Assert
            Assert.Equal(1, particles.NumObjects);
            Assert.Equal(0, id);
        }
    }
}