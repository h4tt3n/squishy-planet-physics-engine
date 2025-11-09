using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;

// *** 1. Add using statements for your physics engine ***
using SquishyPlanet;
using SquishyPlanet.Utility;

// *** 2. Add this to handle the Vector2 name conflict ***
// This is critical because MonoGame and System.Numerics both have a 'Vector2' type.
using PhysicsVector2 = System.Numerics.Vector2;
using DrawingVector2 = Microsoft.Xna.Framework.Vector2;

namespace SquishyPlanet.SpaceGameDemo
{

    public class Game1 : Game
    {
        private GraphicsDeviceManager _graphics;
        private SpriteBatch _spriteBatch;

        // *** 3. Engine and rendering fields ***
        private World _world;
        private Texture2D _particleTexture; // A 1x1 white pixel for drawing

        public Game1()
        {
            _graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";
            IsMouseVisible = true;
        }

        protected override void Initialize()
        {
            // *** 4. Instantiate your physics world ***
            // This constructor assumes World.cs was updated to accept maxDistanceConstraints
            _world = new World(maxParticles: 100000, maxDistanceConstraints: 100000);

            // Set gravity (in pixels/sec^2). 9.81 is too slow for screens.
            _world.Gravity = new PhysicsVector2(0, 500f);

            // *** 5. Create some test particles using the factory ***
            var staticColor = new ColorRgb(100, 255, 100);

            // Create a few dynamic particles
            var rand = new Random();
            for (int i = 0; i < 10000; i++)
            {
                _world.Factory.CreateParticle(
                    objectType: 1,
                    position: new PhysicsVector2(rand.Next(100, 700), rand.Next(50, 200)),
                    velocity: new PhysicsVector2(rand.Next(-100, 100), rand.Next(-100, 100)),
                    mass: 1.0f,
                    radius: rand.Next(4, 8),
                    color: new ColorRgb((byte)rand.Next(0, 255), (byte)rand.Next(0, 255), (byte)rand.Next(0, 255))
                );
            }

            // Create a static "floor" particle
            _world.Factory.CreateParticle(
                objectType: 0,
                position: new PhysicsVector2(400, 0),
                velocity: PhysicsVector2.Zero,
                mass: 0.0f, // Mass = 0 means static (InvMass will be 0)
                radius: 50.0f,
                color: staticColor
            );

            // Create constraints
            for (int i = 0; i < _world.NumParticles; i++)
            {
                int idA = i;
                int idB = (i + 1) % _world.NumParticles; // Connects last particle back to first
                int radius = 10; // Line thickness

                _world.Factory.CreateDistanceConstraint(idA, idB, radius);
            }

            base.Initialize();
        }

        protected override void LoadContent()
        {
            _spriteBatch = new SpriteBatch(GraphicsDevice);

            // *** 6. Create the 1x1 white pixel texture ***
            _particleTexture = new Texture2D(GraphicsDevice, 1, 1);
            _particleTexture.SetData(new[] { Color.White });
        }

        protected override void Update(GameTime gameTime)
        {
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed || Keyboard.GetState().IsKeyDown(Keys.Escape))
                Exit();

            // *** 7. Get delta time and step the simulation ***
            float dt = 0.01f; // Using fixed time step
            _world.Step(dt);

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.Black);

            _spriteBatch.Begin();

            // --- 1. DRAW CONSTRAINTS ---
            // (Draw these first so particles draw on top)

            var constraintParticleA_IDs = _world.GetConstraintParticleA_IDs();
            var constraintParticleB_IDs = _world.GetConstraintParticleB_IDs();
            var constraintRadii = _world.GetConstraintRadii();
            var constraintColor = new Color(50, 50, 80); // Dim color for constraints

            // This origin is (0, 0.5), which is the center of the left edge.
            // This makes the texture rotate from its starting point.
            var lineOrigin = new DrawingVector2(0f, 0.5f);

            for (int i = 0; i < _world.NumDistanceConstraints; i++)
            {
                // Get the stable IDs
                int idA = constraintParticleA_IDs[i];
                int idB = constraintParticleB_IDs[i];
                float radius = constraintRadii[i];

                // Use the new lookup method to get current positions
                PhysicsVector2 simPosA = _world.GetParticlePositionById(idA);
                PhysicsVector2 simPosB = _world.GetParticlePositionById(idB);

                // Convert to drawing vectors
                var drawPosA = new DrawingVector2(simPosA.X, simPosA.Y);
                var drawPosB = new DrawingVector2(simPosB.X, simPosB.Y);

                // Calculate distance, angle, and scale
                DrawingVector2 delta = drawPosB - drawPosA;
                float length = delta.Length();
                float rotation = (float)Math.Atan2(delta.Y, delta.X);

                // Scale.X is the line length, Scale.Y is the line thickness (radius)
                var scale = new DrawingVector2(length, radius);

                _spriteBatch.Draw(
                    _particleTexture,
                    drawPosA, // Start drawing at the first particle's position
                    null,
                    constraintColor,
                    rotation,
                    lineOrigin,
                    scale,
                    SpriteEffects.None,
                    0f
                );
            }

            // --- 2. DRAW PARTICLES ---

            var positions = _world.GetParticlePositions();
            var colors = _world.GetParticleColors();
            var radii = _world.GetParticleRadii();
            var particleOrigin = new DrawingVector2(0.5f, 0.5f); // Center of texture

            for (int i = 0; i < _world.NumParticles; i++)
            {
                PhysicsVector2 simPos = positions[i];
                ColorRgb simColor = colors[i];
                float simRadius = radii[i];

                var drawPos = new DrawingVector2(simPos.X, simPos.Y);
                var drawColor = new Color(simColor.R, simColor.G, simColor.B);
                float scale = simRadius * 2.0f;

                _spriteBatch.Draw(
                    _particleTexture,
                    drawPos,
                    null,
                    drawColor,
                    0f,
                    particleOrigin,
                    scale,
                    SpriteEffects.None,
                    0f
                );
            }

            _spriteBatch.End();

            base.Draw(gameTime);
        }
    }
}