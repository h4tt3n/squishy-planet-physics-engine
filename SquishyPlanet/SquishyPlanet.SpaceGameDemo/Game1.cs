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
            _world = new World(maxParticles: 100000);

            // *** 5. Create some test particles using the factory ***
            //var color = new ColorRgb(255, 100, 150);
            var staticColor = new ColorRgb(100, 255, 100);

            // Create a few dynamic particles
            var rand = new Random();
            for (int i = 0; i < 50000; i++)
            {
                _world.Factory.CreateParticle(
                    objectType: 1,
                    position: new PhysicsVector2(rand.Next(100, 700), rand.Next(50, 200)),
                    velocity: new PhysicsVector2(rand.Next(-100, 100), rand.Next(-100, 100)),
                    mass: 1.0f,
                    radius: rand.Next(4, 16),
                    color: new ColorRgb((byte)rand.Next(0, 255), (byte)rand.Next(0, 255), (byte)rand.Next(0, 255))
                );
            }

            // Create a static "floor" particle
            _world.Factory.CreateParticle(
                objectType: 0,
                position: new PhysicsVector2(400, 400),
                velocity: PhysicsVector2.Zero,
                mass: 0.0f, // Mass = 0 means static (InvMass will be 0)
                radius: 50.0f,
                color: staticColor
            );

            base.Initialize();
        }

        protected override void LoadContent()
        {
            _spriteBatch = new SpriteBatch(GraphicsDevice);

            // *** 6. Create the 1x1 white pixel texture ***
            // This lets us draw circles of any color and size by tinting and scaling.
            _particleTexture = new Texture2D(GraphicsDevice, 1, 1);
            _particleTexture.SetData(new[] { Color.White });
        }

        protected override void Update(GameTime gameTime)
        {
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed || Keyboard.GetState().IsKeyDown(Keys.Escape))
                Exit();

            // *** 7. Get delta time and step the simulation ***
            float dt = (float)gameTime.ElapsedGameTime.TotalSeconds;
            _world.Step(dt);

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.Black);

            _spriteBatch.Begin();

            // *** 8. Get the data spans from the physics engine ***
            var positions = _world.GetParticlePositions();
            var colors = _world.GetParticleColors();
            var radii = _world.GetParticleRadii();

            // This origin is (0.5, 0.5) which is the center of the 1x1 texture.
            // This makes scaling and rotation happen from the center.
            var textureOrigin = new DrawingVector2(0.5f, 0.5f);

            // *** 9. Loop from 0 to NumParticles and draw ***
            for (int i = 0; i < _world.NumParticles; i++)
            {
                // Get data from the simulation
                PhysicsVector2 simPos = positions[i];
                ColorRgb simColor = colors[i];
                float simRadius = radii[i];

                // --- Convert physics data to drawing data ---

                // Convert System.Numerics.Vector2 to Microsoft.Xna.Framework.Vector2
                var drawPos = new DrawingVector2(simPos.X, simPos.Y);

                // Convert your ColorRgb to MonoGame's Color
                var drawColor = new Color(simColor.R, simColor.G, simColor.B);

                // Calculate the scale. Our texture is 1x1.
                // To get a diameter of (radius * 2), we scale the texture by that amount.
                float scale = simRadius * 2.0f;

                _spriteBatch.Draw(
                    _particleTexture,
                    drawPos,
                    null, // Source rectangle (null for whole texture)
                    drawColor,
                    0f, // Rotation
                    textureOrigin,
                    scale, // Scale
                    SpriteEffects.None,
                    0f // Layer depth
                );
            }

            _spriteBatch.End();

            base.Draw(gameTime);
        }
    }
}