using Raylib_cs;
using System.Numerics;
using SquishyPlanet;
using SquishyPlanet.Utility;
using System;

namespace SpaceGameRayLib
{
    static class Program
    {
        public static void Main()
        {
            const int ScreenWidth = 1280;
            const int ScreenHeight = 720;
            const float dt = 0.02f;

            // Set MSAA 4x (smooths jagged edges of geometry)
            Raylib.SetConfigFlags(ConfigFlags.Msaa4xHint);

            Raylib.InitWindow(ScreenWidth, ScreenHeight, "SquishyPlanet - RayLib Batching Fixed");

            var world = new World(
                maxParticles: 100000,
                maxDistanceConstraints: 100000,
                maxAngularConstraints: 100000,
                maxParticleParticleCollisions: 100000
            );

            world.Gravity = new Vector2(0, 500f);

            // --- Setup Test Data ---
            var staticColor = new ColorRgb(100, 255, 100);
            var rand = new Random();

            for (int i = 0; i < 1200; i++)
            {
                float mass = rand.Next(2, 8);
                world.Factory.CreateParticle(
                    objectType: 1,
                    position: new Vector2(rand.Next(100, 1180), rand.Next(100, 620)),
                    velocity: Vector2.Zero,
                    mass: mass,
                    radius: mass,
                    color: new ColorRgb((byte)rand.Next(0, 255), (byte)rand.Next(0, 255), (byte)rand.Next(0, 255))
                );
            }

            // Static floor
            world.Factory.CreateParticle(0, new Vector2(400, 0), Vector2.Zero, 0.0f, 50.0f, staticColor);

            // Create distance constraints
            //for (int i = 0; i < world.NumParticles; i++)
            //{
            //    int idA = i;
            //    int idB = (i + 1) % world.NumParticles; // Connects last particle back to first
            //    int radius = 4; // Line thickness

            //    world.Factory.CreateDistanceConstraint(idA, idB, radius);
            //}

            //world.ComputeData(1 / dt);

            //for (int i = 0; i < world.NumDistanceConstraints; i++)
            //{
            //    int idA = i;
            //    int idB = (i + 1) % world.NumDistanceConstraints;

            //    world.Factory.CreateAngularConstraint(idA, idB);
            //}

            //world.ComputeData(1 / dt);


            // *** FIX 1: Generate Higher Resolution Texture ***
            // 256x256 allows radius up to 128 without upscaling artifacts.
            // Downscaling (drawing small particles) looks much better than upscaling.
            int texSize = 256;
            int texCenter = texSize / 2;
            int texRadius = texCenter - 2; // Leave slight padding

            Image circleImg = Raylib.GenImageColor(texSize, texSize, Color.Blank);
            Raylib.ImageDrawCircle(ref circleImg, texCenter, texCenter, texRadius, Color.White);
            Texture2D particleTexture = Raylib.LoadTextureFromImage(circleImg);
            Raylib.SetTextureFilter(particleTexture, TextureFilter.Bilinear);
            Raylib.UnloadImage(circleImg);

            // Generate smaller versions of the texture for the GPU to use when scaling down
            Raylib.GenTextureMipmaps(ref particleTexture);

            // This blends pixels together when rotated or scaled, smoothing the look.
            Raylib.SetTextureFilter(particleTexture, TextureFilter.Bilinear);

            // Use Trilinear filtering for the best quality when scaling down
            //Raylib.SetTextureFilter(particleTexture, TextureFilter.Trilinear);

            // Generate 1x1 White Pixel for lines
            Image pixelImg = Raylib.GenImageColor(1, 1, Color.White);
            Texture2D pixelTexture = Raylib.LoadTextureFromImage(pixelImg);
            Raylib.UnloadImage(pixelImg);

            // Pre-calculate Source Rectangles (These don't change)
            Rectangle sourceRectCircle = new Rectangle(0, 0, particleTexture.Width, particleTexture.Height);
            Rectangle sourceRectPixel = new Rectangle(0, 0, pixelTexture.Width, pixelTexture.Height);

            Color constraintColor = new Color((byte)50, (byte)50, (byte)80, (byte)255);
            const float Rad2Deg = 180.0f / MathF.PI;

            while (!Raylib.WindowShouldClose())
            {
                world.Step(dt);

                Raylib.BeginDrawing();
                Raylib.ClearBackground(Color.Black);

                // --- 1. Draw Constraints ---
                var constraintA = world.GetConstraintParticleA_IDs();
                var constraintB = world.GetConstraintParticleB_IDs();
                var constraintRadii = world.GetConstraintRadii();

                for (int i = 0; i < world.NumDistanceConstraints; i++)
                {
                    Vector2 posA = world.GetParticlePositionById(constraintA[i]);
                    Vector2 posB = world.GetParticlePositionById(constraintB[i]);

                    Vector2 delta = posB - posA;
                    float lengthSqr = delta.LengthSquared();

                    if (lengthSqr < 0.0001f) continue;

                    float length = MathF.Sqrt(lengthSqr);
                    float thickness = constraintRadii[i];
                    float rotation = MathF.Atan2(delta.Y, delta.X) * Rad2Deg;

                    // Draw Line Texture
                    Rectangle dest = new Rectangle(posA.X, posA.Y, length, thickness);
                    Vector2 origin = new Vector2(0, thickness * 0.5f); // Pivot left-center
                    Raylib.DrawTexturePro(pixelTexture, sourceRectPixel, dest, origin, rotation, constraintColor);
                }

                // --- 2. Draw Particles (Fixed Centering) ---
                var positions = world.GetParticlePositions();
                var colors = world.GetParticleColors();
                var radii = world.GetParticleRadii();
                int numParticles = world.NumParticles;

                for (int i = 0; i < numParticles; i++)
                {
                    Color rayColor = new Color(colors[i].R, colors[i].G, colors[i].B, (byte)255);
                    Vector2 pos = positions[i];
                    float radius = radii[i];

                    // Destination size
                    float diameter = radius * 2.0f;

                    Rectangle dest = new Rectangle(pos.X, pos.Y, diameter, diameter);

                    // *** FIX 2: Calculate Origin per Particle ***
                    // The origin is relative to the DESTINATION rectangle size.
                    // To rotate/scale around the center, origin must be (width/2, height/2).
                    Vector2 origin = new Vector2(diameter / 2.0f, diameter / 2.0f);

                    Raylib.DrawTexturePro(particleTexture, sourceRectCircle, dest, origin, 0f, rayColor);
                }

                Raylib.EndDrawing();
            }

            Raylib.UnloadTexture(particleTexture);
            Raylib.UnloadTexture(pixelTexture);
            Raylib.CloseWindow();
        }
    }
}