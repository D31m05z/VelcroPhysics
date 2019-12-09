using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using VelcroPhysics.Collision.ContactSystem;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Dynamics.Solver;
using VelcroPhysics.Factories;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Dynamics
{
    public class BodyPart
    {
        public Fixture Fixture { get; set; }
        public Body Body { get; set; }
        public float Life { get; set; }
        public bool Destroying { get; set; }

        public BodyPart(Fixture fixture)
        {
            Fixture = fixture;

            Life = (float)GenRand(1.0, 5.0);
            Destroying = false;
        }

        static double GenRand(double one, double two)
        {
            Random rand = new Random();
            return one + rand.NextDouble() * (two - one);
        }
    }

    /// <summary>
    /// A type of body that supports multiple fixtures that can break apart.
    /// </summary>
    public class BreakableBody
    {
        private float[] _angularVelocitiesCache = new float[8];
        private bool _break;
        private Vector2[] _velocitiesCache = new Vector2[8];
        private readonly World _world;

        /// <summary>
        /// The force needed to break the body apart.
        /// Default: 500
        /// </summary>
        public float Strength { get; set; }

        public BreakableBody(World world, IEnumerable<Vertices> vertices, float density, Vector2 position = new Vector2(), float rotation = 0)
        {
            _world = world;
            _world.ContactManager.PostSolve += PostSolve;
            BodyParts = new List<BodyPart>(8);
            MainBody = BodyFactory.CreateBody(_world, position, rotation, BodyType.Dynamic);
            Strength = 500.0f;

            foreach (Vertices part in vertices)
            {
                PolygonShape polygonShape = new PolygonShape(part, density);
                BodyPart bodyPart = new BodyPart(MainBody.CreateFixture(polygonShape));
                BodyParts.Add(bodyPart);
            }
        }

        public BreakableBody(World world, IEnumerable<Shape> shapes, Vector2 position = new Vector2(), float rotation = 0)
        {
            _world = world;
            _world.ContactManager.PostSolve += PostSolve;
            MainBody = BodyFactory.CreateBody(_world, position, rotation, BodyType.Dynamic);
            BodyParts = new List<BodyPart>(8);

            foreach (Shape part in shapes)
            {
                BodyPart bodyPart = new BodyPart(MainBody.CreateFixture(part));
                BodyParts.Add(bodyPart);
            }
        }

        public bool Broken { get; private set; }
        public Body MainBody { get; }
        public List<BodyPart> BodyParts { get; }
        public Vector2 ContactVector { get; set; }
        public Contact ConcactFixture { get; set; }

        private void PostSolve(Contact contact, ContactVelocityConstraint impulse)
        {
            if (!Broken)
            {
                if (BodyParts.Exists(body => body.Fixture == contact.FixtureA || body.Fixture == contact.FixtureB))
                {
                    float maxImpulse = 0.0f;
                    int count = contact.Manifold.PointCount;
                    bool rAY = false;
                    for (int i = 0; i < count; ++i)
                    {
                        maxImpulse = Math.Max(maxImpulse, impulse.Points[i].NormalImpulse);
                        if (impulse.Points[i].rA.Y < 0) rAY = true;
                    }

                    if (maxImpulse > Strength && rAY)
                    {
                        // Flag the body for breaking.
                        _break = true;
                        ContactVector = impulse.Normal * maxImpulse / 2.0f;
                        ConcactFixture = contact;
                    }
                }
            }
        }

        public void Update()
        {
            if (_break)
            {
                Decompose();
                Broken = true;
                _break = false;
            }

            // Cache velocities to improve movement on breakage.
            if (Broken == false)
            {
                //Enlarge the cache if needed
                if (BodyParts.Count > _angularVelocitiesCache.Length)
                {
                    _velocitiesCache = new Vector2[BodyParts.Count];
                    _angularVelocitiesCache = new float[BodyParts.Count];
                }

                //Cache the linear and angular velocities.
                for (int i = 0; i < BodyParts.Count; i++)
                {
                    _velocitiesCache[i] = BodyParts[i].Fixture.Body.LinearVelocity;
                    _angularVelocitiesCache[i] = BodyParts[i].Fixture.Body.AngularVelocity;
                }
            }
        }

        private void Decompose()
        {
            //Unsubsribe from the PostSolve delegate
            _world.ContactManager.PostSolve -= PostSolve;

            for (int i = 0; i < BodyParts.Count; i++)
            {
                Fixture oldFixture = BodyParts[i].Fixture;

                Shape shape = oldFixture.Shape.Clone();
                object userData = oldFixture.UserData;

                MainBody.DestroyFixture(oldFixture);

                BodyParts[i].Body = BodyFactory.CreateBody(_world, MainBody.Position, MainBody.Rotation, BodyType.Dynamic, MainBody.UserData);

                Fixture newFixture = BodyParts[i].Body.CreateFixture(shape);
                newFixture.UserData = userData;
                BodyParts[i].Fixture = newFixture;

                BodyParts[i].Body.AngularVelocity = _angularVelocitiesCache[i];
                BodyParts[i].Body.LinearVelocity = _velocitiesCache[i];
                BodyParts[i].Body.ApplyLinearImpulse(ContactVector);
            }

            _world.RemoveBody(MainBody);
            _world.RemoveBreakableBody(this);
        }

        public void Break()
        {
            _break = true;
        }
    }
}