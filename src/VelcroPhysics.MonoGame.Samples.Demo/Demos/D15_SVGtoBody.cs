﻿using System.Collections.Generic;
using System.Text;
using Genbox.VelcroPhysics.Dynamics;
using Genbox.VelcroPhysics.Factories;
using Genbox.VelcroPhysics.MonoGame.ContentPipelines.SVGImport.Objects;
using Genbox.VelcroPhysics.MonoGame.Samples.Demo.MediaSystem;
using Genbox.VelcroPhysics.MonoGame.Samples.Demo.ScreenSystem;
using Genbox.VelcroPhysics.Shared;
using Genbox.VelcroPhysics.Tools.Triangulation.TriangulationBase;
using Genbox.VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace Genbox.VelcroPhysics.MonoGame.Samples.Demo.Demos
{
    internal class D15_SVGtoBody : PhysicsDemoScreen
    {
        private Sprite _club;
        private Body _clubBody;
        private Sprite _diamond;
        private Body _diamondBody;
        private Sprite _heart;
        private Body _heartBody;
        private VerticesContainer _loadedVertices;
        private Sprite _spade;
        private Body _spadeBody;

        public override void LoadContent()
        {
            base.LoadContent();

            World.Gravity = new Vector2(0f, 10f);

            _loadedVertices = Framework.Content.Load<VerticesContainer>("Pipeline/Body");

            _heartBody = Create(_loadedVertices["Heart"]);
            _heart = new Sprite(ContentWrapper.GetTexture("Heart"), ContentWrapper.CalculateOrigin(_heartBody));

            _clubBody = Create(_loadedVertices["Club"]);
            _club = new Sprite(ContentWrapper.GetTexture("Club"), ContentWrapper.CalculateOrigin(_clubBody));

            _spadeBody = Create(_loadedVertices["Spade"]);
            _spade = new Sprite(ContentWrapper.GetTexture("Spade"), ContentWrapper.CalculateOrigin(_spadeBody));

            _diamondBody = Create(_loadedVertices["Diamond"]);
            _diamond = new Sprite(ContentWrapper.GetTexture("Diamond"), ContentWrapper.CalculateOrigin(_diamondBody));
        }

        private Body Create(List<VerticesExt> ext)
        {
            Body b = BodyFactory.CreateBody(World, bodyType: BodyType.Dynamic);

            foreach (VerticesExt ve in ext)
            {

                List<Vertices> decomposed = Triangulate.ConvexPartition(ve, TriangulationAlgorithm.Bayazit);

                foreach (Vertices v in decomposed)
                {
                    FixtureFactory.AttachPolygon(v, 1, b);
                }
            }

            return b;
        }

        public override void Draw(GameTime gameTime)
        {
            Sprites.Begin(0, null, null, null, null, null, Camera.View);
            Sprites.Draw(_heart.Image, ConvertUnits.ToDisplayUnits(_heartBody.Position), null, Color.White, _heartBody.Rotation, _heart.Origin, 1f, SpriteEffects.None, 0f);
            Sprites.Draw(_club.Image, ConvertUnits.ToDisplayUnits(_clubBody.Position), null, Color.White, _clubBody.Rotation, _club.Origin, 1f, SpriteEffects.None, 0f);
            Sprites.Draw(_spade.Image, ConvertUnits.ToDisplayUnits(_spadeBody.Position), null, Color.White, _spadeBody.Rotation, _spade.Origin, 1f, SpriteEffects.None, 0f);
            Sprites.Draw(_diamond.Image, ConvertUnits.ToDisplayUnits(_diamondBody.Position), null, Color.White, _diamondBody.Rotation, _diamond.Origin, 1f, SpriteEffects.None, 0f);
            Sprites.End();

            base.Draw(gameTime);
        }

        public override string GetTitle()
        {
            return "SVG Importer to bodies";
        }

        public override string GetDetails()
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendLine("This demo shows how to load bodies from a SVG.");
            sb.AppendLine();
            sb.AppendLine("GamePad:");
            sb.Append("  - Exit to demo selection: Back button");
#if WINDOWS
            sb.AppendLine();
            sb.AppendLine();
            sb.AppendLine("Keyboard:");
            sb.AppendLine("  - Exit to demo selection: Escape");
#endif
            return sb.ToString();
        }
    }
}