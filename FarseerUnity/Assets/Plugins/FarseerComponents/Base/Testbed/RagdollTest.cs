using UnityEngine;
using System.Collections;
using FarseerPhysics.Collision;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Collision.ContactSystem;
using VelcroPhysics.Dynamics.Joints;
using VelcroPhysics.Shared;
using VelcroPhysics.Factories;
using VelcroPhysics.Templates;
using VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;

using Transform = UnityEngine.Transform;

namespace CatsintheSky.FarseerDebug
{
	public class RagdollTest : Test
	{
		public RagdollTest(Transform parent) : base(parent)
		{
			this.title = "Ragdolls";
		}
		
		public override void Start()
		{
			base.Start();
			
			CircleShape circ;
			PolygonShape box;
			Fixture fixture;
			RevoluteJoint joint;
			
			// Add 2 ragdolls along the top
			for(int i = 0; i < 2; i++)
			{
				float startX = 70f + Random.value * 20f + 480f * (float)i;
				float startY = (20f + Random.value * 50f) * -1f;

                // Head
                var template2 = new BodyTemplate();
                template2.Type = BodyType.Dynamic;
                Body head = new Body(FSWorldComponent.PhysicsWorld,template2);
				circ = new CircleShape(12.5f / physScale, 1f);
                var fixTemp = new FixtureTemplate();
                fixTemp.Shape = circ;
				fixture = new Fixture(head, fixTemp);

				fixture.Friction = 0.4f;
				fixture.Restitution = 0.3f;
				head.Position = new FVector2(startX / physScale, startY / physScale);
				head.ApplyLinearImpulse(new FVector2(Random.value * 100f - 50f, Random.value * 100f - 50f), head.Position);
				head.BodyType = BodyType.Dynamic;
				
				// Torso 1
				Body torso1 = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(15f / physScale, 10f / physScale),1f);
                var torslTemp = new FixtureTemplate();
                torslTemp.Shape = box;
				fixture = new Fixture(torso1, torslTemp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				torso1.Position = new FVector2(startX / physScale, (startY - 28f) / physScale);
				torso1.BodyType = BodyType.Dynamic;
				
				// Torso 2
				Body torso2 = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(15f / physScale, 10f / physScale), 1f);
                var tors2Temp = new FixtureTemplate();
                tors2Temp.Shape = box;
				fixture = new Fixture(torso2, tors2Temp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				torso2.Position = new FVector2(startX / physScale, (startY - 43f) / physScale);
				torso2.BodyType = BodyType.Dynamic;
				
				// Torso 3
				Body torso3 = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(15f / physScale, 10f / physScale),1f);
                var torso3Temp = new FixtureTemplate();
                torso3Temp.Shape = box;
				fixture = new Fixture(torso3, torso3Temp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				torso3.Position = new FVector2(startX / physScale, (startY - 58f) / physScale);
				torso3.BodyType = BodyType.Dynamic;
				
				// UpperArm
				// L
				Body upperArmL = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(18f / physScale, 6.5f / physScale),1f);
                var upperArmLTemp = new FixtureTemplate();
                upperArmLTemp.Shape = box;
				fixture = new Fixture(upperArmL, upperArmLTemp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				upperArmL.Position = new FVector2((startX - 30f) / physScale, (startY - 20f) / physScale);
				upperArmL.BodyType = BodyType.Dynamic;
				// R
				Body upperArmR = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(18f / physScale, 6.5f / physScale),1f);
                var upperArmRTemp = new FixtureTemplate();
                upperArmRTemp.Shape = box;
				fixture = new Fixture(upperArmR, upperArmRTemp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				upperArmR.Position = new FVector2((startX + 30f) / physScale, (startY - 20f) / physScale);
				upperArmR.BodyType = BodyType.Dynamic;
				
				// LowerArm
				// L
				Body lowerArmL = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(17f / physScale, 6f / physScale), 1f);
                var lowerArmLTemp = new FixtureTemplate();
                lowerArmLTemp.Shape = box;
                fixture = new Fixture(lowerArmL, lowerArmLTemp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				lowerArmL.Position = new FVector2((startX - 57f) / physScale, (startY - 20f) / physScale);
				lowerArmL.BodyType = BodyType.Dynamic;
				// R
				Body lowerArmR = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(17f / physScale, 6f / physScale),1f);
                var lowerArmRTemp = new FixtureTemplate();
                lowerArmRTemp.Shape = box;
                fixture = new Fixture(lowerArmR, lowerArmRTemp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				lowerArmR.Position = new FVector2((startX + 57f) / physScale, (startY - 20f) / physScale);
				lowerArmR.BodyType = BodyType.Dynamic;
				
				// UpperLeg
				// L
				Body upperLegL = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(7.5f / physScale, 22f / physScale),1f);
                var upperLegLTemp = new FixtureTemplate();
                upperLegLTemp.Shape = box;
                fixture = new Fixture(upperLegL, upperLegLTemp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				upperLegL.Position = new FVector2((startX - 8f) / physScale, (startY - 85f) / physScale);
				upperLegL.BodyType = BodyType.Dynamic;
				// R
				Body upperLegR = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(7.5f / physScale, 22f / physScale) ,1f);
                var upperLegRTemp = new FixtureTemplate();
                upperLegRTemp.Shape = box;
                fixture = new Fixture(upperLegR, upperLegRTemp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				upperLegR.Position = new FVector2((startX + 8f) / physScale, (startY - 85f) / physScale);
				upperLegR.BodyType = BodyType.Dynamic;
				
				// LowerLeg
				// L
				Body lowerLegL = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(7.5f / physScale, 22f / physScale), 1f);
                var lowerLegLTemp = new FixtureTemplate();
                lowerLegLTemp.Shape = box;
                fixture = new Fixture(lowerLegL, lowerLegLTemp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				lowerLegL.Position = new FVector2((startX - 8f) / physScale, (startY - 120f) / physScale);
				lowerLegL.BodyType = BodyType.Dynamic;
				// R
				Body lowerLegR = new Body(FSWorldComponent.PhysicsWorld,template2);
				box = new PolygonShape(PolygonUtils.CreateRectangle(7.5f / physScale, 22f / physScale), 1f);
                var lowerLegRTemp = new FixtureTemplate();
                lowerLegRTemp.Shape = box;
                fixture = new Fixture(lowerLegR, lowerLegRTemp);
				fixture.Friction = 0.4f;
				fixture.Restitution = 0.1f;
				lowerLegR.Position = new FVector2((startX + 8f) / physScale, (startY - 120f) / physScale);
				lowerLegR.BodyType = BodyType.Dynamic;
				
				// JOINTS
				
				// Head to shoulders
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, torso1, head, head.GetLocalPoint(new FVector2(startX / physScale, (startY - 15f) / physScale)));
				joint.LowerLimit = -40f * Mathf.Deg2Rad;
				joint.UpperLimit = 40f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
				
				// Upper arm to shoulders
				// L
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, torso1, upperArmL, upperArmL.GetLocalPoint(new FVector2((startX - 18f) / physScale, (startY - 20f) / physScale)));
				joint.LowerLimit = -85f * Mathf.Deg2Rad;
				joint.UpperLimit = 130f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
				// R
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, torso1, upperArmR, upperArmR.GetLocalPoint(new FVector2((startX + 18f) / physScale, (startY - 20f) / physScale)));
				joint.LowerLimit = -130f * Mathf.Deg2Rad;
				joint.UpperLimit = 85f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
				
				// Lower arm to upper arm
				// L
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, upperArmL, lowerArmL, lowerArmL.GetLocalPoint(new FVector2((startX - 45f) / physScale, (startY - 20f) / physScale)));
				joint.LowerLimit = -130f * Mathf.Deg2Rad;
				joint.UpperLimit = 10f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
				// R
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, upperArmR, lowerArmR, lowerArmR.GetLocalPoint(new FVector2((startX + 45f) / physScale, (startY - 20f) / physScale)));
				joint.LowerLimit = -10f * Mathf.Deg2Rad;
				joint.UpperLimit = 130f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
				
				// Shoulders/stomach
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, torso1, torso2, torso2.GetLocalPoint(new FVector2((startX + 0f) / physScale, (startY - 35f) / physScale)));
				joint.LowerLimit = -15f * Mathf.Deg2Rad;
				joint.UpperLimit = 15f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
				// Stomach/hips
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, torso2, torso3, torso3.GetLocalPoint(new FVector2((startX + 0f) / physScale, (startY - 50f) / physScale)));
				joint.LowerLimit = -15f * Mathf.Deg2Rad;
				joint.UpperLimit = 15f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
				
				// Torso to upper leg
				// L
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, torso3, upperLegL, upperLegL.GetLocalPoint(new FVector2((startX - 8f) / physScale, (startY - 72f) / physScale)));
				joint.LowerLimit = -25f * Mathf.Deg2Rad;
				joint.UpperLimit = 45f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
				// R
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, torso3, upperLegR, upperLegR.GetLocalPoint(new FVector2((startX + 8f) / physScale, (startY - 72f) / physScale)));
				joint.LowerLimit = -45f * Mathf.Deg2Rad;
				joint.UpperLimit = 25f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
				
				// Upper leg to lower leg
				// L
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, upperLegL, lowerLegL, lowerLegL.GetLocalPoint(new FVector2((startX - 8f) / physScale, (startY - 105f) / physScale)));
				joint.LowerLimit = -25f * Mathf.Deg2Rad;
				joint.UpperLimit = 115f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
				// R
				joint = JointFactory.CreateRevoluteJoint(FSWorldComponent.PhysicsWorld, upperLegR, lowerLegR, lowerLegR.GetLocalPoint(new FVector2((startX + 8f) / physScale, (startY - 105f) / physScale)));
				joint.LowerLimit = -115f * Mathf.Deg2Rad;
				joint.UpperLimit = 25f * Mathf.Deg2Rad;
				joint.LimitEnabled = true;
				joint.CollideConnected = false;
			}
            var template = new BodyTemplate();
			// Add stairs on the left, these are static bodies so set the type accordingly
			for(int j = 1; j <= 10; j++)
			{
				Body body = new Body(FSWorldComponent.PhysicsWorld, template);
				box = new PolygonShape(PolygonUtils.CreateRectangle((10f * (float)j) / physScale, 10f / physScale), 1f);
                var bodyTemp = new FixtureTemplate();
                bodyTemp.Shape = box;
                fixture = new Fixture(body, bodyTemp);
				body.Position = new FVector2((10f * (float)j) / physScale, ((150f + 20f * (float)j) / physScale) * -1f);
			}
			// Add stairs on the right
			for(int k = 1; k <= 10; k++)
			{
				Body body = new Body(FSWorldComponent.PhysicsWorld,template);
				box = new PolygonShape(PolygonUtils.CreateRectangle((10f * (float)k) / physScale, 10f / physScale), 1f);
                var bodyTemp = new FixtureTemplate();
                bodyTemp.Shape = box;
                fixture = new Fixture(body, bodyTemp);
				body.Position = new FVector2((640f - 10f * (float)k) / physScale, ((150f + 20f * (float)k) / physScale) * -1f);
			}
			
			Body ground = new Body(FSWorldComponent.PhysicsWorld,template);
			box = new PolygonShape(PolygonUtils.CreateRectangle(30f / physScale, 40f / physScale), 1f);
            var groundTemp = new FixtureTemplate();
            groundTemp.Shape = box;
            fixture = new Fixture(ground, groundTemp);
			ground.Position = new FVector2(320f / physScale, (320f / physScale) * -1f);
		}
	}
}

