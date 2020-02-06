using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using Quaternion = System.Numerics.Quaternion;

namespace Demos.Demos
{
    public class CraneDemo : Demo
    {
        private BodyReference _boxReference;
        private BodyReference _baseReference;

        private bool _baseUp = true;

        private int _jointIndex = -1;
        private int _linearMotorIndex = -1;
        private int _angularHingeIndex = -1;


        private float _jointDistance = 5;

        private DefaultPoseIntegratorCallbacks _poseIntegrator;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            
            _poseIntegrator = new DefaultPoseIntegratorCallbacks(BufferPool);

            Simulation = Simulation.Create(BufferPool, new DefaultNarrowPhaseCallbacks(), _poseIntegrator);


            var boxShape = new Box(1, 1, 1);
            var baseShape = new Box(5, 1, 5);


            boxShape.ComputeInertia(1000, out var boxInertia);

            var boxShapeIndex = Simulation.Shapes.Add(boxShape);
            var baseShapeIndex = Simulation.Shapes.Add(baseShape);

            Symmetric3x3.Scale(boxInertia.InverseInertiaTensor, .5f, out boxInertia.InverseInertiaTensor);

            var boxDescription = new BodyDescription
            {
                //Make the uppermost block kinematic to hold up the rest of the chain.
                LocalInertia = boxInertia,
                Pose = new RigidPose
                {
                    Position = new Vector3(0, -2, 0),
                    Orientation = Quaternion.Identity
                },
                Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = .01f },
                Collidable = new CollidableDescription { Shape = boxShapeIndex, SpeculativeMargin = .1f },
            };


            var baseDescription = new BodyDescription
            {
                //Make the uppermost block kinematic to hold up the rest of the chain.
                LocalInertia = new BodyInertia(),
                Pose = new RigidPose
                {
                    Position = new Vector3(0, 0, 0),
                    Orientation = Quaternion.Identity
                },
                Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = .01f },
                Collidable = new CollidableDescription { Shape = baseShapeIndex, SpeculativeMargin = .1f },
            };

            var boxIndex = Simulation.Bodies.Add(boxDescription);
            var baseIndex = Simulation.Bodies.Add(baseDescription);

            _boxReference = new BodyReference(boxIndex, Simulation.Bodies);
            _baseReference = new BodyReference(baseIndex, Simulation.Bodies);





        }

        private void RemoveContrait(ref int index)
        {
            if (index != -1)
            {
                Simulation.Solver.Remove(index);
                index = -1;
            }

        }

        unsafe void ChangeLenght(float lenght)
        {
            var distanceJoint = new DistanceLimit(Vector3.Zero, Vector3.Zero, lenght, lenght, new SpringSettings(10f, 10f));
            Simulation.Solver.ApplyDescription(_jointIndex, ref distanceJoint);
        }


        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.Z))
            {
                _jointDistance++;
                ChangeLenght(_jointDistance);

            }

            if (input.WasPushed(OpenTK.Input.Key.X))
            {
                _jointDistance--;
                ChangeLenght(_jointDistance);

            }


            if (input.WasPushed(OpenTK.Input.Key.C))
            {
                if (_jointIndex == -1)
                {
                    var baseIndex = _baseReference.Handle;
                    var boxIndex = _boxReference.Handle;



                    var distanceJoint = new DistanceLimit(Vector3.Zero, new Vector3(0, .51f, 0), 5f, 5f, new SpringSettings(5, 1));
                    _jointIndex = Simulation.Solver.Add(baseIndex, boxIndex, distanceJoint);

                    var angularSwivelHinge = new AngularHinge() { LocalHingeAxisA = Vector3.UnitY, LocalHingeAxisB = Vector3.UnitY, SpringSettings = new SpringSettings(20f, .0001f) };
                    _angularHingeIndex =  Simulation.Solver.Add(baseIndex, boxIndex, angularSwivelHinge);


                    var linearMotor = new OneBodyLinearMotor() { TargetVelocity = new Vector3(0, 0, 0), Settings = new MotorSettings(20, 1f) };
                    _linearMotorIndex = Simulation.Solver.Add(boxIndex, linearMotor);

                    _poseIntegrator.CustomAngularDamping(boxIndex, .5f, BufferPool);
                    _poseIntegrator.CustomLinearDamping(boxIndex, .5f, BufferPool);
                }
            }


            if (input.WasPushed(OpenTK.Input.Key.V))
            {
                RemoveContrait(ref _jointIndex);
                RemoveContrait(ref _linearMotorIndex);
                RemoveContrait(ref _angularHingeIndex);
            }


            var hasVelocity = false;

            var newVelocity = Vector3.Zero;
            if (input.WasDown(OpenTK.Input.Key.Left))
            {
                hasVelocity = true;
                newVelocity.X += 2;
            }

            if (input.WasDown(OpenTK.Input.Key.Right))
            {
                hasVelocity = true;
                newVelocity.X += -2;
            }

            _baseReference.Velocity.Linear = newVelocity;

            if (hasVelocity)
                Simulation.Awakener.AwakenBody(_baseReference.Handle);



            base.Update(window, camera, input, dt);
        }

        
    }
}