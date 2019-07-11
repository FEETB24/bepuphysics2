using System.Numerics;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;

namespace Demos.Demos
{
    public class BigConveyorDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;
        }
    }
}