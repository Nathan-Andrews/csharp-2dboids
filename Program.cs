using System;
using System.Linq;
using System.Diagnostics;
using System.Threading;
using System.Collections.Generic;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;



namespace Boids
{
    public static class Settings {
        public static int[] WindowSize =  new int[] {900,900};
        public static int BoidCount = 100;
        public static void UpdateWindow(int x, int y) {
            WindowSize[0] = x;
            WindowSize[1] = y;
        }
    }

    public static class RandomExtensions {
        public static float NextFloat(this Random random, float min, float max) {
            return ((float) (random.NextDouble() * (min - max) + max));
        }
    }

    public static class Spawner {
        public static List<Boid> spawnBoids(int count) {
            Random random = new Random();
            List<Boid> boids = new List<Boid>();
            for (int i = 0; i < count; i++) {
                boids.Add(new Boid(new float[] {random.NextFloat(-1.0f,1.0f),random.NextFloat(-1.0f,1.0f)}, random.NextFloat(0.0f,360.0f)));
            }

            return boids;
        }
    }
    

    public class Boid {
        private float[] vertices;
        private float[] position;
        private float rotation; //degrees
        private float[] color;

        //constants
        private float speed = 0.2f;
        private float size = 0.02f; //0.02f
        private float viewRadius = 0.4f; //0.4
        private float viewAngle = 30.0f;
        private float collisionAngle = 40.0f;

        private Random random = new Random();

        private int vertexBufferObject;
        private int vertexArrayObject;

        public Boid(float[] position, float rotation = 0.0f) {

            this.position = position;
            this.rotation = rotation;

            color = new float[] {random.NextFloat(0.0f,0.3f),random.NextFloat(0.2f,0.4f),random.NextFloat(0.5f,1.0f)};

            //speed = random.NextFloat(speed * 0.9f, speed * 1.1f);

            this.vertices = new float[] {
                0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f
            };
            this.vertices = UpdateVertices();

            vertexBufferObject = GL.GenBuffer();
            GL.BindBuffer(BufferTarget.ArrayBuffer, vertexBufferObject);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Length * sizeof(float), vertices, BufferUsageHint.DynamicDraw);

            vertexArrayObject = GL.GenVertexArray();
            GL.BindVertexArray(vertexArrayObject);

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 3 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);
        }

        public void Draw(Shader shader, List<Boid> boids) {
            shader.Use();
            int vertexColorLocation = GL.GetUniformLocation(shader.Handle, "VertexColor"); //gets location of uniform in frag shader

            if (boids[0] == this) {
                GL.Uniform3(vertexColorLocation, 0.0f, 1.0f, 0.0f);
            }
            else if (boids[0].CanSee(this,collisionAngle,viewRadius * 0.3f)) {
                GL.Uniform3(vertexColorLocation, 1.0f, 0.0f, 0.0f);
            }
            else {
                GL.Uniform3(vertexColorLocation, color[0], color[1], color[2]); //passes color to frag shader
            }

            // GL.Uniform3(vertexColorLocation, color[0], color[1], color[2]); //passes color to frag shader
            GL.BindVertexArray(vertexArrayObject);
            GL.DrawArrays(PrimitiveType.Triangles, 0, 3);
        }

        public void UpdatePosition(float deltaTime, List<Boid> boids) {
            //rotation += 10.0f * deltaTime;
            UpdateRotation(deltaTime,boids);

            position[0] += speed * deltaTime * (float) Math.Cos((double) rotation * Math.PI / 180);
            position[1] += speed * deltaTime * (float) Math.Sin((double) rotation * Math.PI / 180);
            
            if (position[0] > 1.1f) {
                position[0] -= 2.2f;
            }
            if (position[0] < -1.1f) {
                position[0] += 2.2f;
            }
            if (position[1] > 1.1f) {
                position[1] -= 2.2f;
            }
            if (position[1] < -1.1f) {
                position[1] += 2.2f;
            }

            rotation = NormalizeAngle(rotation);

            UpdateVertices();


            //update buffer
            GL.BindBuffer(BufferTarget.ArrayBuffer, vertexBufferObject);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Length * sizeof(float), vertices, BufferUsageHint.DynamicDraw);
        }

        public void UpdateRotation(float deltaTime, List<Boid> boids) {
            int countInView = 0;
            List<float[]> foundPositions = new List<float[]>();
            float alignmentSum = 0.0f;
            float[] cohesionSum = {0.0f,0.0f};
            List<float[]> seperationObjects = new List<float[]>();  // angle, strength
            

            foreach (Boid boid in boids) {
                if (this != boid && CanSee(boid)) {
                    countInView++;
                    foundPositions.Add(new float[] {boid.position[0], boid.position[1]});
                    
                    alignmentSum += boid.rotation;

                    cohesionSum[0] += boid.position[0];
                    cohesionSum[1] += boid.position[1];

                    float distance = (float) Math.Sqrt((boid.position[0] - position[0]) * (boid.position[0] - position[0]) + (boid.position[1] - position[1]) * (boid.position[1] - position[1]));
                    float angle = NormalizeAngle(-180.0f + NormalizeAngle(180.0f * (float) Math.Atan2((boid.position[1] - position[1]), (boid.position[0] - position[0])) / (float) Math.PI));

                    if (CanSee(boid,collisionAngle,viewRadius * 0.5f)) seperationObjects.Add(new float[] {angle, 1 / ((float) Math.Pow(distance * 2.0f,2) + 1) + (speed / (distance * 50.0f))});
                }
            }
            if (countInView == 0) {
                rotation += random.NextFloat(-180.0f,180.0f) * deltaTime;
                return; //alone
            }

            float alignmentMean = alignmentSum / (float) countInView; //blend between meadian and mode depending on the stdev

            float[] cohesionMean = {cohesionSum[0] / (float) countInView, cohesionSum[1] / (float) countInView};
            float cohesionRotation = NormalizeAngle(180.0f * (float) Math.Atan2((cohesionMean[1] - position[1]), (cohesionMean[0] - position[0])) / (float) Math.PI);

            float seperationWeightedMean = 0.0f;
            foreach (float[] pair in seperationObjects) {
                //seperationWeightedMean += pair[0] * pair[1];
                seperationWeightedMean += pair[0];
            }
            float seperationStrength = 0.0f;
            if (seperationObjects.Count() > 0) {
                seperationWeightedMean /= seperationObjects.Count();
                seperationStrength = 1.0f;
            }
            else {
                seperationWeightedMean = rotation;
            }


            float[] variance = {(float) foundPositions.Select(x => Math.Pow(x[0] - cohesionSum[0], 2)).Average(),(float) foundPositions.Select(x => Math.Pow(x[1] - cohesionSum[1], 2)).Average()};
            float alignmentStrength = (float) (Math.Sqrt(variance[0]) * Math.Sqrt(variance[1]));  // stdDev
            // Console.WriteLine(alignmentStrength);
            alignmentStrength = Math.Min(1.0f, 10.0f / alignmentStrength);


            float idealRotation = NormalizeAngle(alignmentMean);
            //float idealRotation = (seperationWeightedMean * seperationStrength) + (alignmentMean * (1.0f - seperationStrength * 0.3f)) + (cohesionRotation * 1.0f);
            // idealRotation = NormalizeAngle(alignmentMean) * alignmentStrength + rotation * (1.0f - alignmentStrength);
            //float idealRotation = NormalizeAngle(seperationWeightedMean);

            float deltaRotation = idealRotation - NormalizeAngle(rotation);
            if (deltaRotation > 180) {
                deltaRotation -= 360;
            }
            rotation += deltaRotation * deltaTime * 2.0f;
        }

        public float NormalizeAngle(float rotation) {
            return ((rotation % 360) + 360) % 360;
        }

        //public bool Ray(float[] position, direction)

        public float[] UpdateVertices() {

            vertices[0] = position[0] + size * (float) Math.Cos((double) rotation * Math.PI / 180);
            vertices[1] = position[1] + size * (float) Math.Sin((double) rotation * Math.PI / 180);

            vertices[3] = position[0] + -1.15f * size * (float) Math.Cos((double) (rotation + 35.0f) * Math.PI / 180);
            vertices[4] = position[1] + -1.15f * size * (float) Math.Sin((double) (rotation + 35.0f) * Math.PI / 180);

            vertices[6] = position[0] + -1.15f * size * (float) Math.Cos((double) (rotation - 35.0f) * Math.PI / 180);
            vertices[7] = position[1] + -1.15f * size * (float) Math.Sin((double) (rotation - 35.0f) * Math.PI / 180);

            return vertices;
        }

        public bool CanSee(Boid boid, float viewAngle = -1.0f, float viewRadius = -1.0f) {
            if (viewAngle == -1.0f) viewAngle = this.viewAngle;
            if (viewRadius == -1.0f) viewRadius = this.viewRadius;

            float distance = (float) Math.Sqrt((boid.position[0] - position[0]) * (boid.position[0] - position[0]) + (boid.position[1] - position[1]) * (boid.position[1] - position[1])); // distance formula
            float[] positionOpposite = new float[] {position[0],position[1]};
            if (position[0] > 0.5f) {
                positionOpposite[0] -= 2.0f;
            }
            else if (position[0] < -0.5f) {
                positionOpposite[0] += 2.0f;
            }
            if (position[1] > 0.5f) {
                positionOpposite[1] -= 2.0f;
            }
            else if (position[1] < -0.5f) {
                positionOpposite[1] += 2.0f;
            }
            float distanceOpposite = (float) Math.Sqrt(Math.Pow((boid.position[0] - positionOpposite[0]),2) + Math.Pow((boid.position[1] - positionOpposite[1]),2));
            if ((distance < (size * 2.0f)) || (distanceOpposite < (size * 2.0f))) return true;
            if (distance > viewRadius && distanceOpposite > viewRadius) return false;

            float angle = NormalizeAngle(180.0f * (float) Math.Atan2((boid.position[1] - position[1]), (boid.position[0] - position[0])) / (float) Math.PI);
            float angleOpposite = NormalizeAngle(180.0f * (float) Math.Atan2((boid.position[1] - positionOpposite[1]), (boid.position[0] - positionOpposite[0])) / (float) Math.PI);

            float startAngle = NormalizeAngle(rotation) - (viewAngle / 2.0f);
            float endAngle = NormalizeAngle(rotation) + (viewAngle / 2.0f);

            if (distance < viewRadius && (((angle > startAngle && angle < endAngle) || (angle - 360 > startAngle && angle - 360 < endAngle)) || (angle + 360 > startAngle && angle + 360 < endAngle))) return true;
            if (distanceOpposite < viewRadius && (((angleOpposite > startAngle && angleOpposite < endAngle) || (angleOpposite - 360 > startAngle && angleOpposite - 360 < endAngle)) || (angleOpposite + 360 > startAngle && angleOpposite + 360 < endAngle))) return true;
            return false;
        }
    }

    public class Game : GameWindow {
        public Game(int width, int height, string title) : base(GameWindowSettings.Default, new NativeWindowSettings() { Size = (width, height), Title = title }) { } //constructor

        List<Boid> boids = new List<Boid>();

        Shader shader;

        Stopwatch stopwatch = new Stopwatch();

        protected override void OnLoad()
        {
            base.OnLoad();

            GL.ClearColor(0.0f, 0.03f, 0.15f, 1.0f);

            boids = Spawner.spawnBoids(Settings.BoidCount);
            

            shader = new Shader("shader.vert", "shader.frag");

            shader.Use();

            stopwatch.Start();
            //Code goes here
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);

            GL.Clear(ClearBufferMask.ColorBufferBit);

            stopwatch.Stop();
            float deltaTime = (float)stopwatch.Elapsed.TotalSeconds;
            stopwatch.Restart();

            foreach (Boid boid in boids) {
                boid.UpdatePosition(deltaTime,boids);
                boid.Draw(shader,boids);
            }

            //Code goes here.

            SwapBuffers();
        }
        protected override void OnResize(ResizeEventArgs e)
        {
            base.OnResize(e);

            GL.Viewport(0, 0, e.Width, e.Height);

            Settings.UpdateWindow(e.Width, e.Height);
        }

        protected override void OnUnload() {
            shader.Dispose();
        }
    }

    public class Shader
    {
        public int Handle;

        public Shader(string vertexPath, string fragmentPath)
        {
            int VertexShader;
            int FragmentShader;

            //load the raw text from the shader files
            string VertexShaderSource = File.ReadAllText(vertexPath);
            string FragmentShaderSource = File.ReadAllText(fragmentPath);

            //generate and bind the source code to the shader
            VertexShader = GL.CreateShader(ShaderType.VertexShader);
            GL.ShaderSource(VertexShader, VertexShaderSource);

            FragmentShader = GL.CreateShader(ShaderType.FragmentShader);
            GL.ShaderSource(FragmentShader, FragmentShaderSource);

            //compile the shader and check for errors
            GL.CompileShader(VertexShader);

            int success;

            GL.GetShader(VertexShader, ShaderParameter.CompileStatus, out success);
            if (success == 0)
            {
                string infoLog = GL.GetShaderInfoLog(VertexShader);
                Console.WriteLine(infoLog);
            }

            GL.CompileShader(FragmentShader);

            GL.GetShader(FragmentShader, ShaderParameter.CompileStatus, out success);
            if (success == 0)
            {
                string infoLog = GL.GetShaderInfoLog(FragmentShader);
                Console.WriteLine(infoLog);
            }

            //linking
            Handle = GL.CreateProgram();

            GL.AttachShader(Handle, VertexShader);
            GL.AttachShader(Handle, FragmentShader);

            GL.LinkProgram(Handle);

            GL.GetProgram(Handle, GetProgramParameterName.LinkStatus, out success);
            if (success == 0)
            {
                string infoLog = GL.GetProgramInfoLog(Handle);
                Console.WriteLine(infoLog);
            }

            //cleanup
            GL.DetachShader(Handle, VertexShader);
            GL.DetachShader(Handle, FragmentShader);
            GL.DeleteShader(FragmentShader);
            GL.DeleteShader(VertexShader);
        }

        public void Use()
        {
            GL.UseProgram(Handle);
        }

        //clean up when deleting
        private bool disposedValue = false;

        protected virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                GL.DeleteProgram(Handle);

                disposedValue = true;
            }
        }

        ~Shader()
        {
            if (disposedValue == false)
            {
                Console.WriteLine("GPU Resource leak! Did you forget to call Dispose()?");
            }
        }


        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

    }

    public class Program {
        static void Main(string[] args) {
            using (Game game = new Game(Settings.WindowSize[0], Settings.WindowSize[1], "Boids")) {
                // float x1 = 0.0f;
                // float y1 = 0.0f;
                // float x2 = 1.0f;
                // float y2 = 1.0f;
                // //float distance = (float) Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)); // distance formula
                // Console.WriteLine(180.0f * (float) Math.Atan2((y2 - y1), (x2 - x1)) / (float) Math.PI);
                game.Run();
            }
        
        }
    }
}

