using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using static UnityEngine.ParticleSystem;

public struct CFluidParticle
{
    public uint id;
    public Vector3 position;
    public Vector3 velocity;
    public float density;
    public Vector3 predictedPosition;
    public int2 spartialPosition;
}

public class OptFluidSim2D : MonoBehaviour
{
    public Mesh particleMesh;
    public Material particleMaterial;
    public Vector2 borderBounds;
    public float gravity;
    public float friction;
    public float particleInstancedPadding;
    public float smoothingLength = 0.2f;
    public float restDensity = 0.5f;
    public float stiffness = 1f;
    public float kinematicViscosity = 0.00001f;
    public float deltaTime = 0.2f;
    public uint noOfParticles = 20;
    public float maxSpeed = 5f;
    public float size = 2f;

    public enum KernelType
    {
        Poly,
        Spiky,
        PolySpiky,
        SpikyPoly
    }

    Dictionary<KernelType, int> kernelMap = new Dictionary<KernelType, int>();

    public KernelType kernelType = KernelType.Poly;

    public ComputeShader fluidSim;

    private ComputeBuffer particleBuffer;
    private GraphicsBuffer particleMatrixBuffer;
    
    int spartialUpdateKernel;
    int densityUpdateKernel;
    int velocityUpdateKernel;
    int predictionUpdateKernel;
    int mainKernel;

    //readonly float mass = 1f;
    
    CFluidParticle[] particles;
    private Matrix4x4[] particleMatrices;

    const int threadSize = 64; //change in compute as well
    // Start is called before the first frame update
    void Start()
    {

        particles = new CFluidParticle[noOfParticles];
        particleMatrices = new Matrix4x4[noOfParticles];

        kernelMap = new Dictionary<KernelType, int>
        {
            { KernelType.Poly, 0 },
            { KernelType.Spiky, 1 },
            { KernelType.PolySpiky, 2 },
            { KernelType.SpikyPoly, 3 }
        };

        float padding = size * 2 + particleInstancedPadding;

        int particlesPerAxis = Mathf.CeilToInt(Mathf.Sqrt(noOfParticles)); // Use square root for 2D arrangement

        int index = 0;
        for (int x = 0; x < particlesPerAxis; x++) // Loop through x-axis
        {
            for (int y = 0; y < particlesPerAxis && index < noOfParticles; y++) // Loop through y-axis
            {
                particles[index] = new CFluidParticle
                {
                    id = (uint)index,
                    position = new Vector3(
                        (x - particlesPerAxis / 2f + size) * padding, // Calculate x position
                        (y - particlesPerAxis / 2f + size) * padding, // Calculate y position
                        0 // z is 0 for 2D
                    ),
                    velocity = Vector3.zero,
                    predictedPosition = Vector3.zero
                };

                particleMatrices[index] = Matrix4x4.TRS(particles[index].position, Quaternion.identity, Vector3.one * size);

                index++; // Move to the next particle
            }
        }

        InitializeBuffers();

        //Find Kernels
        densityUpdateKernel = fluidSim.FindKernel("UpdateDensities");
        velocityUpdateKernel = fluidSim.FindKernel("CalculateVelocities");
        predictionUpdateKernel = fluidSim.FindKernel("SetPredictedPosition");
        mainKernel = fluidSim.FindKernel("CSMain");

        SetBuffers();
    }

    void InitializeBuffers()
    {
        int particleStride = System.Runtime.InteropServices.Marshal.SizeOf<CFluidParticle>();
        int matrixStride = System.Runtime.InteropServices.Marshal.SizeOf<Matrix4x4>();

        particleBuffer = new ComputeBuffer(particles.Length, particleStride);
        particleMatrixBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, particles.Length, matrixStride);
        
        
        particleBuffer.SetData(particles);
        particleMatrixBuffer.SetData(particleMatrices);
        
        
    }
    
    void SetBuffers()
    {
        fluidSim.SetBuffer(mainKernel, "particles", particleBuffer);
        fluidSim.SetBuffer(mainKernel, "particleMatrices", particleMatrixBuffer);

        fluidSim.SetBuffer(densityUpdateKernel, "particles", particleBuffer);

        fluidSim.SetBuffer(velocityUpdateKernel, "particles", particleBuffer);
        
        fluidSim.SetBuffer(predictionUpdateKernel, "particles", particleBuffer);

    }

    void PassToShader()
    {
        fluidSim.SetVector("borderBounds", borderBounds);
        fluidSim.SetFloat("deltaTime", deltaTime);
        fluidSim.SetFloat("size", size);
        fluidSim.SetFloat("gravity", gravity);
        fluidSim.SetInt("particlesCount", particles.Length);
        fluidSim.SetFloat("friction", friction);
        fluidSim.SetFloat("smoothingLength", smoothingLength);
        fluidSim.SetFloat("restDensity", restDensity);
        fluidSim.SetFloat("stiffness", stiffness);
        fluidSim.SetFloat("kinematicViscosity", kinematicViscosity);
        fluidSim.SetFloat("maxSpeed", maxSpeed);
        fluidSim.SetInt("kernelType", kernelMap[kernelType]);
    }

    void ThreadDispatch()
    {
        int threadGroups = Mathf.CeilToInt(particles.Length / (float)threadSize); // Based on [threadSize, 1, 1] thread group size

        fluidSim.Dispatch(predictionUpdateKernel, threadGroups, 1, 1);
        fluidSim.Dispatch(densityUpdateKernel, threadGroups, 1, 1);
        fluidSim.Dispatch(velocityUpdateKernel, threadGroups, 1, 1);
        fluidSim.Dispatch(mainKernel, threadGroups, 1, 1);
    }
    //a
    // Update is called once per frame
    void Update()
    {
        PassToShader();
        ThreadDispatch();

        particleMatrixBuffer.GetData(particleMatrices);
        Graphics.DrawMeshInstanced(particleMesh, 0, particleMaterial, particleMatrices);
    }

    private void OnDestroy()
    {
        particleBuffer?.Release();
        particleMatrixBuffer?.Release();
        particleBuffer = null;
        particleMatrixBuffer = null;
    }

    private void OnDrawGizmos()
    {
        // Set the color for the border
        Gizmos.color = Color.green;

        // Draw a wireframe rectangle (2D equivalent of a square) centered at the origin
        Gizmos.DrawWireCube(Vector3.zero, borderBounds);
    }
}
