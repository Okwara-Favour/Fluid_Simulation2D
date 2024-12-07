using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using static UnityEngine.ParticleSystem;

public struct SpartialData
{
    public int cell;
    public uint id;
};
public struct CFluidParticle
{
    public uint id;
    public Vector3 position;
    public Vector3 velocity;
    public float density;
    public Vector3 predictedPosition;
    public Vector2 spartialPosition;
}

public class OptFluidSim2D : MonoBehaviour
{
    public Mesh particleMesh;
    public Material particleMaterial;
    public Vector2 borderBounds;
    public float gravity;
    public float collisionDamp;
    public float particleSpacing;
    public float smoothingRadius = 0.2f;
    public float targetDensity = 0.5f;
    public float pressureMultiplier = 1f;
    public float kinematicViscosity = 0.00001f;
    public float timestep = 0.2f;
    public uint noOfParticles = 20;
    public float maxSpeed = 5f;
    public float size = 2f;
    
    Vector2 gridSize;
    Vector2 currentCellSize;
    Vector2 gridPosition;

    public ComputeShader fluidSim;

    private ComputeBuffer particleBuffer;
    private GraphicsBuffer particleMatrixBuffer;
    private ComputeBuffer spartialBuffer;
    
    int spartialUpdateKernel;
    int densityUpdateKernel;
    int velocityUpdateKernel;
    int gravityUpdateKernel;
    int mainKernel;

    //readonly float mass = 1f;
    
    CFluidParticle[] particles;
    private Matrix4x4[] particleMatrices;
    private SpartialData[] spartialDatas;
    private Vector3[] velocities;
    private float[] densities;

    private SpartialGrid spartialGrid;

    const int threadSize = 64; //change in compute as well
    // Start is called before the first frame update
    void Start()
    {
        spartialGrid = GetComponent<SpartialGrid>();

        particles = new CFluidParticle[noOfParticles];
        particleMatrices = new Matrix4x4[noOfParticles];
        spartialDatas = new SpartialData[noOfParticles];
        velocities = new Vector3[noOfParticles];
        densities = new float[noOfParticles];

        gridSize = new Vector2(spartialGrid.gridWidth, spartialGrid.gridHeight);
        gridPosition = new Vector2(spartialGrid.position.x, spartialGrid.position.y);
        currentCellSize = spartialGrid.cellSize;

        int particlesPerRow = (int)Mathf.Sqrt(noOfParticles);
        int particlesPerCol = ((int)noOfParticles - 1) / particlesPerRow + 1;
        float spacing = size * 2 + particleSpacing;

        for (int i = 0; i < noOfParticles; i++)
        {
            float x = (i % particlesPerRow - particlesPerRow / 2f + 0.5f) * spacing;
            float y = (i / particlesPerRow - particlesPerCol / 2f + 0.5f) * spacing;
            particles[i] = new CFluidParticle
            {
                id = (uint)i,
                position = new Vector3(x, y, 0),
                velocity = Vector3.zero,
                predictedPosition = Vector3.zero
            };

            velocities[i] = Vector3.zero;
            densities[i] = 0;
            spartialDatas[i] = new SpartialData
            {
                cell = 0,
                id = (uint)i
            };
            particleMatrices[i] = Matrix4x4.TRS(particles[i].position, Quaternion.identity, Vector3.one * size);
        }
        
        InitializeBuffers();

        spartialUpdateKernel = fluidSim.FindKernel("PopulateSpartialArray");
        densityUpdateKernel = fluidSim.FindKernel("UpdateDensities");
        velocityUpdateKernel = fluidSim.FindKernel("CalculateVelocities");
        gravityUpdateKernel = fluidSim.FindKernel("UpdateGravity");
        mainKernel = fluidSim.FindKernel("CSMain");

        fluidSim.SetBuffer(mainKernel, "particles", particleBuffer);
        fluidSim.SetBuffer(mainKernel, "particleMatrices", particleMatrixBuffer);
        

        fluidSim.SetBuffer(spartialUpdateKernel, "spartialCell", spartialBuffer);
        fluidSim.SetBuffer(spartialUpdateKernel, "particles", particleBuffer);

        fluidSim.SetBuffer(densityUpdateKernel, "particles", particleBuffer);
        fluidSim.SetBuffer(densityUpdateKernel, "spartialCell", spartialBuffer);

        fluidSim.SetBuffer(velocityUpdateKernel, "particles", particleBuffer);
        fluidSim.SetBuffer(velocityUpdateKernel, "spartialCell", spartialBuffer);

        fluidSim.SetBuffer(gravityUpdateKernel, "particles", particleBuffer);

    }

    void InitializeBuffers()
    {
        int particleStride = System.Runtime.InteropServices.Marshal.SizeOf<CFluidParticle>();
        int matrixStride = System.Runtime.InteropServices.Marshal.SizeOf<Matrix4x4>();
        int spartialStride = System.Runtime.InteropServices.Marshal.SizeOf<SpartialData>();
        
        particleBuffer = new ComputeBuffer(particles.Length, particleStride);
        particleMatrixBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, particles.Length, matrixStride);
        spartialBuffer = new ComputeBuffer(spartialDatas.Length, spartialStride);
        
        particleBuffer.SetData(particles);
        particleMatrixBuffer.SetData(particleMatrices);
        spartialBuffer.SetData(spartialDatas);
        
    }

    void PassToShader()
    {
        fluidSim.SetVector("borderBounds", borderBounds);
        fluidSim.SetFloat("timestep", timestep);
        fluidSim.SetFloat("size", size);
        fluidSim.SetFloat("gravity", gravity);
        fluidSim.SetInt("particlesCount", particles.Length);
        fluidSim.SetVector("gridPosition", gridPosition);
        fluidSim.SetVector("gridSize", gridSize);
        fluidSim.SetVector("currentCellSize", currentCellSize);
        fluidSim.SetFloat("collisionDamp", collisionDamp);
        fluidSim.SetFloat("smoothingRadius", smoothingRadius);
        fluidSim.SetFloat("targetDensity", targetDensity);
        fluidSim.SetFloat("pressureMultiplier", pressureMultiplier);
        fluidSim.SetFloat("kinematicViscosity", kinematicViscosity);
        fluidSim.SetFloat("maxSpeed", maxSpeed);
    }

    void ThreadDispatch()
    {
        

        int threadGroups = Mathf.CeilToInt(particles.Length / (float)threadSize); // Based on [threadSize, 1, 1] thread group size

        
        fluidSim.Dispatch(spartialUpdateKernel, threadGroups, 1, 1);
        //spartialBuffer.GetData(spartialDatas);
        //Sort();
        //spartialBuffer.SetData(spartialDatas);

        fluidSim.Dispatch(gravityUpdateKernel, threadGroups, 1, 1);
        fluidSim.Dispatch(densityUpdateKernel, threadGroups, 1, 1);
        fluidSim.Dispatch(velocityUpdateKernel, threadGroups, 1, 1);
        fluidSim.Dispatch(mainKernel, threadGroups, 1, 1);
    }

    public void Sort()
    {
        //Will change when I get smarter
        Array.Sort(spartialDatas, (x, y) => x.cell.CompareTo(y.cell));
    }
    //a
    // Update is called once per frame
    void Update()
    {
        PassToShader();
        ThreadDispatch();

        particleMatrixBuffer.GetData(particleMatrices);
        /*particleBuffer.GetData(particles);
        
        string res = "";
        for (int i =0; i < spartialDatas.Length; i++)
        {
            var sp = particles[i];
            res += sp.velocity + " ";
            //Debug.Log(sp.velocity * 1);
        }
        //Debug.Log(res);
        */
        Graphics.DrawMeshInstanced(particleMesh, 0, particleMaterial, particleMatrices);
    }

    private void OnDestroy()
    {
        particleBuffer?.Release();
        particleMatrixBuffer?.Release();
       
        spartialBuffer?.Release();

        particleBuffer = null;
        particleMatrixBuffer = null;
        
        spartialBuffer = null;
    }

    private void OnDrawGizmos()
    {
        // Set the color for the border
        Gizmos.color = Color.green;

        // Draw a wireframe rectangle (2D equivalent of a square) centered at the origin
        Gizmos.DrawWireCube(Vector3.zero, borderBounds);
    }
}
