using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct SParticle
{
    public uint id;
    public Vector3 position;
    public Vector3 velocity;
}
public class SimpleParticle : MonoBehaviour
{
    public Mesh particleMesh;
    public Material particleMaterial;
    public Vector2 borderBounds;
    public ComputeShader simpleCompute;

    Vector2 meshSize;
    SParticle[] particles;

    
    private Matrix4x4[] particleMatrices;

    public uint noOfParticles = 20;
    public float size = 2f;

    public float particleSpacing = -0.1f;
    public float timestep = 0.01f;
    private ComputeBuffer particleBuffer;
    private GraphicsBuffer particleMatrixBuffer;

    
    private int kernelHandle;
    const int threadSize = 64; //change in compute as well

    //private ComputeBuffer compborderBounds;
    // Start is called before the first frame update
    void Start()
    {
        
        particles = new SParticle[noOfParticles];
        particleMatrices = new Matrix4x4[noOfParticles];
        meshSize = particleMesh.bounds.size;
        
        int particlesPerRow = (int)Mathf.Sqrt(noOfParticles);
        int particlesPerCol = ((int)noOfParticles - 1) / particlesPerRow + 1;
        float spacing = size * 2 + particleSpacing;

        for (int i = 0; i < noOfParticles; i++)
        {
            float x = (i % particlesPerRow - particlesPerRow / 2f + 0.5f) * spacing;
            float y = (i / particlesPerRow - particlesPerCol / 2f + 0.5f) * spacing;
            particles[i] = new SParticle
            {
                id = (uint)i,
                position = new Vector3(x, y, 0),
                velocity = Vector3.one * 0.5f
            };
        }

        foreach (var particle in particles)
        {
            particleMatrices[particle.id] = Matrix4x4.TRS(particle.position, Quaternion.identity, Vector3.one * size);
        }

        particleBuffer = new ComputeBuffer(particles.Length, sizeof(uint) + sizeof(float) * 6);
        particleMatrixBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, particles.Length, sizeof(float) * 4 * 4);

        particleBuffer.SetData(particles);
        particleMatrixBuffer.SetData(particleMatrices);
        
        // Get the kernel handle
        kernelHandle = simpleCompute.FindKernel("CSMain");

        simpleCompute.SetBuffer(kernelHandle, "particles", particleBuffer);
        simpleCompute.SetBuffer(kernelHandle, "particleMatrices", particleMatrixBuffer);

    }

    void PassToShader()
    {
        simpleCompute.SetVector("borderBounds", borderBounds);
        simpleCompute.SetFloat("timestep", timestep);
        simpleCompute.SetFloat("size", size);
    }
    // Update is called once per frame
    void Update()
    {

        PassToShader();

        int threadGroups = Mathf.CeilToInt(particles.Length / (float)threadSize); // Based on [8, 1, 1] thread group size
        simpleCompute.Dispatch(kernelHandle, threadGroups, 1, 1);

        
        particleMatrixBuffer.GetData(particleMatrices);
        Graphics.DrawMeshInstanced(particleMesh, 0, particleMaterial, particleMatrices);

    }

    private void OnDestroy()
    {
        // Release the compute buffer when no longer needed
        if (particleBuffer != null)
        {
            particleBuffer.Release();
            particleMatrixBuffer?.Release();
            
        }
    }

    private void OnDrawGizmos()
    {
        // Set the color for the border
        Gizmos.color = Color.green;

        // Draw a wireframe rectangle (2D equivalent of a square) centered at the origin
        Gizmos.DrawWireCube(Vector3.zero, borderBounds);
    }
}

