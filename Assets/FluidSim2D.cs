using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;



public class FluidParticle
{
    public uint id = 0;
    public Vector3 position = Vector3.zero;
    public Vector3 velocity = Vector3.zero;
    public Vector3 predictedPosition = Vector3.zero;
    public float density = 0f;
    public List<FluidParticle> neighbors = new();
    public Vector2Int spartialPosition = Vector2Int.zero;
}

public class FluidSim2D : MonoBehaviour
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
    public float deltaTime = 0.2f;
    public uint noOfParticles = 20;
    public float maxSpeed = 5f;
    public float size = 2f;

    readonly float mass = 1f;

    int subsetSize;
    int currentSubset = 0;
    int particlesSplit = 4;

    Vector2 meshSize;
    FluidParticle[] particles;
    private Matrix4x4[] particleMatrices;
    private SpartialGrid spartialGrid;

    private readonly object cellLock = new object();
    private static readonly System.Random random = new System.Random();
    // Start is called before the first frame update
    void Start()
    {
        particles = new FluidParticle[noOfParticles];
        particleMatrices = new Matrix4x4[noOfParticles];
        meshSize = particleMesh.bounds.size;

        spartialGrid = GetComponent<SpartialGrid>();
        subsetSize = Mathf.CeilToInt((float)particles.Length / (float)particlesSplit);

        int particlesPerRow = (int)Mathf.Sqrt(noOfParticles);
        int particlesPerCol = ((int)noOfParticles - 1) / particlesPerRow + 1;
        float spacing = size * 2 + particleSpacing;

        for(int i = 0; i < noOfParticles; i++)
        {
            float x = (i % particlesPerRow - particlesPerRow / 2f + 0.5f) * spacing;
            float y = (i / particlesPerRow - particlesPerCol / 2f + 0.5f) * spacing;
            particles[i] = new FluidParticle
            {
                id = (uint)i,
                position = new Vector3(x, y, 0),
                velocity = Vector3.zero
            };
        }

        AddFluidParticlesToSpartialGrid();
    }
    /*
    public void Sort()
    {
        //Will change when I get smarter
        Array.Sort(spartialDatas, (x, y) => x.cell.CompareTo(y.cell));
    }*/
    void AddFluidParticlesToSpartialGrid()
    {
        //spartialGrid.Clear();
        foreach (var particle in particles)
        {
            spartialGrid.AddFluidParticleToCell(particle);
        }
    }

    float SmoothingKernel(float radius, float dst)
    {
        float alpha = 10 / (Mathf.PI * Mathf.Pow(radius,5));
        float volume = Mathf.Pow((radius - dst), 3);
        return alpha * volume;
    }

    float SmoothingKernelDerivative(float radius, float dst)
    {
        float alpha = 10 / (Mathf.PI * Mathf.Pow(radius, 5));
        float volume = 3 * Mathf.Pow((radius - dst), 2);
        return -alpha * volume;
    }

    float Viscositykernel(float radius, float dst)
    {
        if (dst >= radius) return 0f;

        float alpha = 45f / (Mathf.PI * Mathf.Pow(radius, 6));
        float volume = radius - dst;

        return alpha * volume;
    }

    Vector3 CalculatePressureForce(FluidParticle particle)
    {
        Vector3 pressureForce = Vector3.zero;
        float particlePressure = ConvertDensityToPressure(particle.density) / Mathf.Pow(particle.density, 2);
        foreach (var neighbor in particle.neighbors)
        {
            if (neighbor.id == particle.id) continue;
            Vector3 offset = neighbor.predictedPosition - particle.predictedPosition;
            float dst = offset.magnitude;
            float slope = SmoothingKernelDerivative(smoothingRadius, dst);
            Vector3 dir = offset.normalized;
            float neighborPressure = ConvertDensityToPressure(neighbor.density) / Mathf.Pow(neighbor.density, 2);
            float sharedPressure = particlePressure + neighborPressure;
            pressureForce += sharedPressure * dir * slope * mass;
        }
        return pressureForce;
    }

    Vector3 CalculateViscosityForce2(FluidParticle particle)
    {
        Vector3 laplacianVelocity = Vector3.zero;

        foreach (var neighbor in particle.neighbors)
        {
            Vector3 r = neighbor.predictedPosition - particle.predictedPosition;
            Vector3 r_vel = neighbor.velocity - particle.velocity;
            float r_mag = r.magnitude;
            float r2 = Mathf.Pow((r_mag), 2);

            Vector3 gradient = r * Viscositykernel(smoothingRadius, r_mag);
            float arOd = Vector3.Dot(r, r_vel) / r2;
            float mOd = mass / neighbor.density;

            laplacianVelocity += mOd * arOd * gradient;
        }

        return kinematicViscosity * laplacianVelocity;
    }

    float CalculateDensity(FluidParticle particle)
    {
        float density = 0;
        
        foreach(var neighbor in particle.neighbors)
        {
            //if (neighbor.id == particle.id) continue;

            float dst = (neighbor.predictedPosition - particle.predictedPosition).magnitude;
            float influence = SmoothingKernel(smoothingRadius, dst);
            density += mass * influence;
        }
        return density;
    }

    float ConvertDensityToPressure(float density)
    {
        float densityError = density - targetDensity;
        float pressure = densityError * pressureMultiplier;
        return pressure;
    }

    public static Vector3 GetRandomDirection()
    {
        double theta = random.NextDouble() * 2.0 * Mathf.PI; // Angle around the Y axis
        double phi = Mathf.Acos(2.0f * (float)random.NextDouble() - 1.0f); // Angle from the Y axis

        // Convert spherical coordinates to Cartesian coordinates
        float x = (float)(Mathf.Sin((float)phi) * Mathf.Cos((float)theta));
        float y = (float)(Mathf.Sin((float)phi) * Mathf.Sin((float)theta));
        float z = (float)Mathf.Cos((float)phi);

        return new Vector3(x, y, z);
    }

    void FindNeighbors(FluidParticle particle)
    {
        particle.neighbors.Clear();
        var neighborsId = spartialGrid.GetNeighbors(particle.position, smoothingRadius);

        foreach (var neighborId in neighborsId)
        {
            var neighbor = particles[(int)neighborId];
            Vector3 direction = neighbor.predictedPosition - particle.predictedPosition;
            float distance = direction.magnitude;

            if (neighbor.id != particle.id)
            {
                if (distance < smoothingRadius)
                {
                    particle.neighbors.Add(neighbor);
                }
            }
        }
        //Debug.Log(particle.neighbors.Count);
    }

    void UpdateSpartialPosition(FluidParticle particle)
    {
        
        var gridPos = spartialGrid.WorldToGridPos(particle.position);
        if (gridPos != particle.spartialPosition)
        {
            var currentCell = spartialGrid.GetCell(particle.spartialPosition.x, particle.spartialPosition.y);
            if (currentCell != null)
            {
                lock (cellLock)
                {
                    currentCell.particleIDs.Remove(particle.id);
                    spartialGrid.AddFluidParticleToCell(particle);
                }
            }
        }
        
    }

    void KeepInBounds(FluidParticle particle)
    {

        Vector2 halfBounds = borderBounds / 2 - Vector2.one * size / 2f;
        if(Mathf.Abs(particle.position.x) > halfBounds.x)
        {
            particle.position.x = halfBounds.x * Mathf.Sign(particle.position.x);
            particle.velocity.x *= -1 * collisionDamp;
        }
        if (Mathf.Abs(particle.position.y) > halfBounds.y)
        {
            particle.position.y = halfBounds.y * Mathf.Sign(particle.position.y);
            particle.velocity.y *= -1 * collisionDamp;
        }
    }

    Vector3 ClampVelocity(Vector3 velocity, float maxSpeed)
    {
        float speed = velocity.magnitude;

        // Clamp the magnitude of the velocity vector between 0 and maxSpeed
        if (speed > maxSpeed)
        {
            velocity = velocity.normalized * maxSpeed;
        }
        else if (speed < -maxSpeed)
        {
            velocity = velocity.normalized * -maxSpeed;
        }

        return velocity;
    }
    // Update is called once per frame
    void Update()
    {
        int startIdx = currentSubset * subsetSize;
        int endIdx = Mathf.Min(startIdx + subsetSize, particles.Length);

        Parallel.For(startIdx, endIdx, (i) =>
        {
            var particle = particles[i];
            particle.velocity += Vector3.down * gravity * deltaTime;
            particle.predictedPosition = particle.position + particle.velocity * deltaTime;
        });

        Parallel.For(startIdx, endIdx, (i) =>
        {
            var particle = particles[i];
            FindNeighbors(particle);
        });

        Parallel.For(startIdx, endIdx, (i) =>
        {
            var particle = particles[i];
            particle.density = CalculateDensity(particle);
        });

        Parallel.For(startIdx, endIdx, (i) =>
        {
            var particle = particles[i];
            Vector3 pressureForce = CalculatePressureForce(particle);
            Vector3 viscosityForce = CalculateViscosityForce2(particle);
            Vector3 totalForce = pressureForce + viscosityForce;
            Vector3 acceleration = totalForce / particle.density;
            particle.velocity += acceleration * deltaTime;
        });

        Parallel.For(startIdx, endIdx, (i) =>
        {
            var particle = particles[i];
            particle.velocity = (float.IsNaN(particle.velocity.x) || float.IsNaN(particle.velocity.y)) ?
                                    Vector3.zero : ClampVelocity(particle.velocity, maxSpeed);
            particle.position += particle.velocity * deltaTime;

            KeepInBounds(particle);

            UpdateSpartialPosition(particle);

            //Debug.Log(particle.position + " " + particle.velocity);
            particleMatrices[particle.id] = Matrix4x4.TRS(particle.position, Quaternion.identity, Vector3.one * size);
            
        });
        Graphics.DrawMeshInstanced(particleMesh, 0, particleMaterial, particleMatrices);
        //Graphics.DrawMesh(particleMesh, matrix, particleMaterial, 0);

        currentSubset = (currentSubset + 1) % particlesSplit;
    }

    private void OnDrawGizmos()
    {
        // Set the color for the border
        Gizmos.color = Color.green;

        // Draw a wireframe rectangle (2D equivalent of a square) centered at the origin
        Gizmos.DrawWireCube(Vector3.zero, borderBounds);
    }
}
