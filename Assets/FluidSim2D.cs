using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Unity.Mathematics;
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

    public KernelType kernelType = KernelType.Poly;

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

        float padding = size * 2 + particleInstancedPadding;

        int particlesPerAxis = Mathf.CeilToInt(Mathf.Sqrt(noOfParticles)); // Use square root for 2D arrangement

        int index = 0;
        for (int x = 0; x < particlesPerAxis; x++) // Loop through x-axis
        {
            for (int y = 0; y < particlesPerAxis && index < noOfParticles; y++) // Loop through y-axis
            {
                particles[index] = new FluidParticle
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
                index++; // Move to the next particle
            }
        }

        AddFluidParticlesToSpartialGrid();
    }
    Vector3 ValidateVector(Vector3 v)
    {
        if (float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) ||
            float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z))
        {
            return Vector3.zero;
        }
        return v;
    }
    void AddFluidParticlesToSpartialGrid()
    {
        //spartialGrid.Clear();
        foreach (var particle in particles)
        {
            spartialGrid.AddFluidParticleToCell(particle);
        }
    }

    float PolyKernel(float l, float r)
    {
        if (r < 0 || r > 1) return 0;

        float alpha = 4 / (Mathf.PI * Mathf.Pow(l, 8));
        float volume = Mathf.Pow((l * l - r * r), 3);
        return alpha * volume;
    }

    float PolyKernelDerivative(float l, float r)
    {
        if (r < 0 || r > 1) return 0;

        float alpha = 4 / (Mathf.PI * Mathf.Pow(l, 8));
        float volume = 3 * Mathf.Pow((l * l - r * r), 2);
        return -alpha * volume;
    }

    float SpikyKernel(float l, float r)
    {
        if (r < 0 || r > 1) return 0;

        float alpha = 10 / (Mathf.PI * Mathf.Pow(l,5));
        float volume = Mathf.Pow((l - r), 3);
        return alpha * volume;
    }

    float SpikyKernelDerivative(float l, float r)
    {
        if (r < 0 || r > 1) return 0;

        float alpha = 10 / (Mathf.PI * Mathf.Pow(l, 5));
        float volume = 3 * Mathf.Pow((l - r), 2);
        return -alpha * volume;
    }

    float Viscositykernel(float l, float r)
    {
        if (r >= l) return 0f;

        float alpha = 45f / (Mathf.PI * Mathf.Pow(l, 6));
        float volume = l - r;

        return alpha * volume;
    }

    Vector3 CalculatePressureForce(FluidParticle particle)
    {
        Vector3 pressureForce = Vector3.zero;
        float particlePressure = ConvertDensityToPressure(particle.density) / Mathf.Pow(particle.density, 2);
        foreach (var neighbor in particle.neighbors)
        {
            //if (neighbor.id == particle.id) continue;
            Vector3 offset = neighbor.predictedPosition - particle.predictedPosition;
            float dst = offset.magnitude;
            float slope = 0;
            if (kernelType == KernelType.Poly)
            {
                slope = PolyKernelDerivative(smoothingLength, dst);
            }
            else if (kernelType == KernelType.Spiky)
            {
                slope = SpikyKernelDerivative(smoothingLength, dst);
            }
            else if (kernelType == KernelType.PolySpiky)
            {
                slope = SpikyKernelDerivative(smoothingLength, dst);
            }
            else if (kernelType == KernelType.SpikyPoly)
            {
                slope = PolyKernelDerivative(smoothingLength, dst);
            }
            Vector3 dir = offset.normalized;
            float neighborPressure = Mathf.Max(ConvertDensityToPressure(neighbor.density) / Mathf.Pow(neighbor.density, 2), 0);
            float pressure = particlePressure + neighborPressure;
            Vector3 result = pressure * dir * slope * mass;
            pressureForce += ValidateVector(result);
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

            Vector3 gradient = r * Viscositykernel(smoothingLength, r_mag);
            float arOd = Vector3.Dot(r, r_vel) / r2;
            float mOd = mass / neighbor.density;

            Vector3 result = mOd * arOd * gradient;
            laplacianVelocity += ValidateVector(result);
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
            float influence = 0;
            if (kernelType == KernelType.Poly)
            {
                influence = PolyKernel(smoothingLength, dst);
            }
            else if (kernelType == KernelType.Spiky)
            {
                influence = SpikyKernel(smoothingLength, dst);
            }
            else if (kernelType == KernelType.PolySpiky)
            {
                influence = PolyKernel(smoothingLength, dst);
            }
            else if (kernelType == KernelType.SpikyPoly)
            {
                influence = SpikyKernel(smoothingLength, dst);
            }
            density += mass * influence;
        }
        return density;
    }

    float ConvertDensityToPressure(float density)
    {
        float densityError = density - restDensity;
        float pressure = densityError * stiffness;
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
        var neighborsId = spartialGrid.GetNeighbors(particle.predictedPosition, smoothingLength);

        foreach (var neighborId in neighborsId)
        {
            var neighbor = particles[(int)neighborId];
            Vector3 direction = neighbor.predictedPosition - particle.predictedPosition;
            float distance = direction.magnitude;

            if (neighbor.id != particle.id)
            {
                if (distance < smoothingLength)
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
            particle.velocity.x *= -1 * friction;
        }
        if (Mathf.Abs(particle.position.y) > halfBounds.y)
        {
            particle.position.y = halfBounds.y * Mathf.Sign(particle.position.y);
            particle.velocity.y *= -1 * friction;
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
        //int startIdx = currentSubset * subsetSize;
        //int endIdx = Mathf.Min(startIdx + subsetSize, particles.Length);

        int startIdx = 0;
        int endIdx = particles.Length;

        Parallel.For(startIdx, endIdx, (i) =>
        {
            var particle = particles[i];
            Vector3 accel = particle.velocity * deltaTime;
            particle.predictedPosition = particle.position + ValidateVector(accel);
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
            Vector3 gravityForce = Vector3.down * gravity * mass;
            Vector3 totalForce = pressureForce + viscosityForce + gravityForce;
            Vector3 acceleration = totalForce / particle.density;
            particle.velocity += acceleration * deltaTime;
        });

        Parallel.For(startIdx, endIdx, (i) =>
        {
            var particle = particles[i];
            particle.velocity = ClampVelocity(particle.velocity, maxSpeed);
            particle.velocity = ValidateVector(particle.velocity);
            Vector3 accel = particle.velocity * deltaTime;
            particle.position += ValidateVector(accel);

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
