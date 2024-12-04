using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using static UnityEngine.ParticleSystem;
using static UnityEngine.RuleTile.TilingRuleOutput;
using UnityEngine.WSA;

class Particle
{
    public Vector3 pos = Vector3.zero;
    public Vector3 scale = Vector3.one;
    public Vector3 velocity = Vector3.zero;
    public Vector3 predictedPosition = Vector3.zero;
    public Vector3 predictedVelocity = Vector3.zero;
    public float volume = 1.0f;
    public float mass = 1.0f;
    public float density = 0.0f;
    public float pressure = 0.0f;
    public Vector3 Fpressure = Vector3.zero; 
    public Vector3 Force = Vector3.zero;
    public List<Particle> neighbors = new List<Particle>();
}

public class FluidSimulation2D : MonoBehaviour
{
    public int particleCount = 10000;
    public Vector3 particleScale = Vector3.zero;
    public float gravity = 0.2f;
    public float surfaceFriction = 0.2f;
    public float Volume = 0.5f;
    public float Density = 0.2f;
    public float stiffness = 0.2f;
    public int stiffExp = 2;
    public float smoothing_length = 0.1f;
    public float kinematic_viscosity = 0.001f;

    public float timestep = 0.01f;
    public float boundaryDamp = 0.2f;

    public float widthConstrict = 0.0f;


    public Mesh particleMesh;
    public Material particleMaterial;

    private Particle[] particles;
    private Matrix4x4[] particleMatrices;
    

    private Vector3 minCamBounds = Vector3.zero;
    private Vector3 maxCamBounds = Vector3.zero;

    void Start()
    {
        Camera mainCamera = Camera.main;
        float screenAspect = mainCamera.aspect;
        float mainCamHeight = mainCamera.orthographicSize * 2;
        float mainCamWidth = mainCamHeight * screenAspect;
        Vector3 mCamPos = mainCamera.transform.position;
        minCamBounds = new Vector3(mCamPos.x - mainCamWidth / 2, mCamPos.y - mainCamHeight / 2);
        maxCamBounds = new Vector3(mCamPos.x + mainCamWidth / 2, mCamPos.y + mainCamHeight / 2);


        particles = new Particle[particleCount];
        particleMatrices = new Matrix4x4[particleCount];

        for (int i = 0; i < particleCount; i++)
        {
            float x = Random.Range(-4.0f, 4.0f);
            float y = Random.Range(-4.0f, 4.0f);
            particles[i] = new Particle
            {
                pos = new Vector3(x, y, 0),
                scale = particleScale,
                volume = Volume / particleCount,
                mass = Density * (Volume / particleCount)
            };
        }
    }

    void FindNeighbors(int index)
    {
        particles[index].neighbors.Clear();

        for (int i = 0; i < particles.Length; i++)
        {
            if (index != i)
            {
                Vector3 direction = particles[index].pos - particles[i].pos;
                float distance = direction.magnitude;
                if (distance > 0 && distance < smoothing_length)
                {
                    particles[index].neighbors.Add(particles[i]);
                }
            }
        }
    }

    float ComputeDensity(Particle particle)
    {
        float totalDensity = 0.0f;
        foreach (var neighbor in particle.neighbors) 
        {
            totalDensity += particle.mass * KernelFunction((particle.pos -neighbor.pos), smoothing_length).Item1;
        }
        return totalDensity;
    }

    float PredictedDensity(Particle particle)
    {
        float totalDensity = 0.0f;
        float totalPrediction = 0.0f;

        foreach (var neighbor in particle.neighbors)
        {
            // Get kernel values
            var (w, gradient) = KernelFunction(particle.pos - neighbor.pos, smoothing_length);
            totalDensity += neighbor.mass * w;
            Vector3 velocityDifference = particle.predictedVelocity - neighbor.predictedVelocity;
            totalPrediction += neighbor.mass * Vector3.Dot(velocityDifference, gradient);
        }

        // Predicted density is sum of density + timestep-adjusted prediction term
        return totalDensity + timestep * totalPrediction;
    }

    float PredictedDensityPCISPH(Particle particle)
    {
        float totalDensity = 0.0f;
        float totalPrediction = 0.0f;

        foreach (var neighbor in particle.neighbors)
        {
            // Get kernel values
            var (w, gradient) = KernelFunction(particle.predictedPosition - neighbor.predictedPosition, smoothing_length);

            // Density term based on mass and kernel value
            totalDensity += neighbor.mass * w;

            // Prediction term based on velocity difference and kernel gradient
            Vector3 velocityDifference = particle.predictedVelocity - neighbor.predictedVelocity;
            totalPrediction += neighbor.mass * Vector3.Dot(velocityDifference, gradient);
        }

        // Predicted density is sum of density + timestep-adjusted prediction term
        return totalDensity + timestep * totalPrediction;
    }

    float ComputePressure(Particle particle)
    {
        return stiffness * (Mathf.Pow((particle.density / Density), stiffExp) - 1);
    }

    Vector3 ComputePressureForce(Particle particle)
    {
        Vector3 totalPressureForce = Vector3.zero;
        foreach (var neighbor in particle.neighbors)
        {
            Vector3 r = particle.pos - neighbor.pos;
            float PPoverD = (particle.density == 0f) ? 0 : (particle.pressure / Mathf.Pow(particle.density, 2));
            float NPoverD = (neighbor.density == 0f) ? 0 : (neighbor.pressure / Mathf.Pow(neighbor.density, 2));
            float sum =  (PPoverD + NPoverD) / 2.0f;
            float w = -SpikyKernelFunction(r, smoothing_length).Item3;
            Vector3 gradient = w * r.normalized;
            totalPressureForce += particle.mass * sum * gradient;
        }
        return totalPressureForce * particle.density;
    }

    Vector3 ComputeViscosityForce(Particle particle)
    {
        Vector3 laplacianVelocity = Vector3.zero;
        foreach (var neighbor in particle.neighbors)
        {
            Vector3 r = particle.pos - neighbor.pos;
            Vector3 rel_vel = particle.velocity - neighbor.velocity;
            float top = Vector3.Dot(r, r * SpikyKernelFunction(r, smoothing_length).Item3);
            float bottom = Vector3.Dot(r, r) + 0.01f * Mathf.Pow(smoothing_length, 2);
            float MoverD = (particle.density == 0f) ? 0f : (particle.mass / particle.density);
            laplacianVelocity += MoverD * rel_vel * (top/bottom);            
        }
        return laplacianVelocity * 2;
    }

    (float, Vector3) KernelFunction(Vector3 r, float h)
    {
        float r_length = r.magnitude;
        float q = r_length / h;
        float factor = 0.0f;
        float alpha = 5 / (14 * Mathf.PI * Mathf.Pow(h, 2));
        if (q >= 0 && q < 1) factor = Mathf.Pow((2 - q), 3) - 4 * Mathf.Pow((1 - q), 3);
        else if (q >= 1 && q < 2) factor = Mathf.Pow((2 - q), 3);
        else if (q >= 2) factor = 0;

        float w = factor * alpha;

        //Debug.Log(w);
        return (w, w * r.normalized);
    }

    (float, Vector3, float) SpikyKernelFunction(Vector3 r, float h)
    {
        float r_length = r.magnitude;
        float q = r_length / h;
        float alpha = 5 / (14 * Mathf.PI * Mathf.Pow(h, 2));

        float factor = 0.0f;
        if (q >= 0 && q < 1) factor = Mathf.Pow((2 - q), 3) - 4 * Mathf.Pow((1 - q), 3);
        else if (q >= 1 && q < 2) factor = Mathf.Pow((2 - q), 3);

        float w = factor * alpha;
        Vector3 gradient = w * r.normalized;

        // Compute the Laplacian (example with Spiky kernel)
        float laplacian = -alpha * (1 - q);  // Ensure this is correct for your chosen kernel

        return (w, gradient, laplacian);
    }

    void KeepInBounds(Particle particle)
    {
        Vector3 size = particleMesh.bounds.extents;
        if (particle.pos.x - size.x < minCamBounds.x + widthConstrict || particle.pos.x + size.x > maxCamBounds.x - widthConstrict)
        { 
            particle.velocity.x = -particle.velocity.x * boundaryDamp;
        }
        if(particle.pos.y + size.y > maxCamBounds.y)
        {
            particle.velocity.y = -particle.velocity.y;
        }
        if (particle.pos.y - size.y < minCamBounds.y)
        { 
            particle.velocity.y = -particle.velocity.y * surfaceFriction;
        }
    }

    void SPHSplitting()
    {
        Parallel.For(0, particles.Length, i =>
        {
            FindNeighbors(i);
        });

        // Parallelize the second loop to compute forces for each particle
        Parallel.ForEach(particles, particle =>
        {
            Vector3 Fviscosity = particle.mass * kinematic_viscosity * ComputeViscosityForce(particle);
            Vector3 Fother = Vector3.down * gravity * particle.mass;
            particle.predictedVelocity = particle.velocity + timestep * (Fviscosity + Fother) / particle.mass;
            particle.predictedPosition = particle.pos + timestep * particle.predictedVelocity;
        });

        Parallel.ForEach(particles, particle =>
        {
            particle.density = PredictedDensity(particle);
            particle.pressure = ComputePressure(particle);
        });

        Parallel.ForEach(particles, particle =>
        {
            bool divideByZeroCheck = particle.density == 0;
            //Debug.Log(particle.density);
            Vector3 Fpressure = (divideByZeroCheck) ? Vector3.zero : -(particle.mass / particle.density) * ComputePressureForce(particle);
            particle.Force = Fpressure;
        });



        for (int i = 0; i < particles.Length; i++)
        {
            //Debug.Log(particles[i].mass);
            //Debug.Log(particles[i].predictedVelocity);
            //Debug.Log(particles[i].density + " " + particles[i].pressure + " " + ComputePressure(particles[i]) + " "  + particles[i].Force);
            particles[i].velocity = particles[i].predictedVelocity + timestep * particles[i].Force / particles[i].mass;
            KeepInBounds(particles[i]);
            particles[i].pos += particles[i].velocity * timestep;
            particleMatrices[i] = Matrix4x4.TRS(particles[i].pos, Quaternion.identity, particles[i].scale);
        }
    }
    
    void SPHMod()
    {
        Parallel.For(0, particles.Length, i =>
        {
            FindNeighbors(i);
            particles[i].density = ComputeDensity(particles[i]);
            particles[i].pressure = ComputePressure(particles[i]);
        });

        // Parallelize the second loop to compute forces for each particle
        Parallel.ForEach(particles, particle =>
        {
            bool divideByZeroCheck = particle.density == 0;
            Vector3 Fpressure = (divideByZeroCheck) ? Vector3.zero : -(particle.mass / particle.density) * ComputePressureForce(particle);
            Vector3 Fviscosity = particle.mass * kinematic_viscosity * ComputeViscosityForce(particle);
            Vector3 Fother = Vector3.down * gravity * particle.mass;
            particle.Force = Fpressure + Fviscosity + Fother;
        });

        for (int i = 0; i < particles.Length; i++)
        {
            particles[i].velocity += timestep * particles[i].Force / particles[i].mass;
            KeepInBounds(particles[i]);
            particles[i].pos += particles[i].velocity * timestep;
            particleMatrices[i] = Matrix4x4.TRS(particles[i].pos, Quaternion.identity, particles[i].scale);
        }
    }

    void Update()
    {
        // Update particle positions
        //SPHMod();
        SPHSplitting();
        particleMaterial.SetPass(0);
        // Draw all instances
        Graphics.DrawMeshInstanced(particleMesh, 0, particleMaterial, particleMatrices);
    }
}
