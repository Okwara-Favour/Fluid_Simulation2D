# Fluid Simulation
A Smoothed Particle Hydrodynamics implementation of the fluid simulation. The project was implemented in Unity C# and optimized with a compute shader.

_Research Paper: [CSC473_FinalPaper_Fluid_Research001.pdf](https://github.com/user-attachments/files/19255342/CSC473_FinalPaper_Fluid_Research001.pdf)_


# Demo
https://github.com/user-attachments/assets/13adc379-109d-46e0-bded-9fdf941ae72e

## Instructions
- **Setup**
  - Create an empty object and name it
  - Please select the Fluid Simulation script - **OptFluidSim2D** and assign it to the object
  - Assign a material, a material _New Material_ is provided in the assets to the material slot
  - Assign the compute shader stored in folder **CompShaders/FluidSim.compute** to the object
  - Ensure the mesh is given. A good one is the Sphere mesh
  - For use of **FLuidSim2D** script, ensure **SpartialGrid** script is also assigned.
- **Fields**
  - For a stable simulation ensure the delta time < 0.3
  - Ensure max speed is not 0
  - Ensure smoothing length > 0.5
  - Kinematic viscosity is balanced by the smoothing length that is, for some level of increase in kinematic viscosity ensure an increase smoothing length
  - At smoothing length of value 2, kinematic viscosity can be increased past a 30 threshold
