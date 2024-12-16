# Quadrotor Simulation

This repository contains classes to simulate the control, observers, or any dynamic behavior of a quadrotor. The simulation is designed with modularity in mind, allowing for easy extension and integration of new controllers, estimators, or observers.

## Project Structure

- **`Drone`**: This is the main class where the quadrotor's dynamics and control logic are implemented. It currently includes a quaternion-based PD controller.
- **`DroneDynamic`**: Implements the physical dynamics of the quadrotor, including translational and rotational states.
- **`DroneDataExtention`**: Handles data logging and visualization utilities, including arrays for storing simulation results and methods for plotting.

## How to Run the Simulation

To simulate the quadrotor's behavior:
1. Use the `Drone` class to define the quadrotor's initial conditions, control gains, and desired trajectory.
2. Run the simulation using the `simDrone` script, which provides an example setup.
3. The simulation will execute and generate plots for position, orientation, and errors.

## Adding Custom Modules

### Extending the Controller or Observer
To develop new modules:
1. **Inherit from the `Drone` class**:
   - Create a new class that extends `Drone`.
   - Override methods as needed to implement your custom logic.
2. **Modify the `Drone` class directly**:
   - Update the existing `applyControl` or relevant methods.

### Adding New Plots
To visualize additional data:
1. Add a new array property in the `DroneDataExtention` class.
2. Update the `updateDroneDataExtention` method in the `Drone` class to populate the new array.
3. Implement a plotting method in the `DroneDataExtention` class to generate the desired visualization.

## Current Features

- Quaternion-based PD controller.
- Disturbance handling for both translational and rotational dynamics.
- Modular design for ease of extension.
- Built-in methods to plot:
  - Position trajectory.
  - Orientation errors.
  - Observer states (if applicable).

## Example Usage

The entry point for the simulation is the `Drone` class. You can set up a simulation as follows:

```matlab
mass = 0.405;
q = quaternion(1, 0, 0, 0);
x0 = 0; y0 = 0; z0 = 0;
dt = 0.01;

% Initialize Drone
drone = Drone(mass, q, x0, y0, z0, dt);
drone = drone.setControlGains(4, 1, 2, 20, 2, 6);
drone = drone.setAimPoint(1, 2, 3);

% Run simulation
sim_time = 8; % seconds
iterations = sim_time / dt;
for i = 1:iterations
    drone = drone.update();
end

% Plot results
drone.plotPosition();
drone.plotErrors();
drone.plotOrientation();
drone.animateDroneTrajectory();
```