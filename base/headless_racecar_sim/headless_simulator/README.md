# Headless RACECAR simulator

This package implements a simulator for the RACECAR which does not rely on Gazebo. It is designed to be used either without a visual frontend, or with RViz. The primary goals are flexibility, efficiency, and simplicity. Ideally, these goals should allow this simulator to be used on virtual machines running on underpowered laptops at a reasonably high rate.

## Installation

The following components are required:
- RangeLibc

## License

MIT License

## Laser Scanner

The laser scanner messages are simulated via RangeLibc. If available, an NVIDIA GPU can be used to further accelerate the ray casting computation.

## Odometry

Perfect odometry is provided, optionally with mixed in noise (not yet implemented).

## Cameras

This framework is designed to allow for simulation of depth or RGB cameras via standard and efficient rendering techniques.

## Environment

In contrast to Gazebo, this simulator uses occupancy grid maps to specify the environment. This choice was made to allow easy world specification which as closely as possible matches the real world.

## Simulation

### Physics Based

not implemented

### Ackermann Based

### Empirical Based

