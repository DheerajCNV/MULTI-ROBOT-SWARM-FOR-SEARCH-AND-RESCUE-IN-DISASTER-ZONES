# Multi-Robot Swarm for Search and Rescue in Disaster Zones

This repository contains the MATLAB simulation code, project report, and results for the MAE 598 Multi-Robot Systems final project at Arizona State University. The project models a decentralized, energy-aware multi-robot swarm for dynamic area coverage and hazard avoidance in disaster scenarios.

## Repository Structure
├── code.m\
├── report\
│ └── report.pdf # Detailed project report\
├── results\
│ ├── simulation_video.mp4 # Recorded simulation video\
│ ├── coverage_map.png # Final binary occupancy grid\
│ ├── disaster_influence_heatmap.png # Disaster influence visualization\
│ ├── robot_trajectories.png # Robot path tracking\
│ ├── coverage_over_time.png # Coverage % vs. simulation steps\
│ └── unsafe_robots_over_time.png # Unsafe robots vs. simulation steps\
└── README.md

## Project Overview

The objective of this project is to develop a robust multi-robot system that autonomously explores and covers a disaster-affected region while avoiding dynamically expanding hazard zones. The system utilizes Voronoi-based partitioning, repulsive force modeling, and energy-aware motion planning for decentralized coordination.

### Key Features

- **MATLAB Simulation**: Models 20 robots over 250 time steps in a 200×200 m environment.
- **Dynamic Disaster Zones**: 5 expanding hazard zones with randomized growth rates.
- **Voronoi Coordination**: Prevents overlap and optimizes spatial distribution.
- **Repulsion-Based Hazard Avoidance**: Robots maintain safe distances from hazards.
- **Energy-Aware Planning**: Gradient-based motion conserves energy, retaining >80% battery per robot.
- **Visualizations**: Includes real-time plots, coverage metrics, and a 3D terrain-hazard visualization.

## Usage

1. Clone this repository:
```matlab
git clone https://github.com/yourusername/multi-robot-swarm-disaster-response.git
cd multi-robot-swarm-disaster-response
```
2. Open MATLAB and run the simulation:
```matlab
run('code.m')
```
3. Results such as plots, graphs, and final maps will be generated in the results/ directory.

## Results Summary

- Coverage Achieved: >95% of the area

- Energy Efficiency: >80% energy retained per robot

- Safety: 100% avoidance of disaster zones through repulsion logic

- Scalability: Supports decentralized operation with no central controller

## Report
For a complete description of the mathematical model, algorithms, simulation parameters, results, and future work, refer to the report.pdf in the report/ directory.

## Authors
- Naga Venkata Dheeraj Chilukuri\
- Shashank Anjan Varma Yenuka\
- Amruth Parimi

## Course
- MAE 598: Multi-Robot Systems\
- Arizona State University – Fall 2024\
- Instructor: Dr. Spring Berman
