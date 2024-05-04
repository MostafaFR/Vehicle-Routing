# MATLAB Simulink for Autonomous Multi-Robot Systems

## Project Overview

This repository hosts a comprehensive MATLAB Simulink project aimed at the development and control of autonomous multi-robot systems (MRS). The project intricately combines elements of navigation, localization, and communication, enabling a fleet of robots to perform complex tasks collaboratively in a dynamic environment. This project is based on Robotarium instances.

## Key Features

### Navigation
- **Spline-Based Path Planning**: Implements Catmull-Rom spline for smooth trajectory planning between waypoints, minimizing abrupt changes in direction and speed.
- **Formation Control**: Develops methods to maintain geometric formations, such as diamond and line formations, using Laplacian matrices and control theory to manage distances and alignments between robots.
- **Obstacle Avoidance**: Integrates reactive and predictive strategies to navigate around obstacles effectively, ensuring the robots can adapt their paths in real-time to unforeseen changes in the environment.

### Localization
- **Odometric and Collaborative Localization**: Utilizes odometric models combined with collaborative techniques to enhance the precision of robot positioning within the group.
- **Error Handling and Sensor Integration**: Implements advanced filtering techniques, such as Kalman filters, to manage and mitigate the errors from sensory inaccuracies and environmental factors.

### Communication
- **Quality of Service (QoS) Implementation**: Focuses on maintaining high-quality communication links under varying conditions, addressing issues like packet loss and delays due to environmental factors.
- **Adaptive Communication Protocols**: Designs protocols that adjust to changes in robot formations and proximity, ensuring reliable data exchange even under challenging conditions.

## Project Structure

- **Simulink Models**: Contains all the Simulink models that simulate the different aspects of the multi-robot systems. These models are meticulously annotated to guide users through the functionalities implemented.
- **MATLAB Scripts**: Includes all the scripts used to set up, run simulations, and analyze outcomes. Scripts are modular and well-documented to allow easy customization and testing.
- **Data and Results**: Provides example datasets used in simulations along with the results and visualizations of various experiments conducted during the project development.

## Usage

To run simulations:
1. Open the desired Simulink model from the `Simulink Models` directory.
2. Set parameters as desired in the MATLAB script linked to the model.
3. Execute the script or use the Simulink interface to start the simulation.
