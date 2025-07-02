Modeling and Simulation of Dexter-6 Pro

This repository contains the complete MATLAB-based modeling and simulation framework for Dexter-6, a 6-DOF lightweight robotic manipulator.

![Dexter6-Pro(6DOF_RoboticArm)](https://github.com/user-attachments/assets/4c285935-e235-4d78-8fff-b292335bcb67)

Project Overview

This project includes:
- Symbolic dynamic modeling using the Euler–Lagrange formulation.
- Forward and Inverse Kinematics Simulink models.
- A validated Computed Torque Controller (CTC) with PD control in Simulink.
- Trajectory tracking using custom inverse dynamics and kinematics.

Repository Contents

- Dynamics6DOF_code.m – Symbolic derivation of dynamic equations
- Dynamics_Simulink.slx – Simulink model implementing inverse dynamics controller
- ForwardKinematics.slx – Simulink model for forward kinematics
- IK_PickPlaceTrajectory.slx – Inverse kinematics trajectory generation for pick-and-place task
- README.md – Project overview and usage instructions

## 🧪 Simulation Environment

All simulations are performed in MATLAB R2023a using Simulink and Simscape Multibody.
