# ABB YuMi Task-Priority Control and Collision Avoidance

This code has been adapted by **Tobias Orafu, Emediong Moffat, and Kofoworola Oyeniyi** as a group project to work on the **ABB YuMi dual-arm robot (left arm)**.

The MATLAB scripts correspond to simulations based on the IEEE Robotics and Automation Letters article:

> **“Task Priority Matrix at the Acceleration Level: Collision Avoidance Under Relaxed Constraints”**  
> Maram Khatib, Khaled Al Khudir, and Alessandro De Luca  
> Sapienza University of Rome, Italy.

The project investigates the kinematics and task-priority control of the ABB YuMi robot using both **velocity-level** and **acceleration-level** controllers, including collision avoidance with **static and dynamic obstacles**.

## Project Contents

- `kinematics/` – direct/forward and differential kinematics for the ABB YuMi left arm.
- `common/` – shared robot model, task-priority and collision-avoidance helper functions.
- `scenarios/static_velocity/` – velocity-level control with static obstacles.
- `scenarios/static_acceleration/` – acceleration-level control with static obstacles and trajectory timing-law analysis.
- `scenarios/dynamic_velocity/` – velocity-level control with moving obstacles.
- `scenarios/dynamic_acceleration/` – acceleration-level control with moving obstacles.

## Running the Simulations

1. Open the repository folder in MATLAB.
2. Run `setup_project.m` once. This restores the compressed `common/YUMI_LWR.m` robot-model class and adds the shared functions to the MATLAB path.
3. Open one of the folders under `scenarios/`.
4. Run `main.m` to execute that controller scenario.
5. Run `common/plot_results.m` after a simulation to generate the result plots.

The kinematic analysis scripts can be run separately from the `kinematics/` folder.

## Controller Scenarios

- Static obstacle – velocity-level controller
- Static obstacle – acceleration-level controller
- Dynamic obstacle – velocity-level controller
- Dynamic obstacle – acceleration-level controller

Pre-rendered MP4 demonstrations were produced as part of the project but are not included in this source repository.

## Group Members

- Tobias Orafu
- Emediong Moffat
- Kofoworola Oyeniyi

## Original Authors and Attribution

The controller implementation is based on MATLAB scripts released by **Maram Khatib, Khaled Al Khudir, and Alessandro De Luca** for their 2020 IEEE RA-L paper. The original source code is licensed under the **Apache License 2.0**. Original copyright and licence notices have been retained in the relevant source files, and modified files carry a notice identifying the ABB YuMi group-project changes.

Original repository: `maram-khatib/task-priority-relaxed-constraints` on GitHub.

See [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) and [`LICENSE`](LICENSE) for attribution and licence information.

## Academic Project Note

This repository contains the **Part A implementation and simulation code only**. The university assessment brief and submitted presentation are intentionally not included.
