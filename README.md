# Introduction
This repository contains code and experimental data for the paper **"I-ASM: Iterative Acoustic Scene Mapping for Enhanced Robot Auditory Perception in Complex Indoor Environments"**, presented at the *International Conference on Intelligent Robots and Systems (IROS) 2024*.

The paper addresses the challenge of **acoustic scene mapping (ASM)** in complex indoor environments with multiple sound sources. Unlike existing methods that rely on prior data association or SLAM frameworks, we propose a novel particle filter-based iterative framework, termed **I-ASM**, for ASM using a mobile robot equipped with a microphone array and LiDAR. 

**I-ASM** leverages an innovative *implicit association* to align sound sources with Direction of Arrival (DoA) observations without explicit pairing, streamlining the mapping process. Given inputs like an occupancy map, DoA estimates from various robot positions, and robot pose data, I-ASM performs multi-source mapping through an iterative cycle of *“Filtering-Clustering-Implicit Associating”*. Tested in real-world scenarios with up to 10 concurrent sound sources, I-ASM demonstrates robustness against missing and false DoA estimates, achieving high-quality ASM results.


## Citation
If you use the code or data from this repository, please cite our work:

> **L. Fu, Y. He, J. Wang, X. Qiao, and H. Kong***, “I-ASM: Iterative Acoustic Scene Mapping for Enhanced Robot Auditory Perception in Complex Indoor Environments”, International Conference on Intelligent Robots and Systems (IROS), 2024.*

## Repository Structure
- **`exp_data/`**: Contains all experimental data, organized by different test arrangements. Each arrangement has its own folder with relevant data files.
  
## Usage Guide

### Experimental Data Access
To access and explore the experimental data:

1. Go to the **`exp_data/`** directory.
2. Select the arrangement of interest.
3. Open the corresponding folder to access data files for that arrangement.


### Offline ASM Evaluation
To visualize offline ASM results:

<<<<<<< HEAD
1. Open the **`ASM_offline.m`** script.
2. Modify the script to select the desired arrangement and Sound Source Localization (SSL) method.
3. Run the script to visualize the offline mapping results.

### Online ASM Implementation
To perform online ASM during robot movement:

1. As the robot moves, save audio files (in `.wav` format) to the **`exp_data/audio/`** folder and the corresponding pose data in **`exp_data/pose/pose_theta.xlsx`**.
2. Run the **`ASM_online.m`** script to start the online ASM process.
3. The mapping will update automatically when new audio data is detected in the audio folder.

## Note
Ensure that the required MATLAB Toolbox is installed before running the scripts.

## Contact
For questions or feedback, please contact [linya.fu@outlook.com].
