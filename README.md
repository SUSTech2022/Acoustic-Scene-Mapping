## Introduction
This repository contains the codes and experimental data  for the paper "I-ASM: Iterative acoustic scene mapping for enhanced robot auditory perception in complex indoor environments" presented at the International Conference on Intelligent Robots and Systems (IROS), 2024. 
The paper addresses the challenge of acoustic scene mapping (ASM) in complex indoor environments with multiple sound sources. Unlike existing methods that rely on prior data association or SLAM frameworks, we propose a novel particle filter-based iterative framework, termed I-ASM, for ASM using a mobile robot equipped with a microphone array and LiDAR. I-ASM harnesses an innovative ''implicit association”to align sound sources with Direction of Arrival (DoA) observations without requiring explicit pairing, thereby streamlining the mapping process. Given inputs including an occupancy map, DoA estimates from various
robot positions, and corresponding robot pose data, I-ASM performs multi-source mapping through an iterative cycle of “Filtering-Clustering-Implicit Associating”. The proposedframework has been tested in real-world scenarios with up to 10 concurrent sound sources, demonstrating its robustness against missing and false DoA estimates while achieving high-quality ASM results.

## Citation
If you use the code or data from this repository, please cite our work:
L. Fu, Y. He, J. Wang, X. Qiao, and H. Kong*, “I-ASM: Iterative acoustic scene mapping for enhanced robot auditory perception in complex indoor environments”, International Conference on Intelligent Robots and Systems (IROS), 2024.

## Experimental Data
The experimental data is organized in the `exp_data` directory. Each arrangement has its corresponding folder within `exp_data`, containing relevant data files. To access the data:

1. Navigate to the `exp_data` directory.
2. Choose the arrangement of interest.
3. Access the data files within the arrangement folder.

## Offline ASM Evaluation
To visualize the offline ASM results, follow these steps:

1. Open the `ASM_offline.m` script.
2. Modify the script by selecting arrangement and SSL method.
3. Execute the script to visualize the offline mapping results.

## Online ASM Implementation
For online ASM implementation, adhere to the following steps:

1. As the robot moves: save the audio file in the `exp_data\audio` folder in '.wav' form, and its pose data in the `exp_data\pose\pose_theta.xlsx`
2. Run the `ASM_online.m` script to perform online ASM.
3. The mapping result will update when there's new audio data detected in the audio folder.

## Note
- Ensure that required MATLAB Toolbox are installed before running the scripts. 
- Make sure to review and modify scripts according to specific system configurations and requirements.

## Contact
For questions or feedback, please contact [linya.fu@outlook.com].
