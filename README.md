
## Introduction
This repository contains experimental data and scripts for offline and online Acoustic Scene Mapping (ASM) experiments. The experiments explore different arrangements of sound sources, providing insights into the effectiveness of the proposed I-ASM algorithm.

## Experimental Data
The experimental data is organized in the `exp_data` directory. Each arrangement has its corresponding folder within `exp_data`, containing relevant data files. To access the data:

1. Navigate to the `exp_data` directory.
2. Choose the arrangement of interest.
3. Access the data files within the arrangement folder.

## Offline ASM Evaluation
To visualize the offline SLAM results, follow these steps:

1. Open the `ASM_offline.m` script.
2. Modify the script by commenting or uncommenting sections to select different arrangements' data.
3. Execute the script to visualize the offline mapping results.

## Online ASM Implementation
For online ASM implementation, adhere to the following steps:

1. Ensure real-time data is provided following the naming conventions demonstrated in the `exp_data` directory.
2. Run the `ASM_online.m` script to perform online ASM.

## Note
- Ensure that required dependencies are installed before running the scripts. (MATLAB Toolbox)
- Make sure to review and modify scripts according to specific system configurations and requirements.
