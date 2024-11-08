function params = loadExperimentData(arrangement,  SSLmethod)
    % Load experiment data based on specified arrangement (e.g., 'I' or 'II')
    % and SSL method (e.g., 'GCC-PHAT' or 'MVDR')
    % Input:
    %   arrangement - character string ('I' or 'II') specifying the source arrangement
    %    SSLmethod - character string ('GCC-PHAT' or 'MVDR') specifying the SSL method
    % Output:
    %   params - structure containing all required parameters for the experiment

    baseDir = ".\exp_data\"; 
    posePath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "pose", "pose_theta.xlsx");
    azimuthPath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "azimuth", sprintf("azEst_%s.xlsx",  SSLmethod));
    imagePath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "map", sprintf("map_%s.pgm", arrangement));

    params.robotPoses = readmatrix(posePath);                % Pose Estimates Table
    params.observations = deg2rad(readmatrix(azimuthPath));  % DoA Estimates Table
    image = imread(imagePath);                 
    params.image = flipud(image(1:250, 1:300));              % Occupancy map
    params.resolution = 0.05;                                % Occupancy map resolution
    params.origin = [-4.000000, -5.000000, 0.000000];        % Occupancy map origin
    params.sigma = deg2rad(5);                               % Standard deviation of delta theta
    params.associatedRange = 3 * params.sigma;               % Observation associated range

    if arrangement == "I"
        params.srcGroundTruth = [ 0, -1.2, 0;
                                2.5, -1.2, 0;
                                5.0, -1.2, 0;
                                7.5,    0, 0;
                                7.5,  3.6, 0;
                                5.0,  4.8, 0;
                                2.5,  4.8, 0;
                                  0,  4.8, 0;
                              -1.25,  3.6, 0;
                              -1.25,  1.2, 0];
        params.numObservations = 3; % Number of considered DoA observations per time step
    elseif arrangement == "II"
        params.srcGroundTruth = [1.25,  0.6, 0;
                                 1.25, -0.6, 0;
                                 2.50,  0.6, 0;
                                 2.50, -0.6, 0;
                                 3.74,  0.6, 0;
                                 3.74, -0.6, 0;
                                 4.99,  0.6, 0;
                                 4.99, -0.6, 0;
                                 6.23,  0.6, 0;
                                 6.23, -0.6, 0];
        params.numObservations = 4; 
    else
        error('Invalid arrangement specified. Use ''I'' or ''II''.');
    end
end
