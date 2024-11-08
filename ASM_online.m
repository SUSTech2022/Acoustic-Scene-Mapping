clc
clear
close all

addpath("functions\")
addpath("mbss_locate\v2.0\localization_tools")
addpath("mbss_locate\v2.0\localization_tools\pair_angular_meths");



%% Load experiment data

arrangement = "II"; % Source arrangement: 'I' or 'II'
SSLmethod = "GCC-PHAT"; % SSLmethod: 'MVDR' or 'GCC-PHAT'

baseDir = ".\exp_data\"; 
posePath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "pose", "pose_theta.xlsx");
imagePath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "map", sprintf("map_%s.pgm", arrangement));
audioPath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "audio");

azimuthPath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "online", sprintf("online_azEst_%s.xlsx", SSLmethod));
if exist(azimuthPath, 'file')
    error('The azEst file already exists. Please delete it.')
end

ospaPath = fullfile(baseDir, sprintf("arrangement_%s", arrangement), "online", sprintf("online_ospa_%s.xlsx",  SSLmethod));
if exist(ospaPath, 'file')
    error('The OSPA file already exists. Please delete it.')
end

image = imread(imagePath);
image = flipud(image(1:250, 1:300));  
resolution = 0.05; % occupancy map resolution, copied from yaml
origin = [-4.000000, -5.000000, 0.000000]; % occupancy map origin, copied from yaml

if arrangement == "I"
    srcGroundTruth = [[0, -1.2, 0];[2.5, -1.2, 0];[5.0, -1.2, 0];[7.5, 0, 0];[7.5,  3.6, 0];[5.0,  4.8, 0];[2.5,  4.8, 0];[0,  4.8, 0];[-1.25, 3.6, 0];[ -1.25, 1.2, 0]]; % assignment I
    numObservations = 3; % number of considered DoA per time step, N
elseif arrangement == "II"
    srcGroundTruth = [[1.25,  0.6, 0];[1.25,  -0.6, 0];[2.50,  0.6, 0];[2.50,-0.6,  0];[3.74 , 0.6, 0];[3.74, -0.6, 0];[4.99, 0.6, 0];[4.99, -0.6, 0];[6.23, 0.6, 0];[6.23, -0.6, 0]]; % assignment II
    numObservations = 4;
end

srcGroundTruth(:,1) = (srcGroundTruth(:,1)-origin(1))/resolution;srcGroundTruth(:,2) = (srcGroundTruth(:,2)-origin(2))/resolution; 
sigma = deg2rad(5);  % standard deviation of delta theta
associatedRange = 3*sigma; % observation associated range, gamma

%% Initialization

visualize = 0;

% Initialize particle filter parameters
D = 9; % 9,16,25,36 
[row, col] = initializeParticles(image, D);
numParticles = numel(row); % Number of particles equals the number of pixels found

% DBSCAN clustering parameters
epsilon = 0.1/resolution; % Distance threshold used to define neighborhoods. If the distance between two points is less than or equal to epsilon, then these two points are considered neighbors.
MinPts = numParticles*0.1; % Quantity threshold used to define core points. If an epsilon-neighborhood of a point contains at least MinPts points (including the point itself), then this point is considered a core point.

c = 1/resolution; % cutoff distance 1m
p = 1; %  first order OSPA

merge_tresh = 0.5/resolution;

% Initialize the file counter
fileCounter = 0;

% Display initial map and source ground truths
% set(gcf,'Position',[300 400 800 600])
imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 300);
set(gca, 'YDir', 'normal');
title("k = "+num2str(0));
axis equal;
axis on;
axis image
hold on
scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled');

pause(0.5)

last_length = 0;
while true
    disp('waiting for the audio files ...')
    % Get a list of all the wav files in the folder
    files = dir(fullfile(audioPath, '*.wav'));
    
    % Check if there are any new files
    while length(files) > fileCounter
        fileCounter = fileCounter + 1;
        fprintf('Calculating obervation %d:',fileCounter)
        pause(0.5)
        filename_wav = fullfile(audioPath, files(fileCounter).name);
        azEst = SSL(filename_wav, SSLmethod); % 计算DoA
        writematrix(azEst, azimuthPath, 'WriteMode', 'append');         
    end
        if  (fileCounter >= 1) && (length(files) > last_length) 
            last_length = length(files);
            
            robotPoses = readmatrix(posePath); % 读取最新的Pose Estimates Table
            observations = deg2rad(readmatrix(azimuthPath)); % 读取最新的DoA Estimates Table 
            numTimeSteps = size(observations,1); % number of discrete time steps, K
            
            robotPoses(1:numTimeSteps,1) = (robotPoses(1:numTimeSteps,1)-origin(1))/resolution;
            robotPoses(1:numTimeSteps,2) = (robotPoses(1:numTimeSteps,2)-origin(2))/resolution;
                        
            % Initialize array of detected source particle filters
            detectedSourceFilters = [];
            
            % Initialize the updated DoA Estimates Table 
            updatedObservations = observations;
            
            % Filtering - Clustering - Implicit Associating cycle
            tic % Start timer
            roundCount = 0; % iteration round count
            while true                                
            
                % Particle Filtering
                disp('Filtering...')
                [particleFilter,~] = particleFiltering(roundCount,sigma,numTimeSteps, numObservations, updatedObservations, robotPoses, srcGroundTruth,image,row, col,resolution);

                % DBSCAN clustering
                newParticleFilters = getMaxClusterFromDBSCAN(particleFilter, epsilon, MinPts);
            
                if isempty(newParticleFilters)
                    break;
                end
            
                roundCount = roundCount + 1;
                disp(['Round ',num2str(roundCount),' finished.']);
                           
                % Associate observations and update the DoA Estimates Table  
                [newParticleFilters, updatedObservations] = associateObservations(newParticleFilters, numTimeSteps, numObservations, updatedObservations, robotPoses, associatedRange);
               
                % Add new estimate to the set
                detectedSourceFilters = [detectedSourceFilters, newParticleFilters];
            end

            % Merge similar estimates
            detectedSourceFilters = mergeClusters(detectedSourceFilters, merge_tresh);
            
            % End timer
            elapsedTime = toc;
            disp(['The mapping took ', num2str(elapsedTime), ' seconds.']);
            
            % Visualize ASM results
            num_detected_sources  = size(detectedSourceFilters,2);
            
            clf;
            % set(gcf,'Position',[300 400 800 600])
            imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 300);
            set(gca, 'YDir', 'normal');
            hold on;
            for i = 1:num_detected_sources
                particleFilter = detectedSourceFilters{i};
                plot(particleFilter.particles(:,1), particleFilter.particles(:,2), 'y.');
                scatter(particleFilter.State(:,1), particleFilter.State(:,2), 50, 'b^', 'filled');
            end
            scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled'); 
            scatter(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), 10, 'k', 'filled'); 
            plot(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), 'k--'); 
            
            for i = 1:numObservations 
                angle = wrapToPi(observations(numTimeSteps,i)+ deg2rad(robotPoses(numTimeSteps,3)));
                quiver(robotPoses(numTimeSteps,1), robotPoses(numTimeSteps,2), cos(angle), sin(angle),10); % plot N DoA observations of current time step k
            end
            title("k = "+num2str(numTimeSteps));
            axis equal;
            axis on;
            axis image
            hold off;

            % save fig
            % savePath = exp_folder + 'mapping_result\online\step' + num2str(numTimeSteps) +'.fig';
            % saveas(gcf, savePath);

            % Error analysis
            [ospa_distance, ospa_loc,ospa_card] = calculate_OSPA_distance(detectedSourceFilters, srcGroundTruth, resolution,c,p);
            ospa = [numTimeSteps,ospa_distance,ospa_loc,ospa_card]; % OSPA,loc,cardility
            writematrix(ospa, ospaPath, 'WriteMode', 'append');
            disp('OSPA saved.');
            fprintf('OSPA distance is %.3f meters.\n', ospa_distance);
            fprintf('OSPA localization distance is %.3f meters.\n', ospa_loc);
            fprintf('OSPA cardility error is %d.\n', ospa_card);
        end          
end








