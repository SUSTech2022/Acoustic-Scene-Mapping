clc
clear
close all

addpath("functions\")


%% Load experiment data

% Source Arrangement I  K = 71
robotPoses = readmatrix(".\exp_data\arrangement_I\pose\pose_theta.xlsx"); % Pose Estimates Table
observations = deg2rad(readmatrix(".\exp_data\arrangement_I\azimuth\azEst_MVDR.xlsx")); % DoA Estimates Table - MVDR
% observations = deg2rad(readmatrix(".\exp_data\arrangement_I\azimuth\azEst_GCC-PHAT.xlsx")); % DoA Estimates Table - GCC-PHAT
image = imread(".\exp_data\arrangement_I\map\map_I.pgm"); % Occupancy map
image = flipud(image(1:250, 1:300));  
resolution = 0.05; % occupancy map resolution, copied from yaml
origin = [-4.000000, -5.000000, 0.000000]; % occupancy map origin, copied from yaml
srcGroundTruth = [[0, -1.2, 0];[2.5, -1.2, 0];[5.0, -1.2, 0];[7.5, 0, 0];[7.5,  3.6, 0];[5.0,  4.8, 0];[2.5,  4.8, 0];[0,  4.8, 0];[-1.25, 3.6, 0];[ -1.25, 1.2, 0]]; 
numObservations = 3; % number of considered DoA per time step, N
sigma = deg2rad(5); % standard deviation of delta theta
associatedRange = 3*sigma; % observation associated range, gamma


%  Source Arrangement II  K = 40
% robotPoses = readmatrix(".\exp_data\arrangement_II\pose\pose_theta.xlsx"); % Pose Estimates Table
% observations = deg2rad(readmatrix(".\exp_data\arrangement_II\azimuth\azEst_MVDR.xlsx")); % DoA Estimates Table - MVDR
% % observations = deg2rad(readmatrix(".\exp_data\arrangement_II\azimuth\azEst_GCC-PHAT.xlsx")); % DoA Estimates Table - GCC-PHAT
% image = imread(".\exp_data\arrangement_II\map\map_II.pgm"); % Occupancy map
% image = flipud(image(1:250, 1:300));  
% resolution = 0.05; % occupancy map resolution, copied from yaml
% origin = [-4.000000, -5.000000, 0.000000]; % occupancy map origin, copied from yaml
% srcGroundTruth = [[1.25,  0.6, 0];[1.25,  -0.6, 0];[2.50,  0.6, 0];[2.50,  -0.6,  0];[3.74 , 0.6, 0];[3.74, -0.6, 0];[4.99, 0.6, 0];[4.99, -0.6, 0];[6.23, 0.6, 0];[6.23, -0.6, 0]];
% numObservations = 4; % number of considered DoA per time step, N
% sigma = deg2rad(5);  % standard deviation of delta theta
% associatedRange = 3*sigma; % observation associated range, gamma


%% Initializatioin

numTimeSteps = size(robotPoses,1); % number of discrete time steps, K
for k = 1:numTimeSteps
    robotPoses(k,1) = (robotPoses(k,1)-origin(1))/resolution;
    robotPoses(k,2) = (robotPoses(k,2)-origin(2))/resolution;
end
srcGroundTruth(:,1) = (srcGroundTruth(:,1)-origin(1))/resolution;
srcGroundTruth(:,2) = (srcGroundTruth(:,2)-origin(2))/resolution;

visualize = 0; % Whether visualizng the estimate of each iteration


% Find all pixels with grayscale values greater than or equal to 250 (white, unoccupied)
[row, col] = find(image >= 250);
numCoords = length(row); % Calculate total number of coordinates
% Randomly select 1/D of the coordinates (otherwise, too many particles causing clustering to freeze)
D = 9; % 9,16,25,36 
indices = randsample(numCoords, floor(numCoords/(D))); % Generate random indices, approximately 0.15m*0.15m per particle on average
row = row(indices); % Select corresponding rows
col = col(indices); % Select corresponding columns

% Initialize particle filter parameters
numParticles = numel(row); % Number of particles equals the number of pixels found

% DBSCAN clustering parameters
epsilon = 0.1/resolution; % Distance threshold used to define neighborhoods. If the distance between two points is less than or equal to epsilon, then these two points are considered neighbors.
MinPts = numParticles*0.1; % Quantity threshold used to define core points. If an epsilon-neighborhood of a point contains at least MinPts points (including the point itself), then this point is considered a core point.

% Initialize array of detected source particle filters
detectedSourceFilters = [];

% Initialize the updated DoA Estimates Table 
updatedObservations = observations;


%% Filtering - Clustering - Implicit Associating cycle

% Start timer
tic

roundCount = 0; % iteration round count
weightSum = {};
while true
    roundCount = roundCount + 1;
    close all
    
    % Particle Filtering
    disp('Filtering...')
    [particleFilter,weight_sum] = particleFilteringMap(roundCount,sigma,numTimeSteps, numObservations, updatedObservations, robotPoses, srcGroundTruth,image,row, col,resolution,visualize);

    % DBSCAN clustering
    newParticleFilters = createParticleFiltersFromDBSCAN(particleFilter, epsilon, MinPts,resolution);

    if isempty(newParticleFilters)
        roundCount = roundCount - 1;
        break;
    end
  
    disp(['Round ',num2str(roundCount),' finished.']);

    % Visualize estimate of each iteration
    if visualize
        figure;
        imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
        set(gca, 'YDir', 'normal');
        hold on;
        num_newParticleFilters  = size(newParticleFilters,2);
        for i = 1:num_newParticleFilters
            particleFilter = newParticleFilters{i};
            plot(particleFilter.particles(:,1), particleFilter.particles(:,2), 'y.','MarkerSize', 10,'DisplayName', 'Clustered particles');
            scatter(particleFilter.State(:,1), particleFilter.State(:,2), 50, 'b^', 'filled','DisplayName','Source estimate');
        end
        scatter(weight_sum.State(:,1), weight_sum.State(:,2), 50, 'c^', 'filled','DisplayName', 'Weighted Sum'); 
        scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled','DisplayName', 'Source ground truth'); 
        scatter(robotPoses(1:t,1), robotPoses(1:t,2), 10, 'k', 'filled','DisplayName', 'Robot position'); 
                plot(robotPoses(1:t,1), robotPoses(1:t,2), 'k--','DisplayName', 'Robot trajectory'); 
        title(sprintf("Clustering Result: Round %d", roundCount));
        legend('Location', 'north east');
        axis on;
        axis image
        hold off;
        pause(2); 
    end

    % Associate observations and update the DoA Estimates Table  
    [newParticleFilters, updatedObservations] = associateObservationsMap(newParticleFilters, numTimeSteps, numObservations, updatedObservations, robotPoses, associatedRange);
    
    % Add new estimate to the set
    detectedSourceFilters = [detectedSourceFilters, newParticleFilters];
    weightSum{end+1} = weight_sum;
end

% Merge similar estimates
merge_tresh = 0.5/resolution; % 0.5m
detectedSourceFilters = mergeClusters(detectedSourceFilters, merge_tresh);
weightSum = mergeClusters(weightSum, merge_tresh);


elapsedTime = toc; % End timer
disp(['Your particle filter code took ', num2str(elapsedTime), ' seconds to run.']);

%% Error analysis

% OSPA distance
c = 1/resolution; % cutoff distance 1m
p = 1; %  first order
[ospa_distance, ospa_loc,ospa_card] = calculate_OSPA_distance(detectedSourceFilters, srcGroundTruth, resolution,c,p);
[ospa_distance_ws, ospa_loc_ws,ospa_card_ws] = calculate_OSPA_distance(weightSum, srcGroundTruth, resolution,c,p);
mapping_result = [ospa_distance,ospa_loc,ospa_card,ospa_distance_ws, ospa_loc_ws,ospa_card_ws]; % OSPA,loc,cardility

fprintf('OSPA distance is %.3f m.\n', ospa_distance);
fprintf('OSPA localization error is %.3f m.\n', ospa_loc);
fprintf('OSPA cardility error is %d m\n', ospa_card);
   
%% Visualize ASM results

I = numel(detectedSourceFilters); % number of detected sources

% plot iterative clusters 
colors = hsv(I+1); % skip red
figure;
imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
set(gca, 'YDir', 'normal');
hold on
pause(0.5);
for i = 1:I
    particleFilter = detectedSourceFilters{i};
    plot(particleFilter.particles(:,1), particleFilter.particles(:,2), '.', 'Color', colors(i+1,:));
    hold on
    scatter(particleFilter.State(:,1), particleFilter.State(:,2), 50, 'b^', 'filled');  
    scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled'); 
    hold on
    title('Iterative Clustering Results');
    axis equal;
    pause(1); 
end

% plot final map result
figure;
imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
set(gca, 'YDir', 'normal');
hold on;
legendEntries = gobjects(2, 1);
for i = 1:I 
    particleFilter = detectedSourceFilters{i};
    if i==1
        legendEntries(2) = plot(particleFilter.particles(:,1), particleFilter.particles(:,2), 'y.','MarkerSize', 10);
        legendEntries(3) = scatter(particleFilter.State(:,1), particleFilter.State(:,2), 50, 'b^', 'filled');
    else
        plot(particleFilter.particles(:,1), particleFilter.particles(:,2), 'y.','MarkerSize', 10); 
        scatter(particleFilter.State(:,1), particleFilter.State(:,2), 50, 'b^', 'filled'); 
    end      
end
legendEntries(1) = scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled'); 
legendEntries(4) = scatter(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), 10, 'k', 'filled'); 
legendEntries(5) = plot(robotPoses(1:numTimeSteps,1), robotPoses(1:numTimeSteps,2), 'k--'); 

% Compare with weighted sums (WS)
% for j = 1:numel(weightSum)
%     ws = weightSum{j};
%     if j==1
%         legendEntries(6) = scatter(ws.State(:,1), ws.State(:,2), 50, 'c^', 'filled'); 
%     else
%         scatter(ws.State(:,1), ws.State(:,2), 50, 'c^', 'filled');
%     end
% end
% legend(legendEntries, 'Source ground truth','Clustered particles','Source estimate', 'Robot position','Robot trajectory','Weighted Sum','Location', 'north east');

legend(legendEntries, 'Source ground truth','Clustered particles','Source estimate', 'Robot position','Robot trajectory','Location', 'north east');
title("Final Mapping Results");
axis equal;
axis on;
axis image


