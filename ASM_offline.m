clc
clear
close all

addpath("functions\")

%% Load experiment data

% Source Arrangement I  K = 71
% Source Arrangement II  K = 40
params = loadExperimentData('II', 'MVDR'); % Source arrangement: 'I' or 'II',  SSLmethod: 'MVDR' or 'GCC-PHAT'

robotPoses = params.robotPoses;
observations = params.observations;
image = params.image;
resolution = params.resolution;
origin = params.origin;
srcGroundTruth = params.srcGroundTruth;
numObservations = params.numObservations;
sigma = params.sigma;
associatedRange = params.associatedRange;

%% Initialization

visualize = 0; % Whether visualizng the estimate of each iteration
numTimeSteps = size(robotPoses,1); % number of discrete time steps, K
[robotPoses, srcGroundTruth] = convertCoordinates(robotPoses, srcGroundTruth, origin, resolution);

% Initialize particle filter parameters
D = 9; % 9,16,25,36 
[row, col] = initializeParticles(image, D);
numParticles = numel(row); % Number of particles equals the number of pixels found

% DBSCAN clustering parameters
epsilon = 0.1/resolution; % Distance threshold used to define neighborhoods. If the distance between two points is less than or equal to epsilon, then these two points are considered neighbors.
MinPts = numParticles*0.1; % Quantity threshold used to define core points. If an epsilon-neighborhood of a point contains at least MinPts points (including the point itself), then this point is considered a core point.

% Initialize variables
pfResults = {}; %存储每轮粒子滤波结果里的粒子，聚类前
clusterResults = {}; % 存储每轮聚类后的粒子
weightSum = {};
detectedSourceFilters = [];
updatedObservations = observations; % Initialize the updated DoA Estimates Table 
roundCount = 0; % iteration round count

%% Filtering - Clustering - Implicit Associating cycle

% Start timer
tic

while true
    roundCount = roundCount + 1;
    close all
    
    % Particle Filtering
    disp('Filtering...')
    [particleFilter,weight_sum] = particleFiltering(roundCount,sigma,numTimeSteps, numObservations, updatedObservations, robotPoses, srcGroundTruth,image,row, col,resolution);
    pfResults{roundCount}.particles = particleFilter.particles;
    pfResults{roundCount}.particleWeight = particleFilter.weights;
    pfResults{roundCount}.roundCount = roundCount;

    % DBSCAN clustering
    clusteredParticleSet = getMaxClusterFromDBSCAN(particleFilter, epsilon, MinPts); % cluster with most particles
    
    if ~isempty(clusteredParticleSet)
        clusterResults{roundCount}.particles = clusteredParticleSet{1}.particles;
        clusterResults{roundCount}.roundCount = roundCount;
    else
        roundCount = roundCount - 1;
        break;
    end

    disp(['Round ',num2str(roundCount),' finished.']);

    % Associate observations and update the DoA Estimates Table  
    [associatedParticleSet, updatedObservations] = associateObservations(clusteredParticleSet, numTimeSteps, numObservations, updatedObservations, robotPoses, associatedRange);
    
    % Add new estimate to the set
    detectedSourceFilters = [detectedSourceFilters, associatedParticleSet];
    weightSum{end+1} = weight_sum;

    % Visualize estimate of each iteration
    if visualize
        figure;
        imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
        set(gca, 'YDir', 'normal');
        hold on;
        num_newClusteredParticleSet  = size(clusteredParticleSet,2); % 1
        for i = 1:num_newClusteredParticleSet
            particleCluster = clusteredParticleSet{i}; 
            plot(particleCluster.particles(:,1), particleCluster.particles(:,2), 'y.','MarkerSize', 10,'DisplayName', 'Clustered particles');
            scatter(particleCluster.State(:,1), particleCluster.State(:,2), 50, 'b^', 'filled','DisplayName','Source estimate');
        end
        scatter(weight_sum.State(:,1), weight_sum.State(:,2), 50, 'c^', 'filled','DisplayName', 'Weighted Sum'); 
        scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled','DisplayName', 'Source ground truth'); 
        scatter(robotPoses(1:k,1), robotPoses(1:k,2), 10, 'k', 'filled','DisplayName', 'Robot position'); 
                plot(robotPoses(1:k,1), robotPoses(1:k,2), 'k--','DisplayName', 'Robot trajectory'); 
        title(sprintf("Clustering Result: Round %d", roundCount));
        legend('Location', 'north east');
        axis on;
        axis image
        hold off;
        pause(2); 
    end
   
end

% Merge similar estimates
merge_tresh = 0.5/resolution; % 0.5m
detectedSourceFilters = mergeClusters(detectedSourceFilters, merge_tresh);
weightSum = mergeClusters(weightSum, merge_tresh);

elapsedTime = toc; % End timer
disp(['The mapping took ', num2str(elapsedTime), ' seconds.']);

%% Error analysis

% OSPA distance
c = 1; % cutoff distance 1m
p = 1; %  first order
[ospa, ospa_loc,ospa_card] = calculate_OSPA_distance(detectedSourceFilters, srcGroundTruth, resolution,c,p);
[ospa_ws, ospa_loc_ws,ospa_card_ws] = calculate_OSPA_distance(weightSum, srcGroundTruth, resolution,c,p);
mapping_result = [ospa,ospa_loc,ospa_card,ospa_ws, ospa_loc_ws,ospa_card_ws]; % OSPA,loc,cardility

fprintf('OSPA distance is %.3f m.\n', ospa);
fprintf('OSPA localization error is %.3f m.\n', ospa_loc);
fprintf('OSPA cardility error is %.3f m\n', ospa_card);
   
%% Visualize ASM results

I = numel(detectedSourceFilters); % number of detected sources

% plot iterative clusters 
% colors = hsv(I+1); % skip red
% figure;
% imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
% set(gca, 'YDir', 'normal');
% hold on
% pause(0.5);
% for i = 1:I
%     particleFilter = detectedSourceFilters{i};
%     plot(particleFilter.particles(:,1), particleFilter.particles(:,2), '.', 'Color', colors(i+1,:));
%     hold on
%     scatter(particleFilter.State(:,1), particleFilter.State(:,2), 50, 'b^', 'filled');  
%     scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled'); 
%     hold on
%     title('Iterative Clustering Results');
%     axis equal;
%     pause(1); 
% end

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







