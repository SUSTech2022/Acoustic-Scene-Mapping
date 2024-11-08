function [associatedParticleSet,updatedObservations]  = associateObservations(clusteredParticleSet, numTimeSteps, numObservations, observations, robotPoses, associatedRange)
    
    associatedRange = wrapToPi(associatedRange); % Association range

    % Initialize the observations array for each particle filter, only one
    for i = 1:numel(clusteredParticleSet)
        clusteredParticleSet{i}.observations = [];
    end

    % Iterate over each time step and each observation interval
    for t = 1:numTimeSteps
        for j = 1:numObservations
            minDiff = associatedRange;
            maxIndex = 0;
            angle = wrapToPi(observations(t,j) + deg2rad(robotPoses(t,3)));  % Convert azimuth to the world coordinate system, in radians
            
            % Iterate over each particle filter
            for i = 1:numel(clusteredParticleSet)
                diff = clusteredParticleSet{i}.State - robotPoses(t,1:2); % Calculate the position difference between the cluster center and the robot
                particleAngles = atan2(diff(:,2), diff(:,1));
                angleDiff = abs(angle - particleAngles); % Calculate the difference between the observation ray angle and the particle angle delta_theta       
%                 particleDistance = sqrt(sum(diff.^2, 2));% Calculate the Euclidean distance between the particle and the robot
                
                % If the angle difference is not within the association range, do not associate
                if (angleDiff < minDiff)
                    minDiff = angleDiff;
                    maxIndex = i;
                end

            end
            
            % If a corresponding filter is found, store the observation in the corresponding filter's observations array and mark the corresponding element in the original observation matrix as NaN
            if maxIndex > 0
                clusteredParticleSet{maxIndex}.observations = [clusteredParticleSet{maxIndex}.observations; observations(t,j)];
                observations(t,j) = NaN;
            end
        end
    end
    updatedObservations = observations;
    associatedParticleSet = clusteredParticleSet;
end