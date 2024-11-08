function [particleFilter,weight_sum] = particleFiltering(roundCount,sigma,K, numObservations, observations, robotPoses, srcGroundTruth,image,row, col,resolution)
    
    % Initialize particles and weights
    particles = [col, row]; 
    numParticles = size(particles,1);
    weights = ones(numParticles, 1)/numParticles; 

    % Parameter settings
    beta = 50; % The larger the value, the less sensitive to distance
    interval = 2*sigma;
    lamda = exp(-0.5 * (interval / sigma).^2) / (sqrt(2*pi) * sigma);
    resample_tresh = ceil(numParticles*0.1); 

    % Visualize initial state
    % figure
    % imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
    % set(gca, 'YDir', 'normal');
    % hold on
    % plot(particles(:,1), particles(:,2), 'b.','DisplayName', 'Particles'); 
    % scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled','DisplayName', 'Source ground truth'); 
    % title("Initial State");
    % legend('Location', 'north east')
    % axis on;
    % axis image
    % pause(0.5)
   
    % Particle filtering
    t = 0;
    stepCount = 0;

    while true
        stepCount = stepCount+1;
        if stepCount == K+1
            t = 0;
        end
        t = t + 1;
        particles = particles + randn(numParticles,2)*(0.01/resolution); % 上一时刻状态加高斯噪声

        % Weight update
        for i = 1:numParticles % For each particle
            angleDiff = inf;
            diff = [particles(i,1) - robotPoses(t,1), particles(i,2) - robotPoses(t,2)];
            particleAngle = atan2(diff(2), diff(1)); % Calculate the angle of the line connecting the particle and the robot in the world coordinate system, in radians
            d = norm(diff); % Calculate the Euclidean distance between the particle and the robot

            for j = 1:numObservations 
                if ~isnan(observations(t,j)) % If the observation is not NaN
                    angle = wrapToPi(observations(t,j) + deg2rad(robotPoses(t,3)));  % Convert azimuth to the world coordinate system, in radians
                    delta = angle - particleAngle;
                    delta = mod(delta + pi, 2*pi) - pi; % Standardize delta to [-pi,pi)
                    angleDiff = min(angleDiff,abs(delta)); % Store the minimum angle difference, i.e., find the closest observation
                end    
            end
            
           likelihood = max(lamda,exp(-0.5 * (angleDiff / sigma)^2) / (sqrt(2*pi) * sigma)* exp(-d/beta)); % Add penalty
           % likelihood = max(lamda,exp(-0.5 * (angleDiff / sigma)^2) / (sqrt(2*pi) * sigma)); % Without penalty: WDP
           weights(i) = weights(i) * likelihood ; % Update particle weights
        end

        % Resampling
        weights = weights / sum(weights); % Normalize weights
        Neff = 1 / sum(weights.^2); % Calculate effective number of particles
        if Neff < resample_tresh 
            % disp([num2str(stepCount), 'resampling...'])

            % Plot before resample
            % figure
            % imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
            % set(gca, 'YDir', 'normal');
            % hold on
            % plot(particles(:,1), particles(:,2), 'b.','DisplayName', 'Particles'); 
            % scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled','DisplayName', 'Source ground truth');
            % title("Before Resampling");
            % legend('Location', 'north east')
            % axis on;
            % axis image
            % waitfor(gcf) % Wait for the current figure window to close before continuing to execute subsequent code

            % Low-variance resampling
            indices = zeros(1, numParticles);
            index = 1;
            u = (rand() + (0:(numParticles-1))) / numParticles;
            c = weights(1);
            for j = 1:numParticles
                while u(j) > c
                    index = index + 1;
                    c = c + weights(index);
                end
                    indices(j) = index;
            end
            
            particles = particles(indices,:);
            weights = ones(numParticles, 1) / numParticles;  % Reassign equal weights

            % Plot after resample
            % figure
            % imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
            % set(gca, 'YDir', 'normal');
            % hold on
            % plot(particles(:,1), particles(:,2), 'b.','DisplayName', 'Particles'); % Plot particles on the image
            % scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled','DisplayName', 'Source ground truth'); % Plot the true sound source position in red pentagram
            % title("After resampling");
            % legend('Location', 'north east')
            % axis on;
            % axis image
            % waitfor(gcf) % Wait for the current figure window to close before continuing to execute subsequent code
        end

            % Compare the effect of traversing once
            % if stepCount==K 
            %     clf
            %     imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
            %     set(gca, 'YDir', 'normal');
            %     hold on;
            %     color = 'mgcyrmgcyr';
            %     plot(particles(:,1), particles(:,2), 'b.','MarkerSize', 10,'DisplayName', 'Particles'); % Plot particles
            %     scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled','DisplayName', 'Source ground truth'); % Plot the true sound source position in red pentagram
            %     scatter(robotPoses(1:t,1), robotPoses(1:t,2), 10, 'k', 'filled','DisplayName', 'Robot position'); % Plot robot trajectory points in black solid circle
            %     plot(robotPoses(1:t,1), robotPoses(1:t,2), 'k--','DisplayName', 'Robot trajectory'); % Connect the points with a dashed line
            %     title(sprintf("Filtering Result with stepCount = K: Round %d", roundCount));
            %     legend('Location', 'north east')
            %     axis equal;
            %     set(gca, 'YDir', 'normal');
            %     axis on;
            %     axis image
            %     hold off;
            %     pause(2); 
            % end

        % Determine whether to exit
        covariances = max(eig(cov(particles)))*resolution*resolution; 
        if covariances < 0.1 || stepCount == 2*K 
            weight_sum.State = sum(particles .* weights, 1); 
            weight_sum.particles = weight_sum.State; % For function call compatibility, no practical meaning
            disp(['break, stepCount = ',num2str(stepCount)])
            break
        end
        
    end
    
    % Visualize particle filter results
    % figure
    % clf
    % imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
    % set(gca, 'YDir', 'normal');
    % hold on;
    % color = 'mgcyrmgcyr';
    % plot(particles(:,1), particles(:,2), 'b.','MarkerSize', 10,'DisplayName', 'Particles'); 
    % scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled','DisplayName', 'Source ground truth'); 
    % scatter(robotPoses(1:t,1), robotPoses(1:t,2), 10, 'k', 'filled','DisplayName', 'Robot position'); % Robot trajectory points in black solid circles
    % plot(robotPoses(1:t,1), robotPoses(1:t,2), 'k--','DisplayName', 'Robot trajectory'); % Connect the points with a dashed line
    % title(sprintf("Filtering Result: Round %d", roundCount));
    % legend('Location', 'north east')
    % axis equal;
    % set(gca, 'YDir', 'normal');
    % axis on;
    % axis image
    % hold off;
    % pause(2); 
    

%% Return particle filter results
    
    particleFilter.particles = particles;
    particleFilter.weights = weights;

end
