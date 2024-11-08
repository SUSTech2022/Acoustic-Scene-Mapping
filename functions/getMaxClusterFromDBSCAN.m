function clusteredParticleSet = getMaxClusterFromDBSCAN(particleFilter, epsilon, MinPts) 

    particles = [];
    for i = 1:size(particleFilter,1)
        particles = [particles; particleFilter(i).particles];
    end

    % Run DBSCAN
    labels = DBSCAN(particles, epsilon, MinPts);

    % Get the number of clusters
    numClusters = max(labels);
    % disp(['total cluster number = ',num2str(numClusters)]);

    % Initialize an empty array of particle filters
    clusteredParticleSet = {};

    % Initialize variables
    maxParticleCount = 0;
    maxParticleCluster = [];

    % Iterate through clusters
    for i = 1:numClusters
        % Get particles for this cluster
        clusterParticles = particles(labels == i, :);

        % Calculate the number of particles
        numParticles = size(clusterParticles, 1);

        % If the number of particles in this cluster is greater than the previously recorded maximum
        if numParticles > maxParticleCount
            % Update the maximum particle count and corresponding cluster
            maxParticleCount = numParticles;
            maxParticleCluster = clusterParticles;
        end
    end

    % Create a new particle filter
    newparticleFilter = struct;
    newparticleFilter.particles = maxParticleCluster;

    if maxParticleCount > 0    
        % Calculate the mean value of particle coordinates
        meanX = mean(newparticleFilter.particles(:, 1));
        meanY = mean(newparticleFilter.particles(:, 2));

        % Store the mean values
        newparticleFilter.State = [meanX, meanY];

        % Add this particle filter to the array
        clusteredParticleSet{end+1} = newparticleFilter;
    end

end
