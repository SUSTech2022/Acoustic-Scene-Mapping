function detectedSourceFilters = mergeClusters(detectedSourceFilters, merge_tresh)
    % Merges detected source particle sets that are within a specified distance threshold.

    num_detected_sources = size(detectedSourceFilters, 2);
    positions = zeros(num_detected_sources, 2); % Store each estimated position

    % Get each estimated position
    for i = 1:num_detected_sources
        particleFilter = detectedSourceFilters{i};
        positions(i, :) = particleFilter.State;
    end

    % Calculate the distances between each estimate
    clusterDist = pdist2(positions, positions);

    new_detectedSourceFilters = {};

    n=1;
    for j = 1:length(detectedSourceFilters)
        if ~isempty(detectedSourceFilters{j})
            nearby_particles = find(clusterDist(j, :) < merge_tresh); % Find estimates with distances less than the threshold
            if  length(nearby_particles) > 1 % If the current estimate has nearby neighbors
                nearby_positions = positions(nearby_particles, :); % Get the positions of the nearby estimates
                merged_position = mean(nearby_positions, 1); % Calculate the merged position of the nearby estimates
                new_detectedSourceFilters{n}.State = merged_position; % Update the estimated position
                new_detectedSourceFilters{n}.particles = [];
                for k = 1:length(nearby_particles)
                    new_detectedSourceFilters{n}.particles = [new_detectedSourceFilters{n}.particles;detectedSourceFilters{nearby_particles(k)}.particles];
                end
                for k = 1:length(nearby_particles)
                    index = nearby_particles(k);
                    clusterDist(index, :) = inf; % Remove distances between the current index estimate and other estimates
                    clusterDist(:, index) = inf; % Remove distances between other estimates and the current index estimate
                    detectedSourceFilters{index} = {};
                end
            else
                new_detectedSourceFilters{n} = detectedSourceFilters{j}; % Add the current estimate to the new array        
            end
            n = n+1;
        end
    end

    detectedSourceFilters = new_detectedSourceFilters; % Update detectedSourceFilters
end