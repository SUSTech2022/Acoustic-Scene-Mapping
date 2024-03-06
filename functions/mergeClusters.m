function detectedSourceFilters = mergeClusters(detectedSourceFilters, merge_tresh)
    num_detected_sources = size(detectedSourceFilters, 2);
    positions = zeros(num_detected_sources, 2); % 存储每个估计的位置

    % 获取每个估计的位置
    for i = 1:num_detected_sources
        particleFilter = detectedSourceFilters{i};
        positions(i, :) = particleFilter.State;
    end

    % 计算各个估计之间的距离
    clusterDist = pdist2(positions, positions);

    new_detectedSourceFilters = {};

    n=1;
    for j = 1:length(detectedSourceFilters)
        if ~isempty(detectedSourceFilters{j})
            nearby_particles = find(clusterDist(j, :) < merge_tresh); % 查找距离小于 tresh 的估计
            if  length(nearby_particles) > 1 % 如果当前估计有靠近邻居
                nearby_positions = positions(nearby_particles, :); % 获取靠近的估计的位置
                merged_position = mean(nearby_positions, 1); % 计算靠近的估计的估计位置
                new_detectedSourceFilters{n}.State = merged_position; % 更新估计位置
                new_detectedSourceFilters{n}.particles = [];
                for k = 1:length(nearby_particles)
                    new_detectedSourceFilters{n}.particles = [new_detectedSourceFilters{n}.particles;detectedSourceFilters{nearby_particles(k)}.particles];
                end
                for k = 1:length(nearby_particles)
                    index = nearby_particles(k);
                    clusterDist(index, :) = inf; % 删除当前索引的估计与其他估计之间的距离
                    clusterDist(:, index) = inf; % 删除其他估计与当前索引的估计之间的距离
                    detectedSourceFilters{index} = {};
                end
            else
                new_detectedSourceFilters{n} = detectedSourceFilters{j}; % 将当前估计添加到新的数组中        
            end
            n = n+1;
        end
    end

    detectedSourceFilters = new_detectedSourceFilters; % 更新 detectedSourceFilters
end
