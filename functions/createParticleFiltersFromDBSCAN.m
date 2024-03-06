function particleFilters = createParticleFiltersFromDBSCAN(particleFilter, epsilon, MinPts, resolution)
    
    particles = [];
    for i = 1:size(particleFilter,1)
        particles = [particles; particleFilter(i).particles];
    end

    % 运行DBSCAN
    labels = DBSCAN(particles, epsilon, MinPts);

    % 获取聚类的数量
    numClusters = max(labels);
    % disp(['total cluster number = ',num2str(numClusters)]);

    % 初始化一个空的粒子滤波器数组
    particleFilters = {};

    % 对于每个聚类，创建一个新的粒子滤波器
    i = 0;
    count = 0;

%     while i < numClusters  
%         i = i+1;
%         % 获取这个聚类的粒子
%         clusterParticles = particles(labels == i, :);
%     
%         % covariances = max(eig(cov(clusterParticles)))*resolution*resolution; % 计算粒子位置的协方差，坐标单位转换
%         % 如果这个聚类的粒子数量小于总粒子数1/3，或粒子状态的xy某一方向最大协方差大于1，则跳过这个聚类
%         % if size(clusterParticles, 1) < size(particles,1)*(1/5) || covariances > 1 %0.2
%         %     disp(['covariances = ',num2str(covariances),'>',num2str(0.5)]);
%         %     disp(['or size(clusterParticles, 1)=',num2str(size(clusterParticles, 1)),'<',num2str(size(particles,1)*(1/3))]);
%         %     continue;
%         % end
%     
%         % 创建一个新的粒子滤波器
%         newparticleFilter = struct;
%         newparticleFilter.particles = clusterParticles;
%     
%         % 计算粒子坐标的平均值
%         meanX = mean(newparticleFilter.particles(:, 1));
%         meanY = mean(newparticleFilter.particles(:, 2));
%     
%         % 存储平均值
%         newparticleFilter.mean = [meanX, meanY];
%     
%         % 将这个粒子滤波器添加到数组中
%         particleFilters{end+1} = newparticleFilter;
%         count = count+1;
%     end
%     disp(['available cluster number = ',num2str(count)]);

    % 初始化变量
    maxParticleCount = 0;
    maxParticleCluster = [];

    % 遍历聚类
    for i = 1:numClusters
        % 获取这个聚类的粒子
        clusterParticles = particles(labels == i, :);

        % 计算粒子数量
        numParticles = size(clusterParticles, 1);

        % 如果这个聚类的粒子数量大于之前记录的最大数量
        if numParticles > maxParticleCount
            % 更新最大粒子数量和对应的聚类
            maxParticleCount = numParticles;
            maxParticleCluster = clusterParticles;
        end
    end

    % 创建一个新的粒子滤波器
    newparticleFilter = struct;
    newparticleFilter.particles = maxParticleCluster;

    if maxParticleCount > 0    
        % 计算粒子坐标的平均值
        meanX = mean(newparticleFilter.particles(:, 1));
        meanY = mean(newparticleFilter.particles(:, 2));

        % 存储平均值
        newparticleFilter.State = [meanX, meanY];

        % 将这个粒子滤波器添加到数组中
        particleFilters{end+1} = newparticleFilter;
        count = count + 1;
    end


end
