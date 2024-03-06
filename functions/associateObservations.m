function [newParticleFilters,updatedObservations]  = associateObservations(particleFilters, numTimeSteps, numSources, observations, robotPoses, threshold, associatedRange, soundRange)
    
    angleNoise = wrapToPi(deg2rad(associatedRange)); % 关联范围，仿真：associatedRange=5

    % 初始化每个粒子滤波器的观测值数组
    for i = 1:numel(particleFilters)
        particleFilters{i}.observations = [];
    end

    % 遍历每个时间步和每个观测区间
    for t = 1:numTimeSteps
        disp(['numTimeSteps =',num2str(t)]);
        for j = 1:numSources
            disp(['numObservation =',num2str(j)]);
            % 初始化最大百分比和对应的滤波器索引
            maxPercentage = 0;
            maxIndex = 0;

            angle = wrapToPi(observations(t,j) + deg2rad(robotPoses(t,3)));  % 将azimuth转换到世界坐标系下

            % 遍历每个粒子滤波器
            for i = 1:numel(particleFilters)
                % 计算落在观测区间内粒子数的百分比
                diff = particleFilters{i}.particles - repmat(robotPoses(t,1:2), size(particleFilters{i}.particles, 1), 1);
                particleAngles = atan2(diff(:,2), diff(:,1));

                angleDiff = abs(angle - particleAngles); % 计算观测射线角度与粒子角度之间的差值 delta_theta       
                particleDistance = sqrt(sum(diff.^2, 2));% 计算粒子与机器人之间的欧氏距离

%                 percentage = sum((particleAngles > angle-angleNoise) & (particleAngles < angle+angleNoise)) / size(particleFilters{i}.particles, 1);  
                percentage = sum((angleDiff <= angleNoise) & (particleDistance <= soundRange)) / size(particleFilters{i}.particles, 1);
                disp(['percentage =',num2str(percentage)]);

                % 如果百分比超过阈值且百分比最高，则更新最大百分比和对应的滤波器索引
                if percentage > threshold && percentage > maxPercentage
                    maxPercentage = percentage;
                    maxIndex = i;
                end
            end
            disp(['maxIndex = ',num2str(maxIndex),' maxPercentage =',num2str(maxPercentage)]);

            
            % 如果找到了对应的滤波器，则将观测值存储到该滤波器对应的观测值数组内，并将原观测值矩阵的对应元素记为NaN
            if maxIndex > 0
                particleFilters{maxIndex}.observations = [particleFilters{maxIndex}.observations; observations(t,j)];
                observations(t,j) = NaN;
            end
        end
    end
    updatedObservations = observations;
    newParticleFilters = particleFilters;
end
