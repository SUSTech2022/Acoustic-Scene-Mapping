function [newParticleFilters,updatedObservations]  = associateObservationsMap(particleFilters, numTimeSteps, numObservations, observations, robotPoses, associatedRange)
    
    associatedRange = wrapToPi(associatedRange); % 关联范围

    % 初始化每个粒子滤波器的观测值数组  only one
    for i = 1:numel(particleFilters)
        particleFilters{i}.observations = [];
    end

    % 遍历每个时间步和每个观测区间
    for t = 1:numTimeSteps
        for j = 1:numObservations
            minDiff = associatedRange;
            maxIndex = 0;
            angle = wrapToPi(observations(t,j) + deg2rad(robotPoses(t,3)));  % 将azimuth转换到世界坐标系下，弧度制
            
            % 遍历每个粒子滤波器
            for i = 1:numel(particleFilters)
                diff = particleFilters{i}.State - robotPoses(t,1:2); % 计算聚类中心与机器人之间的位置差
                particleAngles = atan2(diff(:,2), diff(:,1));
                angleDiff = abs(angle - particleAngles); % 计算观测射线角度与粒子角度之间的差值 delta_theta       
%                 particleDistance = sqrt(sum(diff.^2, 2));% 计算粒子与机器人之间的欧氏距离
                
                % if (angleDiff <= associatedRange) && (particleDistance <= soundRange) && (angleDiff < minDiff)
                if (angleDiff < minDiff)  % 如果角度差不在关联范围内，不关联
                    minDiff = angleDiff;
                    maxIndex = i;
                end

            end
            
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
