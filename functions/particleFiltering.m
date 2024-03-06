function particleFilter = particleFiltering(numParticles, angleNoise, numTimeSteps, numObservations, observations, robotPoses, srcGroundTruth,x_range,y_range,random_indices)
    
    resampleThreshold = 0.5; 
    lamda = 0.2*(exp(-0.5 * (angleNoise / angleNoise)^2) / (sqrt(2*pi) * angleNoise));
    particles = random_indices/100;


%     % 计算范围宽度
%     x_width = x_range(2) - x_range(1);
%     y_length = y_range(2) - y_range(1);
%     % 初始化粒子和权重
%     particles = zeros(numParticles, 2); % 初始化粒子
% 
%     % 在指定范围内随机分布粒子
%     particles(:,1) = x_range(1) + x_width * rand(numParticles, 1); % 在x方向上随机分布
%     particles(:,2) = y_range(1) + y_length * rand(numParticles, 1); % 在y方向上随机分布

    
    
    weights = ones(numParticles, 1)/numParticles; % 初始化权重

    % 创建一个新的图形窗口
    figure('units','normalized','outerposition',[0 0 1 1]);

    % 可视化初始状态
    t = 0;
    scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 300, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
    hold on
    plot(particles(:,1), particles(:,2), 'b.'); % 画出粒子;
    title(["Filtering... numTimeSteps =",num2str(t)]);
    axis equal;
    axis([x_range(1) x_range(2) y_range(1) y_range(2)]);
    hold off;
    pause(1); % 暂停

    % 粒子滤波 正序
    for t = 1:numTimeSteps %numTimeSteps
        % 状态预测
        particles = particles + randn(numParticles,2)*0.01; % 上一时刻状态加高斯噪声

          % 权重更新：对落在区间内的粒子按似然更新权重 
        for i = 1:numParticles
            max_likelihood = lamda; % 初始化最大似然函数值 (0.5最好，别改了)如果粒子不在射线区间内或观测值全为NaN，权重变小但不能是0 

            for j = 1:numObservations 
                if ~isnan(observations(t,j)) % 如果观测值不是NaN
                    angle = wrapToPi(observations(t,j) + deg2rad(robotPoses(t,3)));  % 将azimuth转换到世界坐标系下，弧度制
                    diff = particles(i,:) - robotPoses(t,1:2);
                    particleAngle = atan2(diff(2), diff(1)); % 计算粒子与机器人连线在世界坐标系下的角度，弧度制

                    angleDiff = abs(angle - particleAngle); % 计算观测射线角度与粒子角度之间的差值 delta_theta
                    
                    if angleDiff <= angleNoise % 若粒子在观测射线区间内
                      
                        % 根据高斯噪声模型计算似然函数
                        likelihood = exp(-0.5 * (angleDiff / angleNoise)^2) / (sqrt(2*pi) * angleNoise);
                        
                        if likelihood > max_likelihood
                            max_likelihood = likelihood;
                        end

                    end
                end
            end
            weights(i) = weights(i) * max_likelihood; % 更新粒子权重
        end

%         if t<numTimeSteps % 最后权重需要保留
%             % 重采样
%             indices = randsample(1:numParticles, numParticles, true, weights); 
%             particles = particles(indices,:);
%             weights = ones(numParticles, 1)/numParticles;
%         end

        % 按有效粒子数判断是否重采样
        if t < numTimeSteps  % 最后一次就别归一化了，不然看不见最终概率分布
            % 归一化权重
            weights = weights / sum(weights);
        
            % 计算有效粒子数
            Neff = 1 / sum(weights.^2);
        
            % 如果有效粒子数低于阈值，则进行重采样
            if Neff < resampleThreshold * numParticles
                indices = randsample(1:numParticles, numParticles, true, weights);             
                particles = particles(indices,:);
                weights = ones(numParticles, 1)/numParticles; % 重新分配权重
            end
        end

        % 可视化
        clf; % 清除当前图形
        hold on;
        scatter(robotPoses(1:t,1), robotPoses(1:t,2), 50, 'k', 'filled'); % 画出机器人轨迹点 黑色实心圆
        plot(robotPoses(1:t,1), robotPoses(1:t,2), 'k--'); % 使用虚线连接点
        hold on
        theta = deg2rad(robotPoses(t,3)); % 机器人yaw角度， 弧度制
%         quiver(robotPoses(t, 1), robotPoses(t, 2), cos(theta), sin(theta), 'r'); % 红色箭头，表示机器人方向
        hold on
        color = 'mgcymgcy';
        for i = 1:numObservations 
            angle = wrapToPi(observations(t,i)+ deg2rad(robotPoses(t,3)));
            quiver(robotPoses(t,1), robotPoses(t,2), cos(angle), sin(angle), 'color',color(i)); % 画出观测射线
            quiver(robotPoses(t,1), robotPoses(t,2), cos(angle-angleNoise), sin(angle-angleNoise), 'color',color(i), 'LineStyle','--'); % 画出观测射线区间的一端
            quiver(robotPoses(t,1), robotPoses(t,2), cos(angle+angleNoise), sin(angle+angleNoise), 'color',color(i), 'LineStyle','--'); % 画出观测射线区间的另一端
        end
        plot(particles(:,1), particles(:,2), 'b.'); % 画出粒子
        scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 300, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
        title(["Filtering... numTimeSteps =",num2str(t)]);
        axis equal;
        axis([x_range(1) x_range(2) y_range(1) y_range(2)]);
        hold off;
        pause(0.01); % 暂停0.01秒
    end

    % 粒子滤波 倒序
    for t = numTimeSteps:-1:1 %numTimeSteps
        % 状态预测
        particles = particles + randn(numParticles,2)*0.01; % 上一时刻状态加高斯噪声

          % 权重更新：对落在区间内的粒子按似然更新权重 
        for i = 1:numParticles
            max_likelihood = lamda; % 初始化最大似然函数值 (0.5最好，别改了)如果粒子不在射线区间内或观测值全为NaN，权重变小但不能是0 

            for j = 1:numObservations 
                if ~isnan(observations(t,j)) % 如果观测值不是NaN
                    angle = wrapToPi(observations(t,j) + deg2rad(robotPoses(t,3)));  % 将azimuth转换到世界坐标系下，弧度制
                    diff = particles(i,:) - robotPoses(t,1:2);
                    particleAngle = atan2(diff(2), diff(1)); % 计算粒子与机器人连线在世界坐标系下的角度，弧度制

                    angleDiff = abs(angle - particleAngle); % 计算观测射线角度与粒子角度之间的差值 delta_theta
                    
                    if angleDiff <= angleNoise % 若粒子在观测射线区间内
                      
                        % 根据高斯噪声模型计算似然函数
                        likelihood = exp(-0.5 * (angleDiff / angleNoise)^2) / (sqrt(2*pi) * angleNoise);
                        
                        if likelihood > max_likelihood
                            max_likelihood = likelihood;
                        end

                    end
                end
            end
            weights(i) = weights(i) * max_likelihood; % 更新粒子权重
        end

        % 按有效粒子数判断是否重采样
        if t < numTimeSteps  % 最后一次就别归一化了，不然看不见最终概率分布
            % 归一化权重
            weights = weights / sum(weights);
        
            % 计算有效粒子数
            Neff = 1 / sum(weights.^2);
        
            % 如果有效粒子数低于阈值，则进行重采样
            if Neff < resampleThreshold * numParticles
                indices = randsample(1:numParticles, numParticles, true, weights);             
                particles = particles(indices,:);
                weights = ones(numParticles, 1)/numParticles; % 重新分配权重
            end
        end

        % 可视化
        clf; % 清除当前图形
        hold on;
        scatter(robotPoses(1:t,1), robotPoses(1:t,2), 50, 'k', 'filled'); % 画出机器人轨迹点 黑色实心圆
        plot(robotPoses(1:t,1), robotPoses(1:t,2), 'k--'); % 使用虚线连接点
%         scatter(robotPoses(t,1), robotPoses(t,2), 100, 'k', 'filled'); % 画出机器人位置 黑色实心圆
        color = 'mgcyrmgcyr';
        for i = 1:numObservations 
            angle = wrapToPi(observations(t,i)+ deg2rad(robotPoses(t,3)));
            quiver(robotPoses(t,1), robotPoses(t,2), cos(angle), sin(angle), 'color',color(i)); % 画出观测射线
            quiver(robotPoses(t,1), robotPoses(t,2), cos(angle-angleNoise), sin(angle-angleNoise), 'color',color(i), 'LineStyle','--'); % 画出观测射线区间的一端
            quiver(robotPoses(t,1), robotPoses(t,2), cos(angle+angleNoise), sin(angle+angleNoise), 'color',color(i), 'LineStyle','--'); % 画出观测射线区间的另一端
        end
        plot(particles(:,1), particles(:,2), 'b.'); % 画出粒子
        scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 300, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
        title(["Filtering... numTimeSteps =",num2str(t)]);
        axis equal;
        axis([x_range(1) x_range(2) y_range(1) y_range(2)]);
        hold off;
        pause(0.01); % 暂停0.01秒
    end

    % 返回粒子滤波器
    particleFilter.particles = particles;
    particleFilter.weights = weights;
end
