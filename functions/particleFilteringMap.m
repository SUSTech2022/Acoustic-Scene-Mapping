function [particleFilter,weight_sum] = particleFilteringMap(roundCount,sigma,K, numObservations, observations, robotPoses, srcGroundTruth,image,row, col,resolution,visualize)
    

    % 初始化粒子和权重
    particles = [col, row]; % 在找到的像素上撒粒子
    numParticles = size(particles,1);
    weights = ones(numParticles, 1)/numParticles; % 初始化权重

    % 参数设置
    beta = 50; % 越大，对距离越不敏感
    interval = 2*sigma;
    lamda = exp(-0.5 * (interval / sigma).^2) / (sqrt(2*pi) * sigma);
    resample_tresh = ceil(numParticles*0.1);

    if visualize
        % 可视化初始状态
        figure
        imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
        set(gca, 'YDir', 'normal');
        hold on
        plot(particles(:,1), particles(:,2), 'b.','DisplayName', 'Particles'); % 在图像上画出粒子
        scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled','DisplayName', 'Source ground truth'); % 绘制声源位置真值 红色五角星
        title("Initial State");
        legend('Location', 'north east')
        axis on;
        axis image
        pause(0.5)
    end

    % 粒子滤波
    t = 0;
    stepCount = 0;
    while true
        stepCount = stepCount+1;
%     for stepCount = 1:2*K
        if stepCount == K+1
            t = 0;
        end
        t = t + 1;
        % 状态预测
        particles = particles + randn(numParticles,2)*(0.01/resolution); % 上一时刻状态加高斯噪声

        % 权重更新
        for i = 1:numParticles % 对每一个粒子
            angleDiff = inf;
            diff = [particles(i,1) - robotPoses(t,1), particles(i,2) - robotPoses(t,2)];
            particleAngle = atan2(diff(2), diff(1)); % 计算粒子与机器人连线在世界坐标系下的角度，弧度制
            particleDistance = norm(diff);% 计算粒子与机器人之间的欧氏距离 d
            d = abs(particleDistance-0);
            for j = 1:numObservations 
                if ~isnan(observations(t,j)) % 如果观测值不是NaN
                    angle = wrapToPi(observations(t,j) + deg2rad(robotPoses(t,3)));  % 将azimuth转换到世界坐标系下，弧度制
                    angleDiff = min(angleDiff,abs(angle - particleAngle)); % 存储最小角度差 即找到最近的观测
                end   
            end                   
           likelihood = max(lamda,exp(-0.5 * (angleDiff / sigma)^2) / (sqrt(2*pi) * sigma)* exp(-d/beta));
%            likelihood = max(lamda,exp(-0.5 * (angleDiff / sigma)^2) / (sqrt(2*pi) * sigma)); % 不加penalty
           weights(i) = weights(i) * likelihood ; % 更新粒子权重
        end

        weights = weights / sum(weights); % 权重归一化
        Neff = 1 / sum(weights.^2); % 计算有效粒子数
        
        if Neff < resample_tresh %|| t == K
%             disp(num2str(stepCount))
            % 低方差重采样
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
        end

%         if stepCount==K
%             clf
%             imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
%             set(gca, 'YDir', 'normal');
%             hold on;
%             color = 'mgcyrmgcyr';
%             plot(particles(:,1), particles(:,2), 'b.','MarkerSize', 10,'DisplayName', 'Particles'); % 画出粒子
%             scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled','DisplayName', 'Source ground truth'); % 绘制声源位置真值 红色五角星
%             scatter(robotPoses(1:t,1), robotPoses(1:t,2), 10, 'k', 'filled','DisplayName', 'Robot position'); % 画出机器人轨迹点 黑色实心圆
%             plot(robotPoses(1:t,1), robotPoses(1:t,2), 'k--','DisplayName', 'Robot trajectory'); % 使用虚线连接点
%             title(sprintf("Filtering Result with stepCount = K: Round %d", roundCount));
%             legend('Location', 'north east')
%             axis equal;
%             set(gca, 'YDir', 'normal');
%             axis on;
%             axis image
%             hold off;
%             pause(2); % 暂停0.01秒
%         end


        covariances = max(eig(cov(particles)))*resolution*resolution; % 判断是否提前退出

        if covariances < 0.1 || stepCount == 2*K
            weight_sum.State = sum(particles .* weights, 1); 
            weight_sum.particles = weight_sum.State; % 为了函数调用兼容，无实际含义
            disp(['break, stepCount = ',num2str(stepCount)])
            break
        end
        
    end
    
    
        if visualize
        % 可视化采样结果
            figure
            clf
            imshow(image, 'XData', [0, size(image,2)], 'YData', [0, size(image,1)],'InitialMagnification', 200);
            set(gca, 'YDir', 'normal');
            hold on;
            color = 'mgcyrmgcyr';
            % theta = deg2rad(robotPoses(t,3)); % 机器人yaw角度， 弧度制
            % disp(['t=',num2str(t),'theta = ',num2str(theta)]);
            % quiver(robotPoses(t, 1), robotPoses(t, 2), cos(theta), sin(theta), 10,'r'); % 红色箭头，表示机器人方向
%             for i = 1:numObservations 
%                 angle = wrapToPi(observations(t,i)+ deg2rad(robotPoses(t,3)));  
%                 quiver(robotPoses(t,1), robotPoses(t,2), cos(angle), sin(angle), 10,'color',color(i)); % 画出观测射线
%                 quiver(robotPoses(t,1), robotPoses(t,2), cos(angle-interval), sin(angle-interval),10, 'color',color(i), 'LineStyle','--'); % 画出观测射线区间的一端
%                 quiver(robotPoses(t,1), robotPoses(t,2), cos(angle+interval), sin(angle+interval),10, 'color',color(i), 'LineStyle','--'); % 画出观测射线区间的另一端
%             end
            plot(particles(:,1), particles(:,2), 'b.','MarkerSize', 10,'DisplayName', 'Particles'); % 画出粒子
            scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 100, 'rp', 'filled','DisplayName', 'Source ground truth'); % 绘制声源位置真值 红色五角星
            scatter(robotPoses(1:t,1), robotPoses(1:t,2), 10, 'k', 'filled','DisplayName', 'Robot position'); % 画出机器人轨迹点 黑色实心圆
            plot(robotPoses(1:t,1), robotPoses(1:t,2), 'k--','DisplayName', 'Robot trajectory'); % 使用虚线连接点
            title(sprintf("Filtering Result: Round %d", roundCount));
            legend('Location', 'north east')
            axis equal;
            set(gca, 'YDir', 'normal');
            axis on;
            axis image
            hold off;
            pause(2); % 暂停0.01秒


        end

    % 
    %     % covariances = max(eig(cov(particles)))*resolution*resolution; % 计算粒子位置的协方差，坐标单位转换
    %     mean_distance = mean(pdist(particles)) * resolution;
    %     % std_deviation = max(std(particles(:,1)) * resolution,std(particles(:,2)) * resolution)
    %     if mean_distance < 0.1
    %         break
    %     end
    

    %% 第二圈
    
    % beta = 40; % 越大，对距离越不敏感
    % resampleThreshold = 1;
    % % sigma = deg2rad(5); % 观测标准差
    % % interval = 3*sigma;
    % % lamda = exp(-0.5 * (interval / sigma).^2) / (sqrt(2*pi) * sigma);
    % lamda = 0.5;
    % % 粒子滤波
    % for t = startStep:stepsize:K %K
    % 
    %     covariances = max(eig(cov(particles)))*resolution*resolution; % 计算粒子位置的协方差，坐标单位转换
    %     if covariances < 0.05
    %         break
    %     end
    % 
    %     % 状态预测
    %     particles = particles + randn(numParticles,2)*(0.01/resolution); % 上一时刻状态加高斯噪声
    % 
    %     % 权重更新：对落在区间内的粒子按似然更新权重 
    %     for i = 1:numParticles
    %         max_likelihood = lamda; 
    %         for j = 1:numObservations 
    %             if ~isnan(observations(t,j)) % 如果观测值不是NaN
    %                 angle = wrapToPi(observations(t,j) + deg2rad(robotPoses(t,3)));  % 将azimuth转换到世界坐标系下，弧度制
    %                 diff = [particles(i,1) - robotPoses(t,1), particles(i,2) - robotPoses(t,2)];
    %                 particleAngle = atan2(diff(2), diff(1)); % 计算粒子与机器人连线在世界坐标系下的角度，弧度制
    %                 angleDiff = abs(angle - particleAngle); % 计算观测射线角度与粒子角度之间的差值 delta_theta       
    %                 particleDistance = norm(diff);% 计算粒子与机器人之间的欧氏距离
    % 
    %                 % if angleDiff <= angleInterval && particleDistance <= soundRange % 若粒子在观测射线区间内
    %                 % if angleDiff <= angleInterval % 若粒子在观测射线区间内
    %                 % if particleDistance <= soundRange     
    %                     likelihood = exp(-0.5 * (angleDiff / sigma)^2) / (sqrt(2*pi) * sigma)* exp(-particleDistance/beta); % 观测标准差 
    %                     % likelihood = exp(-0.5 * (angleDiff / sigma)^2) / (sqrt(2*pi) * sigma); 
    %                     if likelihood > max_likelihood
    %                         max_likelihood = likelihood;
    %                     end
    %                 % end
    %             end
    %         end
    %         weights(i) = weights(i) * max_likelihood; % 更新粒子权重
    %     end
    % 
    %      % 按有效粒子数判断是否重采样
    %     weights = weights / sum(weights); % 归一化权重
    %     Neff = 1 / sum(weights.^2); % 计算有效粒子数
    % 
    %     % 随机重采样
    %     % if Neff < resampleThreshold * numParticles
    %     %     indices = randsample(1:numParticles, numParticles, true, weights);             
    %     %     particles = particles(indices,:);
    %     %     weights = ones(numParticles, 1)/numParticles; % 重新分配权重
    %     % end
    % 
    %     % if Neff < resampleThreshold * numParticles   % 系统性重采样
    %     %     cdf = cumsum(weights); % Compute cumulative distribution function
    %     %     u = rand(1) / numParticles; % Generate initial sample value
    %     %     newIndices = zeros(numParticles, 1);
    %     % 
    %     %     j = 1;
    %     %     for i = 1:numParticles
    %     %         while u > cdf(j)
    %     %             j = j + 1;
    %     %         end
    %     %         newIndices(i) = j;
    %     %         u = u + 1 / numParticles;
    %     %     end
    %     % 
    %     %     particles = particles(newIndices, :);
    %     %     weights = ones(numParticles, 1) / numParticles; % Reassign equal weights
    %     % end
    % 
    % 
    %    if Neff < resampleThreshold * numParticles
    %    % if t == K
    %         % Step 1: Compute cumulative sum of weights
    %         cumulative_weights = cumsum(weights);
    % 
    %         % Step 2: Initialize variables
    %         indices = zeros(numParticles, 1);
    %         index = 1;
    %         u = (rand() + (0:numParticles-1)) / numParticles;
    %         c = weights(1);
    % 
    %         % Step 3: Perform resampling
    %         for j = 1:numParticles
    %             while u(j) > c
    %                 index = index + 1;
    %                 c = c + weights(index);
    %             end
    %             indices(j) = index;
    %         end
    % 
    %         particles = particles(indices, :);
    %         weights = ones(numParticles, 1) / numParticles; % Reassign equal weights
    %     end
    % 
    %     if visualize
    %     % 可视化滤波过程
    %         imshow(image, 'InitialMagnification', 1000);
    %         hold on;
    %         scatter(robotPoses(1:t,1), robotPoses(1:t,2), 10, 'k', 'filled'); % 画出机器人轨迹点 黑色实心圆
    %         plot(robotPoses(1:t,1), robotPoses(1:t,2), 'k--'); % 使用虚线连接点
    %         color = 'mgcyrmgcyr';
    %         for i = 1:numObservations 
    %             angle = wrapToPi(observations(t,i)+ deg2rad(robotPoses(t,3)));
    %             quiver(robotPoses(t,1), robotPoses(t,2), cos(angle), sin(angle), 10,'color',color(i)); % 画出观测射线
    %             % quiver(robotPoses(t,1), robotPoses(t,2), cos(angle-angleInterval), sin(angle-angleInterval),10, 'color',color(i), 'LineStyle','--'); % 画出观测射线区间的一端
    %             % quiver(robotPoses(t,1), robotPoses(t,2), cos(angle+angleInterval), sin(angle+angleInterval),10, 'color',color(i), 'LineStyle','--'); % 画出观测射线区间的另一端
    %         end
    %         plot(particles(:,1), particles(:,2), 'b.'); % 画出粒子
    %         scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 50, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
    %         title("Filtering... ");
    %         axis equal;
    %         axis([0 size(image,2) 0 size(image,1)]);
    %         hold off;
    %         pause(0.001); % 暂停0.01秒
    %     end
    % end

    %%
    % 返回粒子滤波器
    particleFilter.particles = particles;
    particleFilter.weights = weights;
end
