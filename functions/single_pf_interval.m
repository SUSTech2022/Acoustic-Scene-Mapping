clear
clc

% Load the data from the .mat file

%% 3 source 预设轨迹
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-3_source\results\data\azimuth_output_3source_53.mat");

%% 5 source 预设轨迹
data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_source_fixed_path\results\data\azimuth_output_5source_53.mat");

% Extract the robotPos and DOA data
robotPoses = data.outputStruct.robotPos;
robotPoses(:,3) = zeros(size(robotPoses,1),1); % 假设yaw全0
observations = deg2rad(data.outputStruct.meanAzEst);
srcGroundTruth = data.outputStruct.srcGroundTruth;

% 轨迹倒序
% observations = flipud(observations);
% robotPoses = flipud(robotPoses);

numSources = size(observations,2); % 声源数量
numTimeSteps = size(robotPoses,1); % 时间步长

%% 粒子滤波器实现
% 初始化参数
numParticles = numSources*1000; % 粒子数量 不能太少！
particles = zeros(numParticles, 2, numTimeSteps); % 初始化粒子
weights = ones(numParticles, 1)/numParticles; % 初始化权重
angleNoise = deg2rad(5); % 观测角度的噪声: 最大误差值

% 初始化粒子
particles(:,1,1) = 6 * rand(numParticles, 1); % 在x方向上随机分布
particles(:,2,1) = 4 * rand(numParticles, 1); % 在y方向上随机分布

% 创建一个新的图形窗口
figure('units','normalized','outerposition',[0 0 1 1]);

% 可视化初始状态
t=1;
scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 300, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
hold on
plot(particles(:,1,t), particles(:,2,t), 'b.'); % 画出粒子
axis equal;
axis([0 6 0 4]);
hold off;
pause(1); % 暂停

simsteps = numTimeSteps+1;
% 粒子滤波
for t = 2:simsteps
    % 状态预测
    particles(:,:,t) = particles(:,:,t-1) + randn(numParticles,2)*0.01; % 加高斯噪声

    % 权重更新: 没有用似然函数
    for i = 1:numParticles
        max_likelihood = 0; % 初始化最大似然函数值
        for j = 1:numSources
            angle = observations(t-1,j); % 粒子初始化为0时刻，第一个时刻的观测值要用于更新，故t-1
            diff = particles(i,:,t) - robotPoses(t-1,1:2);
            particleAngle = atan2(diff(2), diff(1)); % 计算粒子相对于机器人的角度
            if particleAngle > angle-angleNoise && particleAngle < angle+angleNoise % 检查粒子是否在扇形区间内
                likelihood = 2; % 如果粒子在射线区间内，增加似然函数值
            else
                likelihood = 1; % 如果粒子不在射线区间内，减小似然函数值
            end
            if likelihood > max_likelihood
                max_likelihood = likelihood;
            end
        end
        weights(i) = weights(i) * max_likelihood; % 更新粒子的权重
    end

    % 权重更新：对所有粒子按似然更新权重
%     for i = 1:numParticles
%         max_likelihood = 0; % 初始化最大似然函数值
%         for j = 1:numSources
%             angle = observations(t-1,j); % 粒子初始化为0时刻，第一个时刻的观测值要用于更新，故t-1
%             diff = particles(i,:,t) - robotPoses(t-1,1:2);
%             particleAngle = atan2(diff(2), diff(1)); % 计算粒子相对于机器人的角度
%             
%             % 计算观测到的射线角度与粒子角度之间的差异
%             angleDiff = abs(angle - particleAngle); 
%             
%             % 根据高斯噪声模型计算似然函数
%             likelihood = exp(-0.5 * (angleDiff / angleNoise)^2) / (sqrt(2*pi) * angleNoise);
%             
%             if likelihood > max_likelihood
%                 max_likelihood = likelihood;
%             end
%         end
%         weights(i) = weights(i) * max_likelihood; % 如果粒子在射线区间内，更新粒子的权重
%     end

    % 权重更新：对落在区间内的粒子按似然更新权重
%     for i = 1:numParticles
%         max_likelihood = 0; % 初始化最大似然函数值
%         for j = 1:numSources
%             angle = observations(t-1,j); % 粒子初始化为0时刻，第一个时刻的观测值要用于更新，故t-1
%             diff = particles(i,:,t) - robotPoses(t-1,1:2);
%             particleAngle = atan2(diff(2), diff(1)); % 计算粒子相对于机器人的角度
%             
%             % 判断粒子是否在观测射线区间内
%             if particleAngle > angle-angleNoise && particleAngle < angle+angleNoise
%                 % 计算观测到的射线角度与粒子角度之间的差异
%                 angleDiff = abs(angle - particleAngle); 
%                 
%                 % 根据高斯噪声模型计算似然函数
%                 likelihood = exp(-0.5 * (angleDiff / angleNoise)^2) / (sqrt(2*pi) * angleNoise);
%                 
%                 if likelihood > max_likelihood
%                     max_likelihood = likelihood;
%                 end
%             end
%         end
%         if max_likelihood > 0
%             weights(i) = weights(i) * max_likelihood; % 如果粒子在射线区间内，更新粒子的权重
%         else
%             weights(i) = weights(i) * 0.1; % 如果粒子不在射线区间内，权重设为一个较小的值
%         end
%     end

    % 归一化权重
    weights = weights / sum(weights);

    % 重采样
    indices = randsample(1:numParticles, numParticles, true, weights);
    particles(:,:,t) = particles(indices,:,t);
    weights = ones(numParticles, 1)/numParticles;

    % 可视化
    clf; % 清除当前图形
    hold on;
    scatter(robotPoses(t-1,1), robotPoses(t-1,2), 100, 'k', 'filled'); % 画出机器人位置 黑色实心圆
    color = 'mgcyr';
    for i = 1:numSources
        angle = observations(t-1,i);
        quiver(robotPoses(t-1,1), robotPoses(t-1,2), cos(angle), sin(angle), 'color',color(i)); % 画出观测射线
        quiver(robotPoses(t-1,1), robotPoses(t-1,2), cos(angle-angleNoise), sin(angle-angleNoise), 'color',color(i), 'LineStyle','--'); % 画出观测射线区间的一端
        quiver(robotPoses(t-1,1), robotPoses(t-1,2), cos(angle+angleNoise), sin(angle+angleNoise), 'color',color(i), 'LineStyle','--'); % 画出观测射线区间的另一端
    end
    plot(particles(:,1,t), particles(:,2,t), 'b.'); % 画出粒子
    scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 300, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
    axis equal;
    axis([0 6 0 4]);
    hold off;
    pause(0.1); % 暂停0.1秒
end

%% 不执行重采样时用下面代码
% % 在所有步骤执行完毕后，画出权重较高的粒子
% weightPercentile = prctile(weights, 75); % 计算权重的75百分位数
% 
% % 找出权重较高的粒子
% highWeightIndices = find(weights >= weightPercentile); % 找出权重在75百分位数以上的粒子
% highWeightParticles = particles(highWeightIndices,:,simsteps); % 获取权重较高的粒子
% 
% % 创建一个新的图形窗口
% figure;
% 
% % 画出权重较高的粒子
% hold on;
% plot(srcGroundTruth(:,1), srcGroundTruth(:,2), 'ro');
% plot(highWeightParticles(:,1), highWeightParticles(:,2), 'b.'); % 画出权重较高的粒子
% axis equal;
% axis([0 6 0 4]);
% hold off;


