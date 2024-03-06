clear
clc

% Load the data from the .mat file

%% 3 source 预设轨迹 -效果不错（比用单个粒子滤波器效果好，不用担心对某个声源的估计会消失）
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-3_source\results\data\azimuth_output_3source_53.mat");

%% 5 source 预设轨迹 -效果差，模拟观测很差的情形
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_source_fixed_path\results\data\azimuth_output_5source_53.mat");

%% 5 source 随机轨迹1_20step -效果很好
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_source_random\results\data\source5_20step_01.mat");

%% 5 source 随机轨迹2_30step -正序效果还行 倒序效果很好
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source5_30step.mat");

%% 8 source 随机轨迹_20step -效果  
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source8_20step.mat");
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source8_20step_01.mat");
data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source8_36step.mat");
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source8_18step.mat");
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source8_50step.mat");

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
numFilter = numSources;
numParticles = 1000; % 每个声源的粒子数量
particles = cell(numFilter , 1); % 初始化粒子，每个声源一个粒子滤波器
weights = cell(numFilter , 1); % 初始化权重，每个声源一个权重向量
angleNoise = deg2rad(5); % 观测角度的噪声: 最大误差值 最好能计算出来

% 初始化粒子和权重
for i = 1:numFilter  % numSources
    particles{i} = [6 * rand(numParticles, 1), 4 * rand(numParticles, 1)]; % 在x和y方向上随机分布
    weights{i} = ones(numParticles, 1)/numParticles;
end

% 创建一个新的图形窗口
figure('units','normalized','outerposition',[0 0 1 1]);

% 定义颜色
numColors = 15; % 你需要的颜色数量
colors = [
    0 1 0;  % 绿色
    0 0 1;  % 蓝色
    1 1 0;  % 黄色
    1 0 1;  % 品红
    0 1 1;  % 青色
    0.5 0 0;  % 深红色
    0 0.5 0;  % 深绿色
    0 0 0.5;  % 深蓝色
    0.5 0.5 0;  % 橄榄色
    0.5 0 0.5;  % 紫色
    0 0.5 0.5;  % 深青色
];


% 粒子滤波
for t = 1:numTimeSteps
%     % 可视化当前状态
%     for i = 1:numFilter  %numSources
%           scatter(particles{i}(:,1), particles{i}(:,2), [], colors(i,:), '.'); % 画出粒子，每个滤波器的粒子用不同的颜色表示
%         hold on
%     end
%     scatter(robotPoses(t,1), robotPoses(t,2), 100, 'k', 'filled'); % 画出机器人位置 黑色实心圆
%     hold on
%     for i = 1:numSources
%         angle = observations(t,i);
%         quiver(robotPoses(t,1), robotPoses(t,2), cos(angle), sin(angle), 'color',colors(i,:)); % 画出观测射线
%         quiver(robotPoses(t,1), robotPoses(t,2), cos(angle-angleNoise), sin(angle-angleNoise), 'color',colors(i,:), 'LineStyle','--'); % 画出观测射线区间的一端
%         quiver(robotPoses(t,1), robotPoses(t,2), cos(angle+angleNoise), sin(angle+angleNoise), 'color',colors(i,:), 'LineStyle','--'); % 画出观测射线区间的另一端
%     end
%     hold on
%     scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 300, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
%     axis equal;
%     axis([0 6 0 4]);
%     hold off;
%     pause(0.1); % 暂停0.1秒

    % 状态预测
    for i = 1:numFilter  % numSources
        particles{i} = particles{i} + randn(numParticles,2)*0.01; % 加高斯噪声
    end

    %权重更新 先做数据关联:使用MTT的最近邻方法
    %初始化一个矩阵来存储所有可能的配对的误差
    errors = zeros(numSources, numSources);
    
    %计算所有可能的配对的误差
    for i = 1:numFilter 
        for j = 1:numSources
            %计算粒子的角度
            diff = particles{i} - repmat(robotPoses(t,1:2), numParticles, 1);
            particleAngles = atan2(diff(:,2), diff(:,1));
            
            %计算观测角度和粒子角度之间的差的平方和
            angleDiffs = abs(particleAngles - observations(t,j));
            errors(i,j) = sum(angleDiffs.^2);
        end
    end

    % 使用匈牙利算法找到最优的配对
    assignmentMatrix = munkres(errors);
    
    % 将逻辑矩阵转换为索引
    assignments = zeros(1, numSources);
    for i = 1:numSources
        assignments(i) = find(assignmentMatrix(i, :));
    end
    
    % 根据最优的配对对观测值进行排序
    sortedObservations = observations(t,assignments);
    
    % 根据排序后的观测值更新权重
    for i = 1:numFilter  % numSources
        % 如果声源没有对应的观测值，那么跳过这个声源
        if isnan(sortedObservations(i))
            continue;
        end
        
        angle = sortedObservations(i);

        for k = 1:numParticles
            diff = particles{i}(k,:) - robotPoses(t,1:2);
            particleAngle = atan2(diff(2), diff(1));
            
            if particleAngle > angle-angleNoise & particleAngle < angle+angleNoise % 在区间内是高斯，出了区间就不是了
                angleDiff = abs(angle - particleAngle);
                likelihood = exp(-0.5 * (angleDiff / angleNoise)^2) / (sqrt(2*pi) * angleNoise);
                weights{i}(k) = weights{i}(k) * likelihood;
            else
                weights{i}(k) = weights{i}(k) * 0.01;  %这个的取值很关键，对不同场景，最优取值不同，最好能算出来
            end
        end

        %在归一化权重之前，检查所有的权重是否都接近于0
        if max(weights{i}) < eps
            weights{i} = ones(size(weights{i})) * eps; % 如果是，就将所有的权重设置为一个非零的小值
        end

         % 归一化权重并重采样
        weights{i} = weights{i} / sum(weights{i});
        indices = randsample(1:numParticles, numParticles, true, weights{i});
        particles{i} = particles{i}(indices,:);
        weights{i} = ones(numParticles, 1)/numParticles;

    end

    % 可视化当前状态
    for i = 1:numFilter  %numSources
          scatter(particles{i}(:,1), particles{i}(:,2), [], colors(i,:), '.'); % 画出粒子，每个滤波器的粒子用不同的颜色表示
        hold on
    end
    scatter(robotPoses(t,1), robotPoses(t,2), 100, 'k', 'filled'); % 画出机器人位置 黑色实心圆
    hold on
    for i = 1:numFilter
        angle = sortedObservations(i);
        quiver(robotPoses(t,1), robotPoses(t,2), cos(angle), sin(angle), 'color',colors(i,:)); % 画出观测射线
        quiver(robotPoses(t,1), robotPoses(t,2), cos(angle-angleNoise), sin(angle-angleNoise), 'color',colors(i,:), 'LineStyle','--'); % 画出观测射线区间的一端
        quiver(robotPoses(t,1), robotPoses(t,2), cos(angle+angleNoise), sin(angle+angleNoise), 'color',colors(i,:), 'LineStyle','--'); % 画出观测射线区间的另一端
    end
    hold on
    scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 300, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
    axis equal;
    axis([0 6 0 4]);
    hold off;
    pause(0.1); % 暂停0.1秒

end



