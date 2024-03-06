clear
clc
close all

% Load the data from the .mat file

%% 3 source 预设轨迹_53step  -成功
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-3_source\results\data\azimuth_output_3source_53.mat");

%% 5 source 预设轨迹_53step -观测值错误太多 但也有成功可能
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_source_fixed_path\results\data\azimuth_output_5source_53.mat");

%% 5 source 随机轨迹1_20step  -成功
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_source_random\results\data\source5_20step_01.mat");

%% 5 source 随机轨迹2_30step -成功
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source5_30step.mat");

%% 6 source 预设轨迹 -6个定4个 轨迹没探索到的没定出来
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source6_24step.mat");

%% 8 source 预设轨迹 

% 较精探索轨迹, 成功
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source8_36step.mat");

% 粗探索轨迹，成功
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source8_18step.mat");

% 精探索轨迹，成功
% data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-5_random_source_planned_path\results\data\source8_50step.mat");

% 仿真实验
% robotPoses = data.outputStruct.robotPos;
% robotPoses(:,3) = zeros(size(robotPoses,1),1); % 假设yaw全0
% observations = deg2rad(data.outputStruct.meanAzEst); % 转弧度制
% srcGroundTruth = data.outputStruct.srcGroundTruth;
% x_range = [0,6]; 
% y_range =[0,4]; 
% resolution = 1;
% soundRange = 5/resolution;

%%
% 真实实验 0101
% robotPoses = readmatrix("D:\SUSTech\Audio map\experiment\0101_exp\pose_theta_0101 - WHOLE.xlsx");
% observations = deg2rad(readmatrix("D:\SUSTech\Audio map\experiment\0101_exp\wav_denoised\SRP_azimuth_denoised_0101.xlsx"));
% srcGroundTruth = [[-0.68,-0.59,0];[-0.55,0.6,0]];
% x_range = [-2,8]; 
% y_range =[-1,5]; 
% resolution = 1; % 坐标单位=1米

% 真实实验 0112
% robotPoses = readmatrix("D:\SUSTech\Audio map\experiment\0112_exp\pose\pose_theta_0112.xlsx");
% observations = deg2rad(readmatrix("D:\SUSTech\Audio map\experiment\0112_exp\Wiener_azimuth_0112.xlsx"));
% srcGroundTruth = [[-0.68,-0.59,0];[-0.55,0.6,0]];
% x_range = [-2,2]; 
% y_range =[-2,2]; 
% resolution = 1; % 坐标单位=1米

% 真实实验 0127_t1
% robotPoses = readmatrix("D:\SUSTech\Audio map\experiment\0127_exp_t1\pose\pose_theta.xlsx");
% observations = deg2rad(readmatrix("D:\SUSTech\Audio map\experiment\0127_exp_t1\azimuth_GCC-PHAT_0127_t1.xlsx"));
% srcGroundTruth = [[0,       0,   0];[2.5,    0,   0];[5.0,    0,   0];[7.5,    1.2,0];[7.5,    3.6,0];
%                     [5.0,    4.8,0];[2.5,    4.8,0];  [0,       4.8,0];[-1.25, 3.6,0];[-1.25,1.2,0]]; 
% resolution = 1; % 坐标单位=1米
% soundRange = 3/resolution;
% x_range = [-3,8]; 
% y_range =[-3,8]; 

% 真实实验 0127_t2
robotPoses = readmatrix("D:\SUSTech\Audio map\experiment\0127_exp_t2\pose\pose_theta.xlsx");
observations = deg2rad(readmatrix("D:\SUSTech\Audio map\experiment\0127_exp_t2\azimuth_MVDR_0127_t2.xlsx"));
srcGroundTruth = [[0.54,1.09,0.775];[2.34,1.09,0.775];[4.14,1.09,0.775];[5.94,1.09,0.775];
                  [0,-1.2,0];[1.25,-1.2,0];[2.5,-1.2,0];[3.75,-1.2,0];[5,-1.2,0];[6.25,-1.2,0]]; 
resolution = 1; % 坐标单位=1米
soundRange = 3/resolution;
x_range = [-3,8]; 
y_range =[-4,5]; 

% 真实实验 0124_t2 10声源，地上，缺一个声源观测
% robotPoses = readmatrix("D:\SUSTech\Audio map\experiment\0124_exp_t2\pose\pose_theta.xlsx");
% observations = deg2rad(readmatrix("D:\SUSTech\Audio map\experiment\0124_exp_t2\azimuth_MVDR_0124_t2.xlsx"));
% angleNoise = deg2rad(7.5); % 观测角度的噪声
% srcGroundTruth = [[0.54,1.09,0.775];[2.34,1.09,0.775];[4.14,1.09,0.775];[5.94,1.09,0.775];
%                   [0,-1.2,0];[1.25,-1.2,0];[2.5,-1.2,0];[3.75,-1.2,0];[5,-1.2,0];[6.25,-1.2,0]]; 
% image = flip(image, 1);  % 颠倒图像的行
% resolution = 1; % 坐标单位=0.05米
% soundRange = 3/resolution; % 麦克风拾音范围
% x_range = [-3,8]; 
% y_range =[-4,5]; 


% 轨迹倒序
% observations = flipud(observations);
% robotPoses = flipud(robotPoses);

numSources = size(observations,2); % 声源数量
numTimeSteps = size(robotPoses,1); % 时间步长

%% 多声源定位算法实现

% 启动计时器
tic

x_width = (x_range(2)-x_range(1))*100;
y_length = (y_range(2)-y_range(1))*100;

% 计算总方格数
total_cells = x_width * y_length;

% 生成所有方格的索引列表
all_indices = zeros(total_cells, 2);
count = 1;
for i = 1:x_width
    for j = 1:y_length
        all_indices(count, :) = [x_range(1)*100+i, y_range(1)*100+j];
        count = count + 1;
    end
end

% 从索引列表中随机选取一部分
ratio = 1/(10*10);
random_indices = datasample(all_indices, total_cells * ratio, 'Replace', false);


% 初始化粒子滤波器参数
% numParticles = (x_width/20) * (y_length/20); % 初始粒子数量
numParticles = length(random_indices);
angleNoise = deg2rad(5); % 观测角度的噪声，弧度制
numObservations = 4; % 只取最强的N个观测 ceil(1/2 * size(observations,2))
threshold = 0.1; % 设置粒子比例阈值 仿真：0.5
associatedRange = 15; % 关联角度范围,degree 仿真：5

% DBSCAN聚类参数
epsilon = 0.1/resolution; % 距离阈值，用于定义邻域。如果两个点之间的距离小于或等于epsilon，那么这两个点就被认为是邻居
MinPts = numParticles*0.1; % 数量阈值，用于定义核心点。如果一个点的epsilon-邻域中至少包含MinPts个点（包括该点自身），那么这个点就被认为是一个核心点。
% DBSCAN算法的工作原理是，它首先找到一个核心点，然后找到这个核心点的所有直接密度可达的点，这些点构成一个聚类。
% 然后，算法继续找到下一个尚未被访问的核心点，重复上述过程，直到所有的点都被访问过，这样就得到了所有的聚类。

% 初始化声源粒子滤波器数组
detectedSourceFilters = [];

% 初始化更新后的观测值矩阵
updatedObservations = observations;

% 循环执行粒子滤波器的实现
count = 0; % 记录有效循环次数
while true
    close all

    % 粒子滤波器实现
    particleFilter = particleFiltering(numParticles, angleNoise, numTimeSteps, numObservations, updatedObservations, robotPoses, srcGroundTruth,x_range,y_range,random_indices);

    % 使用DBSCAN聚类声源
    newParticleFilters = createParticleFiltersFromDBSCAN(particleFilter, epsilon, MinPts, resolution);

    if isempty(newParticleFilters)
        break;
    end

    count = count + 1;

    % 可视化聚类结果
    figure('units','normalized','outerposition',[0 0 1 1]);
    hold on;
    num_newParticleFilters  = size(newParticleFilters,2);
    colors = hsv(num_newParticleFilters*5);
    for i = 1:num_newParticleFilters
        particleFilter = newParticleFilters{i};
        plot(particleFilter.particles(:,1), particleFilter.particles(:,2), '.', 'Color', colors(i*3,:));
    end
    scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 300, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
    title(["Cluster Results: Round ",count]);
    axis equal;
    axis([x_range(1) x_range(2) y_range(1) y_range(2)]);
    hold off;
    pause(0.5); % 暂停xx秒

    % 关联并更新observations矩阵
    [newParticleFilters, updatedObservations] = associateObservations(newParticleFilters, numTimeSteps, numSources, updatedObservations, robotPoses, threshold, associatedRange, soundRange);
    
    % 将新的粒子滤波器添加到数组中
    detectedSourceFilters = [detectedSourceFilters, newParticleFilters];
end


% 停止计时器
elapsedTime = toc;
% 显示运行时间
disp(['Your particle filter code took ', num2str(elapsedTime), ' seconds to run.']);

%% 可视化定位结果
num_detected_sources  = size(detectedSourceFilters,2);
figure('units','normalized','outerposition',[0 0 1 1]);
hold on;
colors = hsv(num_detected_sources+1);
% 画出每次声源定位（聚类）结果
for i = 1:num_detected_sources
    particleFilter = detectedSourceFilters{i};
    subplot(4,4,i);
    plot(particleFilter.particles(:,1), particleFilter.particles(:,2), '.', 'Color', colors(i+1,:));
    hold on
    scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 30, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
    title(["Cluster Results: Round ",i]);
    axis equal;
    axis([x_range(1) x_range(2) y_range(1) y_range(2)]);
    pause(0.5); % 暂停0.1秒
end


% 画出最后声源定位（聚类）结果
figure('units','normalized','outerposition',[0 0 1 1]);
hold on;
for i = 1:num_detected_sources
    particleFilter = detectedSourceFilters{i};
    plot(particleFilter.particles(:,1), particleFilter.particles(:,2), '.', 'Color', colors(i+1,:));
end
scatter(srcGroundTruth(:,1), srcGroundTruth(:,2), 300, 'rp', 'filled'); % 绘制声源位置真值 红色五角星
scatter(robotPoses(:,1), robotPoses(:,2), 50, 'k', 'filled'); % 画出机器人轨迹点 黑色实心圆
plot(robotPoses(:,1), robotPoses(:,2), 'k--'); % 使用虚线连接点
title("Final Localization Results");
axis equal;
axis([x_range(1) x_range(2) y_range(1) y_range(2)]);
hold off;

%%
% 初始化一个空的矩阵来存储所有的粒子位置
allParticles = [];

% 遍历所有的粒子滤波器
for i = 1:num_detected_sources
    particleFilter = detectedSourceFilters{i};
    
    % 获取这个滤波器中的所有粒子位置，并转换为毫米单位
    particles_mm = round(particleFilter.particles * 100);
    
    % 添加第三列，值为全1
    particles_mm = [particles_mm, ones(size(particles_mm, 1), 1)];
    
    % 将这些粒子位置添加到总的粒子位置矩阵中
    allParticles = [allParticles; particles_mm];
end

%% 使用unique函数删除重复的行
allParticles_deleted = unique(allParticles, 'rows');






