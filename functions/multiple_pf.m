clear;
clc;

% 假设参数
numParticles = 100; % 粒子数量

% Load the data from the .mat file
data = load("D:\SUSTech\Audio map\code\sound map framework\mbss_locate\v2.0\examples\sim_multi-3_source\results\data\azimuth_output_3source_53.mat");

% Extract the robotPos and DOA data
robotPoses = data.outputStruct.robotPos;
robotPoses(:,3) = zeros(size(robotPoses,1),1) % 假设yaw全0

observations = deg2rad(data.outputStruct.meanAzEst);
srcGroundTruth = data.outputStruct.srcGroundTruth;

% robotPoses = [0.705	1.926  0; 
%               0.705	2.126  0; 
%               0.705	2.326  0;
%               0.705	2.526  0; 
%               0.705	2.726  0;
%               0.705	2.926  0]; % 机器人位姿  x y yaw
% 
% observations = deg2rad([36	-8	16; 
%                         26	-9	15; 
%                         12	-14	-11; 
%                         -3	-17	-40;
%                         -16	2	43;
%                         -28	1	44]); % 观测值

numSources = size(observations,2); % 声源数量
numTimeSteps = size(robotPoses,1); % 时间步长

detectRange = 5; % 麦克风阵列拾音范围


% 初始化粒子和权重
particles = cell(1, numSources);
weights = cell(1, numSources);
for i = 1:numSources
    particles{i} = [robotPoses(i,1)+detectRange*cos(observations(1,i)) + 0.1*randn(1, numParticles); robotPoses(i,2)+detectRange*sin(observations(1,i)) + 0.1*randn(1, numParticles)];
    weights{i} = ones(1, numParticles) / numParticles;
end


% 创建一个新的图形窗口
figure;
% 预测-更新循环
for t = 2:numTimeSteps
    
    for i = 1:numSources
        % 预测步骤
        particles{i} = particles{i} + randn(size(particles{i}));
        
        % 更新步骤
        particlePositions = particles{i};
%         observationAngles = observations(t,:) + robotPoses(t,3);  % 实际观测数据需要进行坐标系       
        observationAngles = observations(t,:);  % 仿真的DOA结果已是世界坐标系下的表达
        distances = zeros(numParticles, numSources);
        % 计算粒子到每条观测线的距离
        for j = 1:numSources 
            a = sin(observationAngles(j));
            b = -cos(observationAngles(j));
            c = -a*robotPoses(t,1) - b*robotPoses(t,2);
            distances(:,j) = abs(a*particlePositions(1,:) + b*particlePositions(2,:) + c) / sqrt(a^2 + b^2);
        end
        particleCenter = mean(particlePositions, 2);
        centerDistances = zeros(1, numSources);
        for j = 1:numSources
            a = sin(observationAngles(j));
            b = -cos(observationAngles(j));
            c = -a*robotPoses(t,1) - b*robotPoses(t,2);
            centerDistances(j) = abs(a*particleCenter(1) + b*particleCenter(2) + c) / sqrt(a^2 + b^2);
        end
        [~, idx] = min(centerDistances); % 数据关联结束，对于该粒子滤波器i，选择第idx个观测值进行权重更新
        weights{i} = exp(-0.5*((distances(:,idx))/0.1).^2);
        weights{i} = weights{i} / sum(weights{i});
        
        % 重采样步骤
        indices = randsample(1:numParticles, numParticles, true, weights{i});
        particles{i} = particles{i}(:, indices);
        weights{i} = ones(1, numParticles) / numParticles;
        
        % 可视化粒子
        scatter(particles{i}(1,:), particles{i}(2,:));
        hold on;
    end
    % 画出机器人的位置
    plot(robotPoses(t,1), robotPoses(t,2), 'k*', 'MarkerSize', 10);
    % 画出观测值的射线
    for j = 1:numSources
        line([robotPoses(t,1), robotPoses(t,1) + 10*cos(observationAngles(j))], [robotPoses(t,2), robotPoses(t,2) + 10*sin(observationAngles(j))], 'Color', 'r');
    end
    title('所有声源的粒子位置、机器人的位置以及观测值的射线');
    xlabel('x');
    ylabel('y');
    axis equal;
    axis([0 6 0 4]);
    hold off;
    
    % 暂停一下以便观察
    pause(1);
end
