% 读取数据
data = readmatrix("D:\SUSTech\Audio map\experiment\0128_exp\pose\pose_data.xlsx");

% 初始化theta数组
theta = zeros(size(data, 1), 1);

% 遍历每一行
for i = 1:size(data, 1)
    % 提取四元数 qw qx qy qz
    quat = [data(i, 7),data(i, 4:6)];
    
    % 转换为欧拉角
    eul = quat2eul(quat,'ZYX');
    
    % 计算theta
    theta(i) = rad2deg(eul(1));
end

% 创建新的数据矩阵
new_data = [data(:, 1:2), theta];

% 写入新的xlsx文件: x y theta
writematrix(new_data, "D:\SUSTech\Audio map\experiment\0128_exp\pose\pose_theta.xlsx");
