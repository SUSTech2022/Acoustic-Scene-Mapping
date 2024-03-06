function [ospa_distance, ospa_loc,ospa_card] = calculate_OSPA_distance(detectedSourceFilters, srcGroundTruth, resolution,c,p)
    I = size(detectedSourceFilters, 2); % 估计的点数 I
    H = size(srcGroundTruth, 1); % 真实数量
  
    cardility_error = I - H;

    d_c = zeros(H, I);
    for i = 1:I
        particleFilter = detectedSourceFilters{i};
        meanPosition = particleFilter.State;
        d_c(:, i) = min(c, abs((srcGroundTruth(:,1) - meanPosition(1)).^p + abs(srcGroundTruth(:,2) - meanPosition(2)).^p).^(1/p)); 
    end

    % 使用匈牙利算法找到最优的配对
    assignment = munkres(d_c);

    if I <= H
        % 计算 OSPA 距离
        ospa_distance = (1/H * (sum(d_c(assignment).^p) + (H-I)*c^p))^(1/p) *resolution;
        ospa_loc = (1/H * sum(d_c(assignment).^p))^(1/p) *resolution;
        ospa_card = (1/H *(H-I)*c^p)^(1/p) *resolution;
    else
        d_c = d_c';
        assignment = munkres(d_c);
        ospa_distance = (1/I * (sum(d_c(assignment).^p) + (I-H)*c^p))^(1/p) *resolution;
        ospa_loc = (1/I * sum(d_c(assignment).^p))^(1/p) *resolution;
        ospa_card = (1/I *(I-H)*c^p)^(1/p) *resolution;
    end
    
end
