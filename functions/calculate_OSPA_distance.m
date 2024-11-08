function [ospa, locOspa,cardOspa] = calculate_OSPA_distance(detectedSourceFilters, srcGroundTruth, resolution,c,p)
    % Calculates the OSPA (Optimal Sub-Pattern Assignment) distance between estimated source position and ground truth data.

    I = size(detectedSourceFilters, 2); % Number of estimated points I
    H = size(srcGroundTruth, 1); % Number of true sources
    srcGroundTruth = srcGroundTruth * resolution; % unit: m
    
    % Convert detectedSourceFilters to a structure array to meet the requirements of trackOSPAMetric
    tracks.State = detectedSourceFilters{1}.State; 
    for i = 1:I
        position = detectedSourceFilters{i}.State * resolution;
        tracks(i).State = [position(1); 0; position(2); 0; 0; 0];  % position 1,3,5, velocity 2,4,6
    end 
    
    truths.Position = srcGroundTruth(1,:);
    truths.Velocity = [0, 0, 0];
    for h = 1:H
        truths(h).Position = srcGroundTruth(h,:);
        truths(h).Velocity = [0, 0, 0];
    end
    
    % Create a trackOSPAMetric object
    ospaMetric = trackOSPAMetric(Metric="OSPA", ...
        LabelingError=0, ...
        CutoffDistance=c, ...
        Order=p, ...
        Distance='posabserr');
    
    % Call the object to calculate the OSPA value
    [ospa, locOspa, cardOspa] = ospaMetric(tracks, truths);
       
end
