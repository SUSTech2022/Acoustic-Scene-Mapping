function [row, col] = initializeParticles(image, D)
    % Initialize particles' positions randomly on the map
    
    % Find all valid pixels 
    [row, col] = find(image ~= 205);
    numCoords = length(row); 

    % Randomly select 1/D of the coordinates (otherwise, too many particles causing clustering to freeze)
    indices = randsample(numCoords, floor(numCoords/(D))); % Generate random indices, approximately 0.15m*0.15m per particle on average
    
    row = row(indices); % Select corresponding rows
    col = col(indices); % Select corresponding columns
end
