function matrixMap = convB2M(BinaryOccupancyGrid)

% This function converts a binary ogrid to a DStar compatible matrix
% The map given back is a carbon copy image of the binary map.
% Tested and working!!

size = (BinaryOccupancyGrid.GridSize);
resolution = BinaryOccupancyGrid.Resolution;
% xWorld = BinaryOccupancyGrid.XWorldLimits;
% yWorld = BinaryOccupancyGrid.YWorldLimits;

tempM = zeros(size(1), size(2));

for xx=1:size(1)
    
    xAdjusted = (-51.2+(xx/resolution)-0.1);
    
    for yy=1:size(2)
        
        yAdjusted=(51.1-(yy/resolution)+0.1);
        tempM(xx, yy) = getOccupancy(BinaryOccupancyGrid, [xAdjusted, yAdjusted]);
    end
end

matrixMap = tempM;