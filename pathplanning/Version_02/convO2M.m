function [matrix, start] = convO2M(occupancyGrid, currLocation)

% For Map conversion
map=readBinaryOccupancyGrid(occupancyGrid);
inflate(map, 0.02);

size=(map.GridSize);
resolution = map.Resolution;
xWorld=map.XWorldLimits;
yWorld=map.YWorldLimits;

tempM=zeros(size(1), size(2));

for xx=1:size(1)
    for yy=1:size(2)
        
        xAdjusted=(xWorld(1)+(xx/resolution));
        yAdjusted=(yWorld(1)+(yy/resolution));
        
        tempM(xx, yy) = getOccupancy(map, [xAdjusted, yAdjusted]);
    end
end

matrix = tempM;

% For currX, currY conversion
xAdjusted = (xWorld(1)+(currLocation(1)/resolution));
yAdjusted = (yWorld(1)+(currLocation(2)/resolution));
start = [xAdjusted, yAdjusted];

