%% convGrid2Mat -- used to convert an occupancy grid to a matrix, which
%                  then can be used in Dstar.

function occupancyMatrix = convG2M(occupancyGrid)
% converts this BinaryOccupancyGrid from ROS into a matrix for Dstar
% compatibility

gSize = occupancyGrid.GridSize;
tempMat = zeros(gSize(1), gSize(2));

for i=1: gSize(1)
    for j=1: gSize(2)
        tempMat(i, j) = getOccupancy(occupancyGrid, [i, j]);
    end
end

occupancyMatrix = tempMat;