%% Test secton for baisc matrix grid
% part 1
maptest = [0 0 0 0 0 1 0 0 0 0;0 0 0 0 0 0 1 1 0 0 ; 1 1 1 1 0 0 0 0 0 0;0 0 0 0 0 0 1 0 0 0;
           0 0 0 0 0 0 0 0 0 1; 0 0 0 0 0 0 1 1 0 0 ; 0 0 0 0 1 0 0 0 0 0;
           0 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 0 0 0 0 0]
starttest = [1, 1];
goaltest = [10, 10];
dstest=Dstar(maptest);
ds.plan(goaltest)
ds.path(starttest)

%% taken from OGrid

gridtest = robotics.BinaryOccupancyGrid(10, 10, 10);
xtest = [5:0.1:6];
ytest = [5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5]';
xytest = [x, y];
