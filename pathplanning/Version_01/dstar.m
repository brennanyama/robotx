%% Initialize ROS Interface
clear all;                  % for testing
rosshutdown;                % for testing
rosinit('localhost');       % for testing

% Get Occupancy Grid from ROS
%msg = rosmessage('nav_msgs/OccupancyGrid');
%msg.Info.Height = 10; 
%msg.Info.Width = 10;
%msg.Info.Resolution = 0.1;
%msg.Data = 100*rand(100,1);

OGridSub = rossubscriber('/OccupancyGrid', 'nav_msgs/OccupancyGrid'); % TODO establish which topic for OGrid msgs
GPSSub = rossubscriber('GPSLocation', 'sensor_msgs/NavSatFix');       % TODO establish topic
OGridPub = rospublisher('/OptimalPath', 'nav_msgs/Path');
optimalPath = rosmessage('nav_msgs/Path');  % no instance of sent message yet

% OGridData = OGridSub.LatestMessage;   -- Alternative
OGridData = receive(OGridSub, 3);       % receives data from sub, 3 sec timeout, disable for testing
GPAData = receive(GPSSub, 3);           % Receive Raw GPS data, lat and long

%% Establish OGrid, OMatrix, goal for Dstar

curOGrid = OGridData;       % This should be a BinaryOccupancyGrid
map = convG2M(curOGrid);    % converts BOG to a matrix for algorithm
pos = [50, 50];             % TODO: get position from ROS
goal = [1, 1];              % get goal from ROS or use arbitrary goal

%map = randn(100, 100);     % enable for testing  
%map = floor(map);
%dimm = size(map);

%% Dstar algorithm start       !! Algorithm sees '-1' as obstruction
parent = pos;               % Initialize the parent node
dimm = size(map);
% Initialization
OPEN = [];                  % Nodes that are to be explored
open_count = 1;             % Number of nodes in OPEN list
CLOSED = [];                % Nodes that are explored
k = inf;                    % Smallest value of H that is on the OPEN list
path_cost = 0;              % Path Cost 
[H, RHS, G] = initGRIDS(map, goal);

OPEN(open_count, :) = insert_open(pos, parent);
H(pos) = path_cost;
RHS(pos) = distance(pos, goal);
G(pos) = distance(pos, goal);

NoPath = 1;                 % Solution has not yet been found.

% Places all obstructed locations on the closed list. In Occupancy Grid, a
% value of -1 denotes an obstruction at that location.
z = 1;                      % Counter
for i = 1 : dimm(1)
    for j = 1 : dimm(2)
        if(map(i,j) == -1)
            CLOSED(z,1)=i; 
            CLOSED(z,2)=j; 
            z=z+1;
        end
    end
end

closed_count = size(CLOSED, 1);

% Set the starting node as the first node
xNode = pos(1);
yNode = pos(2);
open_count = 1;
path_cost = 0;
dist = distance([xNode, yNode], goal);
OPEN(open_count, :) = insert_open([xNode, yNode], [xNode, yNode]);
H(xNode, yNode) = path_cost;
RHS(xNode, yNode) = distance([xNode, yNode], goal);
G(xNode, yNode) = distance([xNode, yNode], goal);
closed_count = closed_count + 1;
CLOSED(closed_count, 1) = xNode;
CLOSED(closed_count, 2) = yNode;
NoPath = 1;

while((xNode ~= goal(1) || yNode ~= goal(2)) && NoPath == 1)
    EXP_ARRAY = expand_array(xNode, yNode, path_cost, goal, CLOSED, dimm);
    exp_count = size(EXP_ARRAY, 1);
    
    for i = 1 : exp_count
        flag = 0;
        for j = 1 : open_count
            if(EXP_ARRAY(i, 1) == OPEN(j, 2) && EXP_ARRAY(i, 2) == OPEN(j, 3))
                RHS(OPEN(j, 2), OPEN(j, 3)) = min(RHS(OPEN(j, 2), OPEN(j, 3)), EXP_ARRAY(i, 5));
                if(RHS(OPEN(j, 2), OPEN(j, 3)) == EXP_ARRAY(i, 5))
                    % Update parents, G, and H
                    OPEN(j, 4) = xNode;
                    OPEN(j, 5) = yNode;
                    H(EXP_ARRAY(i, 1), EXP_ARRAY(i, 2)) = EXP_ARRAY(i, 3);
                    G(EXP_ARRAY(i, 1), EXP_ARRAY(i, 2)) = EXP_ARRAY(i, 4);
                end
                flag = 1;
            end
        end
        if flag == 0
            open_count = open_count+1;
            OPEN(open_count, :) = insert_open([EXP_ARRAY(i, 1), EXP_ARRAY(i, 2)], [xNode, yNode]);
        end
    end
    
    index_min_node = min_fn(OPEN, open_count, goal, G, RHS);
    if(index_min_node ~= -1)
        xNode = OPEN(index_min_node, 2);
        yNode = OPEN(index_min_node, 3);
        path_cost = H(xNode, yNode);
        OPEN(index_min_node, 1) = 0;
    else
        NoPath = 0;
    end
end

i = size(CLOSED, 1);
OPTIMAL_PATH = [];
xVal = CLOSED(i, 1);
yVal = CLOSED(i, 2);
i = 1;

OPTIMAL_PATH(i, 1) = xVal;
OPTIMAL_PATH(i, 2) = yVal;
i = i + 1;

if ((xVal == goal(1))&&(yVal == goal(2)))
    iNode = 0;
    
    parent_x = OPEN(node_index(OPEN, xVal, yVal), 4);
    parent_y = OPEN(node_index(OPEN, xVal, yVal), 5);
    
    while( parent_x ~= pos(1) || parent_y ~= pos(2) ) 
        OPTIMAL_PATH(i, 1) = parent_x;
        OPTIMAL_PATH(i, 2) = parent_y;
        
        iNode = node_index(OPEN, parent_x, parent_y);
        parent_x = OPEN(iNode, 4);
        parent_y = OPEN(iNode, 5);
        i = i + 1;
    end
    
else
    disp('No path exists to Goal.');
end

        
