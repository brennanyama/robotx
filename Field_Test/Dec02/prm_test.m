% Script to test the PRM path planner with ROS bagfile data.
MAPsub = rossubscriber('/map', 'nav_msgs/OccupancyGrid');
OdoSub = rossubscriber('/odometry/filtered', 'nav_msgs/Odometry');

% Initialize PRM
robotRadius = 0.25;

prm = robotics.PRM;
prm.NumNodes = 500;
prm.ConnectionDistance = 8;
goal = input('Goal Coordinates [x, y]: ');
done = false;

while(~done)
	% Receive new Occupancy Grid
	OdoData = receive(OdoSub);
	X = OdoData.Pose.Pose.Position.X;
	Y = OdoData.Pose.Pose.Position.Y;
	currentPos = [X, Y];

	OGrid = receive(MAPsub);
	binaryMap = readBinaryOccupancyGrid(OGrid);
	inflate(binaryMap, robotRadius);
	prm.Map = binaryMap;
	update(prm);

	% Plan path
	prm_path = findpath(prm, currentPos, goal);
	
	% Command Line Display
	disp(['Current Position', num2str(currentPos)]);
	disp(prm_path);

	if (isequal(currentPos, goal))
		done = true;
	end
end
