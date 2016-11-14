% This script is used to test the mission_plan.m script

% Defind buoy constant values
RED = 1;
GREEN = 2;

% Define current position
currX = 500;
currY = 500;
% Define known buoy locations
% RED BUOYS
red_buoys = []%[100, 100;
             %200, 100];
% GREEN BUOYS
green_buoys = []%[100, 150;
               %200, 150];

% Populate event_map
event_map = zeros([1024, 1024]);

red_num = size(red_buoys);
green_num = size(green_buoys);

for i = 1:red_num(1)
    event_map(red_buoys(i, 1), red_buoys(i, 2)) = RED;
end

for i = 1:green_num(1)
    event_map(green_buoys(i, 1), green_buoys(i, 2)) = GREEN;
end

[destX, destY] = mission_plan(event_map, currX, currY);
destX= [currX, destX]; destY = [currY, destY];

% Plot
figure(1)
hold on
plot(destX, destY, 'k--d');
axis([0, 1024, 0, 1024]);
hold off