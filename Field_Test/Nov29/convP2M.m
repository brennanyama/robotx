function pos = convP2M(xLocation, yLocation, binaryMap)

% The purpose of this function is to take an x and y pair relative to a
% Matlab pair of points
% This pair of points returned is UNROUNDED, unless round = -1 or 0

resolution = binaryMap.Resolution;


x = round(xLocation*resolution);
y = round(yLocation*resolution);

% if round == -1
%     x = roundn(xTemp, round);
%     y = roundn(xTemp, round);
%     
% elseif round == 1
%     x = round(xTemp);
%     y = round(xTemp);
% else
%     x = xTemp;
%     y = yTemp;
% end
    
    

xAdj = 512 + x + 1;
yAdj = 512 - y;
pos = [xAdj, yAdj];