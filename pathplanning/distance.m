function dist = distance(pos, dest)
%Function finds the distance between two coordinates in 2D space.
    dist = sqrt((pos(1)-dest(1))^2+(pos(2)-dest(2))^2);
end
