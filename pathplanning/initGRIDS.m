function [H, RHS, G] = initGRIDS(map, dest)
%Function initialize the heuristic grid with the same size as the map dimensions.
%This function initializes the destination to a value of 0, while all other
%points on the grid are estimates from that location to the destination.
    dimm = size(map);
    H = zeros(dimm);
    RHS = zeros(dimm);
    G = zeros(dimm);

    for i = 1:dimm(1)
        for j = 1:dimm(2)
            H(i, j) = sqrt((i - dest(1))^2 + abs(j - dest(2))^2);
            RHS(i, j) = inf;
            G(i, j) = inf;
        end
    end
    
    RHS(dest) = 0;
end
