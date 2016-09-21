function new_row = insert_open(pos, parent)
%Function to populate the OPEN list. Formatted as such:
%| On List  1/0 | X Coor | Y Coor | Parent X Coor | Parent Y Coor |
    new_row = [1, 5];
    new_row(1, 1) = 1;
    new_row(1, 2) = pos(1);
    new_row(1, 3) = pos(2);
    new_row(1, 4) = parent(1);
    new_row(1, 5) = parent(2);
end