function [xOg, yOg] = coo2M(Xlim, Ylim, res, X, Y)
    xOg = (Xlim(1)+(X/res));
    yOg = (Ylim(1)+(Y/res));
end