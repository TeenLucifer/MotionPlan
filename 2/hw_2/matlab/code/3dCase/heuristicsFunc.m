function dist = heuristicsFunc(pos1, pos2, heuType)
    dx = pos1(1) - pos2(1);
    dy = pos1(2) - pos2(2);
    dz = pos1(3) - pos2(3);
    if(heuType == "EuclideanHeu")
        dist = sqrt(dx^2 + dy^2 + dz^2);
    elseif(heuType == "ManhattanHeu")
        dist = dx + dy + dz;
    elseif(heuType == "DiagonalHeu")
        dist = dx + dy + dz + (sqrt(3) - 2) * min([dx, dy, dz]);
    end
end

