function dist = distance(x1, y1, x2, y2, heuType)
%This function calculates the distance between any two cartesian 
%coordinates.
%   Copyright 2009-2010 The MathWorks, Inc.
if(heuType == "EuclideanHeu")
    dist = sqrt((x1 - x2)^2 + (y1 - y2)^2);
elseif(heuType == "ManhattanHeu")
    dist = (x1 - x2) + (y1 - y2);
elseif(heuType == "DiagonalHeu")
    dist = (x1 - x2) + (y1 - y2) + (sqrt(2) - 2) * min([(x1 - x2), (y1 - y2)]);
end
    
    
