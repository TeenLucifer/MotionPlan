function index = getIndex(OPEN, xval, yval)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.

    for i = 1 : size(OPEN, 1)
        if(OPEN(i, 2) == xval && OPEN(i, 3) == yval)
            index = i;
            return;
        end
    end
    index = i + 1;
end

