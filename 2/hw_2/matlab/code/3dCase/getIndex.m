function index = getIndex(OPEN, node)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.

    for i = 1 : size(OPEN, 1)
        if(OPEN(i, 2) == node(1) && OPEN(i, 3) == node(2) && OPEN(i, 4) == node(3))
            index = i;
            return;
        end
    end
    index = i + 1;
end

