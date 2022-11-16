function exp_array = expand_array(currentNode, goalNode, gn, CLOSED, MAX_X, MAX_Y, MAX_Z, heuType)
    %Function to return an expanded array
    %This function takes a node and returns the expanded list
    %of successors,with the calculated fn values.
    %The criteria being none of the successors are on the CLOSED list.
    %
    %Copyright 2009-2010 The MathWorks, Inc.
    
    %EXPANDED ARRAY FORMAT
    %--------------------------------
    %|X val |Y val ||h(n) |g(n)|f(n)|
    %--------------------------------
    
    exp_array = [];
    exp_count = 1;
    c2 = size(CLOSED, 1);%Number of elements in CLOSED including the zeros
    for i = 1 : -1 : -1
        for j = 1 : -1 : -1
            for k = 1 : -1 : -1
                newX = currentNode(1) + i;
                newY = currentNode(2) + j;
                newZ = currentNode(3) + k;
                if(newX > 0 && newX <= MAX_X && newY > 0 && newY <= MAX_Y && newZ > 0 && newZ <= MAX_Z)
                    flag = 1;
                    for c1 = 1 : c2
                        if(newX == CLOSED(c1, 1) && newY == CLOSED(c1, 2) && newZ == CLOSED(c1, 3))
                            flag = 0;
                        end
                    end
                    if(flag == 1)
                        exp_array(exp_count, 1) = newX;
                        exp_array(exp_count, 2) = newY;
                        exp_array(exp_count, 3) = newZ;
                        exp_array(exp_count, 4) = heuristicsFunc(goalNode, [newX; newY; newZ], heuType);%distance between node and goal,hn
                        exp_array(exp_count, 5) = gn + heuristicsFunc(currentNode, [newX; newY; newZ], "EuclideanHeu");%cost of travelling to node，gn
                        exp_array(exp_count, 6) = exp_array(exp_count, 4) + exp_array(exp_count, 5);%fn
                        exp_count = exp_count + 1;
                    end
                end
            end
        end%End of j for loop
    end%End of k for loop
end