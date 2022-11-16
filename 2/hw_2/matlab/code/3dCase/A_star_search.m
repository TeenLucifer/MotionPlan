function path = A_star_search(map, MAX_X, MAX_Y, MAX_Z);
    nodeVisited = 0;
    heuType = "DiagonalHeu";
    
    size_map = size(map, 1);
    MAP = 2 * ones(MAX_X, MAX_Y, MAX_Z);
    
    xval = map(size_map, 1);
    yval = map(size_map, 2);
    zval = map(size_map, 3);
    xTarget = xval;
    yTarget = yval;
    zTarget = zval;
    MAP(xval, yval, zval) = 0;
    
    for i = 2 : size_map - 1
        xval = map(i, 1);
        yval = map(i, 2);
        zval = map(i, 3);
        MAP(xval, yval, zval) = -1;
    end
    
    xval = map(1, 1);
    yval = map(1, 2);
    zval = map(1, 3);
    xStart = xval;
    yStart = yval;
    zStart = zval;
    MAP(xval, yval, zval) = 1;
    
    OPEN = [];
    CLOSED = [];
    
    t = 1;
    for i = 1 : MAX_X
        for j = 1 : MAX_Y
            for k = 1 : MAX_Z
                if(MAP(i, j, k) == -1)
                    CLOSED(t, 1) = i;
                    CLOSED(t, 2) = j;
                    CLOSED(t, 3) = k;
                    t = t + 1;
                end
            end
        end
    end
    CLOSED_COUNT = size(CLOSED, 1);

    currentNode = [xStart; yStart; zStart];
    parentNode = currentNode;
    goalNode = [xTarget; yTarget; zTarget];
    OPEN_COUNT = 1;
    goal_distance = heuristicsFunc(currentNode, goalNode, heuType);
    path_cost = 0;
    hn = goal_distance;
    gn = path_cost;
    fn = gn + hn;
    OPEN(OPEN_COUNT, :) = insert_open(currentNode, parentNode, hn, gn, fn);
    NoPath = 1;

    while(1)
        nodeVisited = nodeVisited + 1;
        if((sum(OPEN(:, 1)) == 0) || (OPEN(OPEN_COUNT, 2) == goalNode(1) && OPEN(OPEN_COUNT, 3) == goalNode(2) && OPEN(OPEN_COUNT, 4) == goalNode(3)))
            break;
        end
        currentNodeIdx = min_fn(OPEN, OPEN_COUNT);
        currentNode = OPEN(currentNodeIdx, 2 : 4);
        currentNode = currentNode';
        gn = OPEN(currentNodeIdx, 9);
        
        OPEN(currentNodeIdx, 1) = 0;
        CLOSED_COUNT = CLOSED_COUNT + 1;
        CLOSED(CLOSED_COUNT, 1 : 3) = currentNode';
        
        neighbors = expand_array(currentNode, goalNode, gn, CLOSED, MAX_X, MAX_Y, MAX_Z, heuType);
        
        for i = 1 : size(neighbors, 1)
            x = neighbors(i, 1);
            y = neighbors(i, 2);
            z = neighbors(i, 3);
            hn = neighbors(i, 4);
            gn = neighbors(i, 5);
            fn = neighbors(i, 6);
            if(goalNode == [x; y; z])
                NoPath = 0;
                OPEN_COUNT = OPEN_COUNT + 1;
                OPEN(OPEN_COUNT, :) = insert_open([x; y; z], currentNode, hn, gn, fn);
                break;
            end
            idx = getIndex(OPEN, [x; y; z]);
            if(idx > OPEN_COUNT)
                OPEN_COUNT = OPEN_COUNT + 1;
                OPEN(OPEN_COUNT, :) = insert_open([x; y; z], currentNode, hn, gn, fn);
            else
                if(OPEN(idx, 10) > fn)
                    OPEN(idx, 5 : 7) = currentNode';
                    OPEN(idx, 9) = gn;
                    OPEN(idx, 10) = fn;
                end
            end
        end
    end
    
    path = [];
    if(NoPath)
        return;
    else
        n = 1;
        path(n, :) = goalNode';
        index = OPEN_COUNT;
        while(index > 1)
            parentNode = OPEN(index, 5 : 7);
            parentNode = parentNode';
            index = getIndex(OPEN, parentNode);
            n = n + 1;
            path(n, :) = parentNode';%通过OPEN中记录的各个节点的父节点进行回溯，获得path
        end
    end
%     path(n + 1, :) = goalNode';
    path = flip(path);
    str = sprintf("访问的总节点数为%d\n", nodeVisited);
   disp(heuType + "启发函数" + str);
end

