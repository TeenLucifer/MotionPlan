function map = obstacle_map(posStart, posTarget, MAX_X, MAX_Y, MAX_Z)

    rand_map = rand(MAX_X, MAX_Y, MAX_Z);
    map = [];
    map(1, :) = posStart';
    t = 2;
    obstacle_ration = 0.25;
    for i = 1 : MAX_X
        for j = 1 : MAX_Y
            for k = 1 : MAX_Z
                if(rand_map(i, j, k) < obstacle_ration && (i ~= posStart(1) || j ~= posStart(2) || k ~= posStart(3)) && (i ~= posTarget(1) || j ~= posTarget(2) || k ~= posTarget(3)))
                    map(t, :) = [i; j; k]';
                    t = t + 1;
                end
            end
        end
    end
    map(t, :) = posTarget';
end

