function idx = min_fn(OPEN, OPEN_COUNT)

    minNum = 99999;
    idx = -1;
    for i = 1 : size(OPEN, 1)
        if(OPEN(i, 1) ~= 0 && OPEN(i, 10) < minNum)
            minNum = OPEN(i, 10);
            idx = i;
        end
    end

end

