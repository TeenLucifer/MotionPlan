function x_nearIdxList = nearC(Tree, x_new, dis)

    x_nearIdxList = [];
    for i = 1 : size(Tree.v, 2)
        if norm([Tree.v(i).x; Tree.v(i).y] - x_new') < dis
            x_nearIdxList = [x_nearIdxList; i];
        end
    end

end

