%% 流程初始化
clc
clear; 
close all;

x_I = 1; y_I = 1;           % 设置初始点
x_G = 700; y_G = 700;       % 设置目标点（可尝试修改终点）
Thr = 50;                   % 设置目标点阈值
Delta = 30;                 % 设置扩展步长
goal = [x_G, y_G];
%% 建树初始化
T.v(1).x = x_I;           % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;       % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist = 0;          % 从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;       %
% T.v(1).gVal = 0;          %从起点到当前节点的最优距离
%% 开始构建树，作业部分
figure(1);
ImpRgb = imread('newmap.png');
Imp = rgb2gray(ImpRgb);
imshow(Imp)
xL = size(Imp, 2);%地图x轴长度
yL = size(Imp, 1);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(x_G, y_G, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');% 绘制起点和目标点
count = 1;
bFind = false;             % 设置扩展步长

for iter = 1 : 3000
    x_rand = [];
    x_rand(1) = randi([1, xL]);
    x_rand(2) = randi([1, yL]);
    
    x_near = [];
    min_dis = 99999;
    min_idx = -1;
    for i = 1 : size(T.v, 2)
        dis = norm([T.v(i).x; T.v(i).y] - [x_rand(1); x_rand(2)]);
        if dis < min_dis
            min_idx = i;
            min_dis = dis;
            x_near(1) = T.v(i).x;
            x_near(2) = T.v(i).y;
        end
    end
    
    x_new = [];
    theta = atan2(x_rand(2) - x_near(2), x_rand(1) - x_near(1));
    x_new(1) = x_near(1) + Delta * cos(theta);
    x_new(2) = x_near(2) + Delta * sin(theta);
    
    if ~collisionChecking(x_near, x_new, Imp)
        continue;
    end
    
    %nearC && chooseParent
    nearptr = [];
    nearcount = 0;
    neardist = norm(x_new - x_near) + T.v(min_idx).dist;
    near_iter = min_idx;
    for j = 1 : size(T.v, 2)
        if j == min_idx
            continue;
        end
        x_neartmp(1) = T.v(j).x;
        x_neartmp(2) = T.v(j).y;
        dist = norm(x_new - x_neartmp) + T.v(j).dist;
        norm_dist = norm(x_new - x_neartmp);
        if norm_dist < 120
            %nearC
            if collisionChecking(x_neartmp, x_new, Imp)
                nearcount = nearcount + 1;
                nearptr(nearcount, 1) = j;
                if neardist > dist
                    neardist = dist;
                    near_iter = j;
                end
            end
        end
    end
    
    x_near(1) = T.v(near_iter).x;
    x_near(2) = T.v(near_iter).y;
    count = count + 1;
    
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).xPrev = x_near(1);
    T.v(count).yPrev = x_near(2);
    T.v(count).dist = norm(x_new - x_near) + T.v(near_iter).dist;
    T.v(count).indPrev = near_iter;
    
    %rewire
    for k = 1 : size(nearptr, 1)
        x_1(1) = T.v(nearptr(k, 1)).x;
        x_1(2) = T.v(nearptr(k, 1)).y;
        x1_prev(1) = T.v(nearptr(k, 1)).xPrev;
        x1_prev(2) = T.v(nearptr(k, 1)).yPrev;
        if T.v(nearptr(k, 1)).dist > (T.v(count).dist + norm(x_1 - x_new))
            T.v(nearptr(k, 1)).dist = T.v(count).dist + norm(x_1 - x_new);
            T.v(nearptr(k, 1)).xPrev = x_new(1);
            T.v(nearptr(k, 1)).yPrev = x_new(2);
            T.v(nearptr(k, 1)).indPrev = count;
            plot([x_1(1), x1_prev(1)], [x_1(2), x1_prev(2)], '-w');
            hold on;
            plot([x_1(1), x_new(1)], [x_1(2), x_new(2)], '-g');
            hold on;
        end
    end
    
    plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], '-r');
    hold on;
    plot(x_new(1), x_new(2), '-r');
    hold on;
    %Step 5:检查是否到达目标点附近
    %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
    if norm(x_new - goal) < Thr
        bFind = true;
        break;
    end
    
    pause(0.05); %暂停一会，使得RRT扩展过程容易观察
    
end
%% 路径已经找到，反向查询
if bFind
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % 终点加入路径
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end