% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;
addpath('A_star')

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 9.0;
yTarget = 9.0;
MAX_X = 10;
MAX_Y = 10;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);
% map = [1,1;1,2;1,10;2,3;2,4;2,10;3,1;4,4;4,6;4,8;5,4;5,5;5,6;6,1;6,2;6,10;7,8;8,7;9,7;9,10;10,3;10,4;10,6;9,9];

% Waypoint Generator Using the A* 
path = A_star_search(map, MAX_X, MAX_Y);

% visualize the 2D grid map
visualize_map(map, path, []);

% save map
% save('Data/map.mat', 'map', 'MAX_X', 'MAX_Y');
