clc;
clear;
close all;

posStart = [1; 1; 1];
posTarget = [9; 9; 9];
MAX_X = 10;
MAX_Y = 10;
MAX_Z = 10;

map = obstacle_map(posStart, posTarget, MAX_X, MAX_Y, MAX_Z);
data = load('map.mat');
map = data.map;
path = A_star_search(map, MAX_X, MAX_Y, MAX_Z);
visualize_map(map, path, []);