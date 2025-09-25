clear;clc;
load('test1.mat');

path = getPath(mapWidth, Start, Target, Blocker, Blocker_Ori, Blocker_Sizes, isWithHuman);

disp('done');