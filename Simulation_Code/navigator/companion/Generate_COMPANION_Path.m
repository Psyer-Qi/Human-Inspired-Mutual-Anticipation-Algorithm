%% Generate the path using COMPANION with ToM-Level1;

% =========================================
%                 设置初始条件
% =========================================

clc;clear;
y_more  = 100; % empty space in the real environment in y axis(cm);
Start = [0,y_more/100]; % The position of the start point
Target = [6,3.5+y_more/100]; % The position of the end point
Start_Sub = [6,3.5+y_more/100]; % The position of the start point of subject;
Target_Sub = [0,y_more/100]; % The position of the end point of subject;
mapWidth = 7; % How wide is the map
Blocker = [3.4,1.35+y_more/100]; % The position of the Blocker;
Blocker_Ori = 0; % The orientation of the Blocker;
a_R = 50; % the width of the Blocker from top view;
b_R = 50; % the length of the Blocker from top view;

% =========================================
% 1.下面计算人的路径
% =========================================

% Define the Experiment Condition of subject;
info_Sub.width = mapWidth; % The largest lateral distance possible (m) 
info_Sub.length = mapWidth; % The largest vertical distance possible (m)
info_Sub.step = 0.125; % The distance between two connnected nodes (m)
info_Sub.StartPoint = [(info_Sub.width + Start_Sub(1))/info_Sub.step, Start_Sub(2)/info_Sub.step + 1];
info_Sub.Target = [(info_Sub.width + Target_Sub(1))/info_Sub.step, Target_Sub(2)/info_Sub.step + 1];
info_Sub.Blocker_v = {0};
info_Sub.v_desire = 1.36; % (m/s)
info_Sub.blocker_a = b_R/200; % (m)
info_Sub.blocker_b = a_R/200; % (m)
info_Sub.robot_a = 0.24; % (m)
info_Sub.robot_b = 0.12; % (m)

% Define the Weights;
info_Sub.w_dis = 1;
info_Sub.w_buffer = 0.4;
info_Sub.w_human_space = 0;
info_Sub.w_robot_space = 0;
info_Sub.w_right = 0;
info_Sub.w_v = 0;
info_Sub.w_face = 0;
info_Sub.w_inertia = 0;

% Model the Route Given Barrier's Position & Orientation;
info_Sub.ori = {Blocker_Ori};
info_Sub.Blocker = {[(info_Sub.width + Blocker(1))/info_Sub.step, Blocker(2)/info_Sub.step + 1]};

% Define the Initial Values;
info_Sub.g = zeros(2*info_Sub.width/info_Sub.step - 1,info_Sub.length/info_Sub.step + 1);
info_Sub.f = zeros(2*info_Sub.width/info_Sub.step - 1,info_Sub.length/info_Sub.step + 1);
info_Sub.direction = cell(2*info_Sub.width/info_Sub.step - 1,info_Sub.length/info_Sub.step + 1);
info_Sub.orientation = cell(2*info_Sub.width/info_Sub.step - 1,info_Sub.length/info_Sub.step + 1);
info_Sub.speed = nan(2*info_Sub.width/info_Sub.step - 1,info_Sub.length/info_Sub.step + 1);
info_Sub.Open = {info_Sub.StartPoint};
info_Sub.Closed = {};
info_Sub.parent = cell(2*info_Sub.width/info_Sub.step - 1,info_Sub.length/info_Sub.step + 1);
info_Sub.Blocker_all = cell(0);
info_Sub.human = cell(0);
withhuman = 0; % there is no person in the environment as a blocker;

info_Sub.f(info_Sub.StartPoint(1),info_Sub.StartPoint(2)) = norm(info_Sub.Target - info_Sub.StartPoint);
info_Sub.direction{info_Sub.StartPoint(1),info_Sub.StartPoint(2)} = [-1,0];
info_Sub.orientation{info_Sub.StartPoint(1),info_Sub.StartPoint(2)} = [-1,0];
info_Sub.speed(info_Sub.StartPoint(1),info_Sub.StartPoint(2)) = info_Sub.v_desire;

[info_Sub.Blocker_all, info_Sub.space] = isBlocker(info_Sub,withhuman); % get all the points that belong to the blocker
% define all these points inaccessible
for i = 1:length(info_Sub.Blocker_all)
    info_Sub.g(info_Sub.Blocker_all{i}(1),info_Sub.Blocker_all{i}(2)) = inf;
end

% Run A* Algorithm;
[points,~,~] = runAAlgorithm(info_Sub);

% Convert the Route;
temp = cell(Target(1)/info_Sub.step+1,2);
for i = 1:size(points,1)
    temp{(points(i,1)-info_Sub.width/info_Sub.step+1),1} = [temp{(points(i,1)-info_Sub.width/info_Sub.step+1),1},points(i,1)-info_Sub.width/info_Sub.step];
    temp{(points(i,1)-info_Sub.width/info_Sub.step+1),2} = points(i,2)-1;
end
temp1 = zeros(Target(1)/info_Sub.step+1,2);
for i = 1:Target(1)/info_Sub.step+1
    temp1(i,1) = 100*temp{i,1}*info_Sub.step;
    temp1(i,2) = 100*temp{i,2}*info_Sub.step;
end
route = interp1(temp1(:,1),temp1(:,2),600:-1:11);
path_Sub = [600:-1:11;route]';

% =========================================
% 2.下面计算机器人的路径
% =========================================

% Define the Experiment Condition of robot;
info.width = mapWidth; % The largest lateral distance possible (m) 
info.length = mapWidth; % The largest vertical distance possible (m)
info.step = 0.125; % The distance between two connnected nodes (m)
info.StartPoint = [(info.width + Start(1))/info.step, Start(2)/info.step + 1];
info.Target = [(info.width + Target(1))/info.step, Target(2)/info.step + 1];
info.Blocker_v = {0};
info.v_desire = 1.36; % (m/s)
info.blocker_a = b_R/200; % (m)
info.blocker_b = a_R/200; % (m)
info.robot_a = 0.24; % (m)
info.robot_b = 0.12; % (m)

% Define the Weights;
info.w_dis = 1;
info.w_buffer = 0.4;
info.w_human_space = 0;
info.w_robot_space = 0;
info.w_right = 0;
info.w_v = 0;
info.w_face = 0;
info.w_inertia = 0;

% Model the Route Given Barrier's Position & Orientation;
info.ori = {Blocker_Ori};
info.Blocker = {[(info.width + Blocker(1))/info.step, Blocker(2)/info.step + 1]};

% Define the Initial Values;
info.g = zeros(2*info.width/info.step - 1,info.length/info.step + 1);
info.f = zeros(2*info.width/info.step - 1,info.length/info.step + 1);
info.direction = cell(2*info.width/info.step - 1,info.length/info.step + 1);
info.orientation = cell(2*info.width/info.step - 1,info.length/info.step + 1);
info.speed = nan(2*info.width/info.step - 1,info.length/info.step + 1);
info.Open = {info.StartPoint};
info.Closed = {};
info.parent = cell(2*info.width/info.step - 1,info.length/info.step + 1);
info.Blocker_all = cell(0);
info.human = cell(0);
withhuman = 0; % there is no person in the environment as a blocker;

info.f(info.StartPoint(1),info.StartPoint(2)) = norm(info.Target - info.StartPoint);
info.direction{info.StartPoint(1),info.StartPoint(2)} = [1,0];
info.orientation{info.StartPoint(1),info.StartPoint(2)} = [1,0];
info.speed(info.StartPoint(1),info.StartPoint(2)) = info.v_desire;

[info.Blocker_all,info.space] = isBlocker(info,withhuman); % get all the points that belong to the blocker
% define all these points inaccessible
for i = 1:length(info.Blocker_all)
    info.g(info.Blocker_all{i}(1),info.Blocker_all{i}(2)) = inf;
end

% Run A* Algorithm;
[points,~,~] = runAAlgorithm(info);

% Convert the Route;
temp = cell(Target(1)/info.step+1,2);
for i = 1:size(points,1)
    temp{(points(i,1)-info.width/info.step+1),1} = [temp{(points(i,1)-info.width/info.step+1),1},points(i,1)-info.width/info.step];
    temp{(points(i,1)-info.width/info.step+1),2} = points(i,2)-1;
end
temp1 = zeros(Target(1)/info.step+1,2);
for i = 1:Target(1)/info.step+1
    temp1(i,1) = 100*temp{i,1}*info.step;
    temp1(i,2) = 100*temp{i,2}*info.step;
end
route = interp1(temp1(:,1),temp1(:,2),1:590);
path_Robot = [1:590;route]';


% =========================================
% 3.下面计算 最近碰撞点
% =========================================


% calculate the nearest point between the path_Sub and path_Robot;
d = path_Robot - path_Sub;
d1 = d(:,1).^2 + d(:,2).^2;
Block_Person = path_Sub(d1==min(d1),:);
Block_Person = Block_Person(1,:)./100;
Cal_Ori = diff(path_Sub);
Cal_Ori(:,3) = atan(Cal_Ori(:,2)./Cal_Ori(:,1));
Cal_Ori(:,3) = rad2deg(Cal_Ori(:,3));
Cal_Ori = [[0 0 0];Cal_Ori];
Cal_Ori(:,3) = Cal_Ori(:,3) - 90; % 0 means the avatar is facing the participant, and 90 means it is facing the right side;
Block_Ori = Cal_Ori(d1==min(d1),3);
Block_Ori = Block_Ori(1,:);


% =========================================
% 4.下面计算 更新后的路径（考虑到了碰撞点的情况）
% =========================================

% Re-generate the robot path: Define the Experiment Condition of robot;
clear info;
info.width = mapWidth; % The largest lateral distance possible (m) 
info.length = mapWidth; % The largest vertical distance possible (m)
info.step = 0.125; % The distance between two connnected nodes (m)
info.StartPoint = [(info.width + Start(1))/info.step, Start(2)/info.step + 1];
info.Target = [(info.width + Target(1))/info.step, Target(2)/info.step + 1];
info.Blocker_v = {0,0};
info.v_desire = 1.36; % (m/s)
info.blocker_a = [b_R/200,0.24]; % (m)
info.blocker_b = [a_R/200,0.12]; % (m)
info.robot_a = 0.24; % (m)
info.robot_b = 0.12; % (m)

% Define the Weights;
info.w_dis = 1;
info.w_buffer = 0.4;
info.w_human_space = 0;
info.w_robot_space = 0;
info.w_right = 0;
info.w_v = 0;
info.w_face = 0;
info.w_inertia = 0;

% Model the Route Given Barrier's Position & Orientation;
info.ori = {Blocker_Ori,Block_Ori};
info.Blocker = {[(info.width + Blocker(1))/info.step, Blocker(2)/info.step + 1],[(info.width + Block_Person(1))/info.step, Block_Person(2)/info.step + 1]};

% Define the Initial Values;
info.g = zeros(2*info.width/info.step - 1,info.length/info.step + 1);
info.f = zeros(2*info.width/info.step - 1,info.length/info.step + 1);
info.direction = cell(2*info.width/info.step - 1,info.length/info.step + 1);
info.orientation = cell(2*info.width/info.step - 1,info.length/info.step + 1);
info.speed = nan(2*info.width/info.step - 1,info.length/info.step + 1);
info.Open = {info.StartPoint};
info.Closed = {};
info.parent = cell(2*info.width/info.step - 1,info.length/info.step + 1);
info.Blocker_all = cell(0);
info.human = cell(0);
withhuman = 1; % there is a person in the environment as a blocker, with is the last blocker;

info.f(info.StartPoint(1),info.StartPoint(2)) = norm(info.Target - info.StartPoint);
info.direction{info.StartPoint(1),info.StartPoint(2)} = [1,0];
info.orientation{info.StartPoint(1),info.StartPoint(2)} = [1,0];
info.speed(info.StartPoint(1),info.StartPoint(2)) = info.v_desire;

[info.Blocker_all,info.space] = isBlocker(info,withhuman); % get all the points that belong to the blocker
% define all these points inaccessible
for i = 1:length(info.Blocker_all)
    info.g(info.Blocker_all{i}(1),info.Blocker_all{i}(2)) = inf;
end

% Run A* Algorithm;
[points,~,~] = runAAlgorithm(info);

% Convert the Route;
temp = cell(size(points,1),2);
for i = 1:size(points,1)
    temp{i,1} = points(i,1) - info.width/info.step;
    temp{i,2} = points(i,2)-1;
end
temp1 = zeros(size(points,1),2);
for i = 1:size(points,1)
    temp1(i,1) = 100*temp{i,1}*info.step;
    temp1(i,2) = 100*temp{i,2}*info.step;
end
temp2 = temp1(end:-1:1,:);
route = interp1(temp2(:,1),temp2(:,2),1:590);
path = [1:590;route;]';

path(:,2) = path(:,2) - y_more;
disp('Have generated path using COMPANION with ToM-Level1!');
dlmwrite("Path_COMPANION_with_ToM1.csv",path);