function path = generatePath_COMPANIAN(mapWidth,Start,Target,Blocker,Blocker_Ori)
Start = [2.5,0]; % The position of the start point
Target = [2.5,5]; % The position of the end point
mapWidth = 5; % How wide is the map
Blocker = [2.2,2.5]; % The position of the blocking avatar
Blocker_Ori = 90; % The orientation of the blocking avatar (deg)

%% Define the Experiment Condition
info.width = mapWidth; % The largest lateral distance possible (m) 
info.length = Target(2); % The largest vertical distance possible (m)
info.step = 0.125; % The distance between two connnected nodes (m)
info.StartPoint = [(info.width + Start(1))/info.step, Start(2)/info.step + 1];
info.Target = [(info.width + Target(1))/info.step, Target(2)/info.step + 1];
info.Blocker_v = {0};
info.v_desire = 1.36; % (m/s)
info.blocker_a = 0.24; % (m)
info.blocker_b = 0.12; % (m)
info.robot_a = 0.24; % (m)
info.robot_b = 0.12; % (m)

%% Define the Weights
info.w_dis = 1;
info.w_buffer = 0.4;
info.w_human_space = 0;
info.w_robot_space = 0;
info.w_right = 0;
info.w_v = 0;
info.w_face = 0;
info.w_inertia = 0;

%% Model the Route Given Blocker's Position & Orientation
route_all = [];
data = [];
info.ori = {Blocker_Ori};
info.Blocker = {[(info.width + Blocker(1))/info.step, Blocker(2)/info.step + 1]};

%% Define the Initial Values
info.g = zeros(2*info.width/info.step - 1,info.length/info.step + 1);
info.f = zeros(2*info.width/info.step - 1,info.length/info.step + 1);
info.direction = cell(2*info.width/info.step - 1,info.length/info.step + 1);
info.orientation = cell(2*info.width/info.step - 1,info.length/info.step + 1);
info.speed = nan(2*info.width/info.step - 1,info.length/info.step + 1);
info.Open = {info.StartPoint};
info.Closed = {};
info.parent = cell(2*info.width/info.step - 1,info.length/info.step + 1);
info.f(info.StartPoint(1),info.StartPoint(2)) = norm(info.Target - info.StartPoint);
info.direction{info.StartPoint(1),info.StartPoint(2)} = [0,1];
info.orientation{info.StartPoint(1),info.StartPoint(2)} = [0,1];
info.speed(info.StartPoint(1),info.StartPoint(2)) = info.v_desire;

[info.human,info.space] = isHuman(info); % get all the points that belong to the blocker
% define all these points inaccessible
for i = 1:length(info.human)
    info.g(info.human{i}(1),info.human{i}(2)) = inf;
end

%% Run A* Algorithm
[points,~,~] = runAAlgorithm(info);

%% Convert the Route
temp = cell(info.length/info.step+1,1);
for i = 1:size(points,1)
    temp{points(i,2)} = [temp{points(i,2)},points(i,1)-info.width/info.step];
end
temp1 = zeros(info.length/info.step + 1,2);
for i = 1:info.length/info.step + 1
    temp1(i,1) = 100*mean(temp{i})*info.step;
end
temp1(:,2) = 0:100*info.step:100*info.length;
route = interp1(temp1(:,2),temp1(:,1),1:100*info.length);
path = [route;1:100*info.length]';
end

