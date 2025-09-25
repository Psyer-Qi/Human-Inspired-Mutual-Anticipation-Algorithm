function path = getPath(mapWidth, Start, Target, Blocker, Blocker_Ori, Blocker_Sizes, isWithHuman)

orientation_vec = Target - Start;
min_delta = inf;
for i = 0:45:315
    ori_temp = [cosd(i), sind(i)];
    if sum(abs(ori_temp)) > 1
        ori_temp = ori_temp/cosd(45);
    end
    delta_alpha = acosd(sum(ori_temp .* orientation_vec)/(norm(ori_temp) * norm(orientation_vec)));
    if delta_alpha < min_delta
        min_delta = delta_alpha;
        ori_init = ori_temp;
    end
end



% -----------------------------------------
% 基本参数
% -----------------------------------------

info.width = mapWidth; % The largest lateral distance possible (m)
info.length = mapWidth; % The largest vertical distance possible (m)

info.step = 0.1; % The distance between two connnected nodes (m)

info.StartPoint = [floor((info.width + Start(1))/info.step), floor(Start(2)/info.step + 1)];
info.Target = [floor((info.width + Target(1))/info.step), floor(Target(2)/info.step + 1)];

%info.Blocker_v = {0, 0};
info.v_desire = 1.36; % (m/s)
numOfStaticBlockers = size(Blocker, 1);
if isWithHuman == 1
    numOfStaticBlockers = numOfStaticBlockers - 1;
end
info.Blocker_v = cell(0);
for i = 1:numOfStaticBlockers
    info.Blocker_v{i} = 0;
end
if isWithHuman == 1
    info.Blocker_v{numOfStaticBlockers + 1} = 1.36;
end

% info.blocker_a = [b_R/200,0.24]; % (m)
% info.blocker_b = [a_R/200,0.12]; % (m)

info.blocker_a = Blocker_Sizes(:, 1)';
info.blocker_b = Blocker_Sizes(:, 2)';

info.robot_a = 0.24; % (m)
info.robot_b = 0.12; % (m)

% -----------------------------------------
% 定义算法权重
% -----------------------------------------
info.w_dis = 1;
info.w_buffer = 0.4;
info.w_human_space = 0;
info.w_robot_space = 0;
info.w_right = 0;
info.w_v = 0;
info.w_face = 0;
info.w_inertia = 0;

% -----------------------------------------
% 前置计算
% -----------------------------------------

% Model the Route Given Barrier's Position & Orientation;
info.ori = cell(0);
for i = 1:size(Blocker_Ori, 2)
    info.ori{i} = Blocker_Ori(i);
end
% info.Blocker = {[(info.width + Blocker(1))/info.step, Blocker(2)/info.step + 1],[(info.width + Block_Person(1))/info.step, Block_Person(2)/info.step + 1]};
info.Blocker = cell(0);
for i = 1:size(Blocker, 1)
    info.Blocker{i} = [(info.width + Blocker(i,1))/info.step, Blocker(i,2)/info.step + 1];
end


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

p_x = info.StartPoint(1);
p_y = info.StartPoint(2);

info.f(p_x, p_y) = norm(info.Target - info.StartPoint);
info.direction{p_x, p_y} = ori_init;
info.orientation{p_x, p_y} = ori_init;
info.speed(p_x, p_y) = info.v_desire;

[info.Blocker_all, info.space] = isBlocker(info, isWithHuman); % get all the points that belong to the blocker

% define all these points inaccessible
for i = 1:length(info.Blocker_all)
    info.g(info.Blocker_all{i}(1), info.Blocker_all{i}(2)) = inf;
end
% -----------------------------------------
% Run A* Algorithm;
% -----------------------------------------
[points, ~, flag] = runAAlgorithm(info);

% -----------------------------------------
% 路径插值
% -----------------------------------------
if flag
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
    path = temp1(end:-1:1,:);
else
    path = [0, 0];
end

end