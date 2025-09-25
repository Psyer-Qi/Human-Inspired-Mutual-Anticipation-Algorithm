function RMSE = runExp4(weights)
%runExp4 Calculate the RMSE of the Model Prediction & Experimental Data
%with Given Weights
%% Define the Experiment Condition
info.width = 3; % The largest lateral distance possible (m) 
info.length = 5; % The largest vertical distance possible (m)
info.step = 0.1; % The distance between two connnected nodes (m)
info.StartPoint = [info.width/info.step,1];
info.Target = [info.width/info.step,info.length/info.step + 1];
info.Blocker = {[info.width/info.step,info.length/info.step/2 + 1]};
info.v_desire = 1.36; % (m/s)
info.blocker_a = 0.24; % (m)
info.blocker_b = 0.12; % (m)
info.robot_a = 0.24; % (m)
info.robot_b = 0.12; % (m)

%% Define the Weights
info.w_dis = weights(1);
info.w_buffer = weights(2);
info.w_human_space = weights(3);
info.w_robot_space = weights(4);
info.w_right = weights(5);
info.w_v = weights(6);
info.w_face = weights(7);
info.w_inertia = weights(8);

%% Model the Route Given Blocker's Orientation
route_all = [];
for ori = 0:30:330 % 0:facing right side; 90:facing the participant
    info.ori = {ori};
    
    %% Define the Initial Values 
    info.g = zeros(2*info.width/info.step - 1,info.length/info.step + 1);
    info.f = zeros(2*info.width/info.step - 1,info.length/info.step + 1);
    info.direction = cell(2*info.width/info.step - 1,info.length/info.step + 1);
    info.orientation = cell(2*info.width/info.step - 1,info.length/info.step + 1);
    info.speed = nan(2*info.width/info.step - 1,info.length/info.step + 1);
    info.Open = {info.StartPoint};
    info.Closed = {};
    info.parent = cell(2*info.width/info.step - 1,info.length/info.step + 1);
    info.g(1:info.width/info.step,info.length/info.step/2 + 1) = inf; % never passing from the left side
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
    temp = cell(51,1);
    for i = 1:size(points,1)
        temp{points(i,2)} = [temp{points(i,2)},points(i,1)-info.width/info.step];
    end
    temp1 = nan(51,2);
    for i = 1:51
        temp1(i,1) = mean(temp{i})/info.step;
    end
    temp1(:,2) = 0:10:500;
    route = interp1(temp1(:,2),temp1(:,1),1:470);
    route_all = [route_all;route'];
end

%% Import Data
data = readmatrix('/Users/miao/Desktop/COMPANION framework fit/Data/Exp4_Route.xlsx');
path = data(2:5641,4);

%% Calculate the Weights
RMSE = sqrt(mean((route_all - path).^2));
end

