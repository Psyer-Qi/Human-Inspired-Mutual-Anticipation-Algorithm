function [pos,ori,v,cost_all,flag] = runAAlgorithm_Coarse(info)
%runAAlgorithm_Coarse Detour moving avatars
flag = true;
while ~isempty(info.Open)
    %% Find the State from Open with the Lowest f
    min_value = inf;
    for i = 1:length(info.Open)
        if info.f(info.Open{i}(1),info.Open{i}(2)) < min_value
            min_value = info.f(info.Open{i}(1),info.Open{i}(2));
            s = info.Open{i};
        end
    end
    
    %% No Option
    if min_value == inf
        path = s;
        p = s;
        while ~isequal(p,info.StartPoint)
            p = info.parent{p(1),p(2)};
            path = [path;p];
        end
        if size(path,1) > 1 
            pos = path(end-1,:);
            ori = info.orientation{pos(1),pos(2)};
            v = info.speed(pos(1),pos(2));
            cost_all = info.g(pos(1),pos(2));
            return
        else
            disp('No Accessible Path is Found!');
            pos = s;
            ori = info.orientation{s(1),s(2)};
            v = 0;
            cost_all = inf;
            flag = false;
            return
        end
    end
    
    %% Set Grid Step Width
    dis = norm(s - info.StartPoint) * info.step;
    if dis < 1
        step = info.step;
    elseif dis < 3
        step = 3 * info.step;
    else
        step = 6 * info.step;
    end
    
    %% Reconstruct Path in Reverse from Target to Start Point with Parent Links
    if isClose(info.step*(info.Target - s),step/2) % close to the target
        path = s;
        p = s;
        while ~isequal(p,info.StartPoint)
            p = info.parent{p(1),p(2)};
            path = [path;p];
        end
        pos = path(end-1,:);
        ori = info.orientation{pos(1),pos(2)};
        v = info.speed(pos(1),pos(2));
        cost_all = info.g(pos(1),pos(2));
        return
    end
    
    %% Put s to Closed
    info.Closed{end+1} = s;
    index = findCell(info.Open,s);
    info.Open(index) = [];
    
    %% Search the Neighbors of s
    [neighbors,orientation,speed] = getNeighbors(info,s);
    for i = 1:size(neighbors,1)
        %% Calculate the Cost of a Neighbor State
        sn = neighbors(i,:);
        cost_distance = info.w_dis * norm(sn - s) * info.step; % Minimize Distance
        heuristic = norm(info.Target - sn) * info.step; % Heuristic
        cost_buffer = info.w_buffer * getBufferCost(info,sn,orientation{i},speed(i)); % Obstacle Buffer Space
        cost_space_human = 0;
        cost_space_robot = 0;
        cost_right = 0;
        for j = 1:size(info.Blocker,2)
            cost_space_human = cost_space_human + info.w_human_space * getPersonalSpaceCost(info,info.Blocker{j},info.ori{j},0,sn); % Human Personal Space
            cost_space_robot = cost_space_robot + info.w_robot_space * getPersonalSpaceCost(info,sn,45*(getNum(orientation{i})-1),speed(i),info.Blocker{j}); % Robot Personal Space
            cost_right = cost_right + info.w_right * getSocialNormCost(info,info.Blocker{j},info.ori{j},sn); % Pass on Right
        end
        if speed(i) == 0 % speed == 0
            cost_velocity = info.w_v * info.step * norm(info.direction{s(1),s(2)})/info.speed(s(1),s(2)) * info.v_desire; % Default Velocity
        else
            cost_velocity = info.w_v * info.step * norm(sn-s)/speed(i)*abs(info.v_desire-speed(i)); % Default Velocity
        end
        cost_direction = info.w_face * getDirectionCost(info,s,sn,orientation{i},speed(i)); % Face Direction
        cost_inertia = info.w_inertia * getInertiaCost(info.orientation{s(1),s(2)},orientation{i}); % Inertia
        cost = info.g(s(1),s(2)) + cost_distance + cost_buffer + cost_space_human + cost_space_robot + cost_right + cost_velocity + cost_direction + cost_inertia;
        
        %% Remove the Old State if a Better Path is Found
        if findCell(info.Open,sn) ~= 0 && info.g(sn(1),sn(2)) > cost
            index = findCell(info.Open,sn);
            info.Open(index) = [];
        end
        
        %% Update a New State
        if findCell(info.Open,sn) == 0 && findCell(info.Closed,sn) == 0
            info.direction{sn(1),sn(2)} = sn - s;
            info.orientation{sn(1),sn(2)} = orientation{i};
            info.speed(sn(1),sn(2)) = speed(i);
            info.g(sn(1),sn(2)) = cost;
            info.f(sn(1),sn(2)) = cost + heuristic;
            info.Open{end+1} = sn;
            info.parent{sn(1),sn(2)} = s;
        end
    end 
end
disp('No Accessible Path is Found!');
pos = s;
ori = info.orientation{s(1),s(2)};
v = 0;
cost_all = inf;
flag = false;
end

