function [neighbors,orientation,speed] = getNeighbors_Coarse(info,s)
%getNeighbors_Coarse Get all neighbor states of the current state
neighbors = [];
orientation = {};
speed = [];
num = getNum(info.orientation{s(1),s(2)});

%% Straight Ahead
p = s + info.orientation{s(1),s(2)};
if norm(p - info.StartPoint)*info.step >= 1
    p = s + 3 * info.orientation{s(1),s(2)};
    if norm(p - info.StartPoint)*info.step > 3
        p = s + 6 * info.orientation{s(1),s(2)};
    end
end
if isAccessible(info,p)
    flag = true;
    for i = 1:size(info.Blocker,1)
        if isCollide(info,p,info.orientation{s(1),s(2)},info.Blocker{i},[cosd(info.ori{i}),-sind(info.ori{i})])
            flag = false;
            break;
        end
    end
    
    if flag
        for i = 0.5:0.5:1.5
            neighbors = [neighbors;p];
            orientation{end+1} = info.orientation{s(1),s(2)};
            speed = [speed,i*info.v_desire];
        end
    end
end

%% Turn
for i = -1:2:1
    n = mod(num + i - 0.5,8) + 0.5;
    p = s + getDirection(n);
    if norm(p - info.StartPoint)*info.step >= 1
        p = s + 3 * getDirection(n);
        if norm(p - info.StartPoint)*info.step > 3
            p = s + 6 * getDirection(n);
        end
    end
    if isAccessible(info,p)
        flag = true;
        for j = 1:size(info.Blocker,1)
            if isCollide(info,p,getDirection(n),info.Blocker{j},[cosd(info.ori{j}),-sind(info.ori{j})])
                flag = false;
                break;
            end
        end
        
        if flag
            for j = 0.5:0.5:1.5
                neighbors = [neighbors;p];
                orientation{end+1} = getDirection(n);
                speed = [speed,j*info.v_desire];
            end
        end
    end
end

%% Side Walk
for i = -1:2:1
    for j = 1:2
        n = mod(num + i * j - 0.5,8) + 0.5;
        p = s + getDirection(n);
        if norm(p - info.StartPoint)*info.step >= 1
            p = s + 3 * getDirection(n);
            if norm(p - info.StartPoint)*info.step > 3
                p = s + 6 * getDirection(n);
            end
        end
        if isAccessible(info,p)
            flag = true;
            for k = 1:size(info.Blocker,1)
                if isCollide(info,p,info.orientation{s(1),s(2)},info.Blocker{k},[cosd(info.ori{k}),-sind(info.ori{k})])
                    flag = false;
                    break;
                end
            end
            
            if flag
                neighbors = [neighbors;p];
                orientation{end+1} = info.orientation{s(1),s(2)};
                speed = [speed,info.v_desire];
            end
        end
    end
end

%% Stop
neighbors = [neighbors;s];
orientation{end+1} = info.orientation{s(1),s(2)};
speed = [speed,0];
end

