function [neighbors,orientation,speed] = getNeighbors(info,s)
%getNeighbors Get all neighbor states of the current state
neighbors = [];
orientation = {};
speed = [];
num = getNum(info.orientation{s(1),s(2)});

%% Straight Ahead
p = s + info.orientation{s(1),s(2)};
if isAccessible(info,p)
    flag = true;
    for i = 1:size(info.Blocker,2)
        if isCollide(info,p,info.orientation{s(1),s(2)},info.Blocker{i},info.blocker_a(i),info.blocker_b(i),[cosd(info.ori{i}),-sind(info.ori{i})])
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
    if isAccessible(info,p)
        flag = true;
        for j = 1:size(info.Blocker,2)
            if isCollide(info,p,getDirection(n),info.Blocker{j},info.blocker_a(j),info.blocker_b(j),[cosd(info.ori{j}),-sind(info.ori{j})])
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
        if isAccessible(info,p)
            flag = true;
            for k = 1:size(info.Blocker,2)
                if isCollide(info,p,info.orientation{s(1),s(2)},info.Blocker{k},info.blocker_a(k),info.blocker_b(k),[cosd(info.ori{k}),-sind(info.ori{k})])
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

