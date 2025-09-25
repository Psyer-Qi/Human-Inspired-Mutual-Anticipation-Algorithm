function [human,space] = isHuman(info)
%isHuman List all points in a map that belong to a human
human = cell(0);
space = cell(0);

xmin_list = [];xmax_list = [];
ymin_list = [];ymax_list = [];
for i = 1:size(info.Blocker,2)
    xmin_list = [xmin_list,floor(info.Blocker{i}(1) - (info.blocker_a + info.robot_a)/info.step)];
    ymin_list = [ymin_list,floor(info.Blocker{i}(2) - (info.blocker_a + info.robot_a)/info.step)];
    xmax_list = [xmax_list,floor(info.Blocker{i}(1) + (info.blocker_a + info.robot_a)/info.step)];
    ymax_list = [ymax_list,floor(info.Blocker{i}(2) + (info.blocker_a + info.robot_a)/info.step)];
end
[max_x,max_y] = size(info.g);

%% Search Part of the Whole Map
for x = max(min(xmin_list),1):min(max(xmax_list),max_x)
    for y = max(min(ymin_list),1):min(max(ymax_list),max_y)
        flag_space = false;
        flag_human = false;
        
        for i = 1:size(info.Blocker,2)
            vec_y = [cosd(info.ori{i}),-sind(info.ori{i})];
            vec_x = [sind(info.ori{i}),cosd(info.ori{i})];
            p = info.step*([x,y] - info.Blocker{i})/[vec_x;vec_y]; % relative position to the human
            if norm(p) - info.robot_a > 0
                s = (norm(p) - info.robot_a)/norm(p)*p;
                if (s(1)/info.blocker_a)^2 + (s(2)/info.blocker_b)^2 <= 1 % see if the point is inside the oval
                    flag_space = true;
                    if (p(1)/info.blocker_a)^2 + (p(2)/info.blocker_b)^2 <= 1 % see if the point is inside the oval
                        flag_human = true;
                    end
                end
            else
                flag_space = true;
                flag_human = true;
            end
        end
        
        if flag_space
            space{end+1} = [x,y];
        end
        
        if flag_human
            human{end+1} = [x,y];
        end
    end
end
end

