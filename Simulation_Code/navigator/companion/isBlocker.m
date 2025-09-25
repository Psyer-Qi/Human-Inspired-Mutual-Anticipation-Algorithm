function [Blocker,space] = isBlocker(info,withhuman)
%isHuman List all points in a map that belong to a Blocker
Blocker = cell(0);
space = cell(0);

xmin_list = [];xmax_list = [];
ymin_list = [];ymax_list = [];
for i = 1:size(info.Blocker,2)
    xmin_list = [xmin_list,floor(info.Blocker{i}(1) - (info.blocker_a(i) + info.robot_a)/info.step)];
    ymin_list = [ymin_list,floor(info.Blocker{i}(2) - (info.blocker_a(i) + info.robot_a)/info.step)];
    xmax_list = [xmax_list,floor(info.Blocker{i}(1) + (info.blocker_a(i) + info.robot_a)/info.step)];
    ymax_list = [ymax_list,floor(info.Blocker{i}(2) + (info.blocker_a(i) + info.robot_a)/info.step)];
end
[max_x,max_y] = size(info.g);

%% Search Part of the Whole Map
if withhuman == 0
    for i = 1:size(xmin_list,2)
        for x = max(min(xmin_list(i)),1):min(max(xmax_list(i)),max_x)
            for y = max(min(ymin_list(i)),1):min(max(ymax_list(i)),max_y)
                space{end+1} = [x,y];
                Blocker{end+1} = [x,y];
            end
        end
    end
else
    for i = 1:size(xmin_list,2)-1 % the last blocker is human;
        for x = max(min(xmin_list(i)),1):min(max(xmax_list(i)),max_x)
            for y = max(min(ymin_list(i)),1):min(max(ymax_list(i)),max_y)
                space{end+1} = [x,y];
                Blocker{end+1} = [x,y];
            end
        end
    end
    % add the points belonging to human;
    for x = max(min(xmin_list(size(xmin_list,2))),1):min(max(xmax_list(size(xmin_list,2))),max_x)
        for y = max(min(ymin_list(size(xmin_list,2))),1):min(max(ymax_list(size(xmin_list,2))),max_y)
            flag_space = false;
            flag_human = false;

            vec_y = [cosd(info.ori{size(xmin_list,2)}),-sind(info.ori{size(xmin_list,2)})];
            vec_x = [sind(info.ori{size(xmin_list,2)}),cosd(info.ori{size(xmin_list,2)})];
            p = info.step*([x,y] - info.Blocker{size(xmin_list,2)})/[vec_x;vec_y]; % relative position to the human
            if norm(p) - info.robot_a > 0
                s = (norm(p) - info.robot_a)/norm(p)*p;
                if (s(1)/info.blocker_a(size(xmin_list,2))^2 + (s(2)/info.blocker_b(size(xmin_list,2))))^2 <= 1 % see if the point is inside the oval
                    flag_space = true;
                    if (p(1)/info.blocker_a(size(xmin_list,2)))^2 + (p(2)/info.blocker_b(size(xmin_list,2)))^2 <= 1 % see if the point is inside the oval
                        flag_human = true;
                    end
                end
            else
                flag_space = true;
                flag_human = true;
            end

            if flag_space
                space{end+1} = [x,y];
            end

            if flag_human
                Blocker{end+1} = [x,y];
            end
        end
    end
end
end

