function flag = isCollide(info,s1,ori1,s2,blocker_a,blocker_b,ori2)
%isCollide Judge whether two ovals collides
xmin_list = [floor(s1(1) - info.robot_a/info.step),floor(s2(1) - blocker_a/info.step)];
ymin_list = [floor(s1(2) - info.robot_a/info.step),floor(s2(2) - blocker_a/info.step)];
xmax_list = [floor(s1(1) + info.robot_a/info.step),floor(s2(1) + blocker_a/info.step)];
ymax_list = [floor(s1(2) + info.robot_a/info.step),floor(s2(2) + blocker_a/info.step)];
[max_x,max_y] = size(info.g);

%% Search Part of the Whole Map
flag = false;
for x = max(min(xmin_list),1):min(max(xmax_list),max_x)
    for y = max(min(ymin_list),1):min(max(ymax_list),max_y)
        flag1 = 0;
        flag2 = 0;
        
        vec_y = ori1/norm(ori1);
        vec_x = [vec_y(2),-vec_y(1)];
        p = info.step*([x,y] - s1)/[vec_x;vec_y];
        if (p(1)/info.robot_a)^2 + (p(2)/info.robot_b)^2 <= 1 % see if the point is inside the oval
            flag1 = 1;
        end
        
        vec_y = ori2/norm(ori2);
        vec_x = [vec_y(2),-vec_y(1)];
        p = info.step*([x,y] - s2)/[vec_x;vec_y];
        if (p(1)/blocker_a)^2 + (p(2)/blocker_b)^2 <= 1 % see if the point is inside the oval
            flag2 = 1;
        end
        
        if flag1*flag2 == 1
            flag = true;
            return
        end
    end
end
end

