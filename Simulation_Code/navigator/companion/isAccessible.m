function flag = isAccessible(info,p)
%isAccessible Judge if p is a point in the map
flag = true;
[max_x,max_y,~,~] = size(info.g);
if p(1) < 1 || p(1) > max_x || p(2) < 1 || p(2) > max_y || info.g(p(1),p(2)) == inf
    flag = false;
end
end

