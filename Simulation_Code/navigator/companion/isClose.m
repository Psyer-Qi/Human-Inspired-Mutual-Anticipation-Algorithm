function flag = isClose(vec,d)
flag = false;
if abs(vec(1)) < d && abs(vec(2)) < d
    flag = true;
end
end

