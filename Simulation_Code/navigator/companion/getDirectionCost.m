function cost = getDirectionCost(info,s1,s2,ori,speed)
%getDirectionCost Get the cost of sidewalking
if speed == 0
    cost = 0;
    return
end
t = info.step * norm(s2-s1)/speed;
vec1 = info.orientation{s1(1),s1(2)}/norm(info.orientation{s1(1),s1(2)});
vec2 = [vec1(2),-vec1(1)];
vec = speed*ori/norm(ori);
v = vec/[vec1;vec2];
cost = t*abs(v(2));
end