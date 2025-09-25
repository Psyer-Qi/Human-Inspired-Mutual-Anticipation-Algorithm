function vec = getDirection(num)
%getDirection Get the direction from a number
vector = [1,0;1,1;0,1;-1,1;-1,0;-1,-1;0,-1;1,-1];
vec = vector(num,:);
end

