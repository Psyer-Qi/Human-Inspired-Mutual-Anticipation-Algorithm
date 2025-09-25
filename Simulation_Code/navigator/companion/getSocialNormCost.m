function cost = getSocialNormCost(info,pos,ori,point)
%getSocialNormCost Get the cost of passing from right
d_front = 2;
d_side = 0.25;
d_rear = 0.01;

theta = (ori+90)*pi/180; % right side

cost = getAsyGaussian(pos*info.step,theta,d_front,d_side,d_rear,point*info.step);
end

