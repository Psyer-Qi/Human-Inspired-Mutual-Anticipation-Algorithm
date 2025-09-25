function cost = getPersonalSpaceCost(info,pos,ori,speed,point)
%getPersonalSpaceCost Get the cost of Personal Space
d_front = max(2*speed,0.5);
d_side = 2*d_front/3;
d_rear = d_front/2;

theta = ori*pi/180;

cost = getAsyGaussian(pos*info.step,theta,d_front,d_side,d_rear,point*info.step);
end

