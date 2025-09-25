function value = getAsyGaussian(center,theta,d_front,d_side,d_rear,pos)
%getAsyGaussian Get the value of a position in the given asymmetric
%gaussian function
alpha = pi/2 - acos((pos-center)*[cos(theta),sin(theta)]'/norm(pos-center));
if alpha > 0
    delta = d_front;
else
    delta = d_rear;
end

a = cos(theta)^2/(2*delta^2) + sin(theta)^2/(2*d_side^2);
b = sin(2*theta)/(4*delta^2) - sin(2*theta)/(4*d_side^2);
c = sin(theta)^2/(2*delta^2) + cos(theta)^2/(2*d_side^2);

value = exp(-(a*(pos(1)-center(1))^2 + 2*b*(pos(1)-center(1))*(pos(2)-center(2)) + c*(pos(2)-center(2))^2));
end

