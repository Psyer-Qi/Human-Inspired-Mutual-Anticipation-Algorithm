function cost = getInertiaCost(ori1,ori2)
%getInertiaCost Get the cost of turing
cost = abs(acos(ori1*ori2'/(norm(ori1)*norm(ori2))));
end

