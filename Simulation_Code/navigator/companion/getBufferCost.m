function cost = getBufferCost(info,s,ori,speed)
%getBufferCost Get the cost of the Buffer Space
cost = 0;
if size(info.Blocker_all,2)>1
    for i = 1:length(info.Blocker_all)
        c = getAsyGaussian(s*info.step,(getNum(ori)-1)*pi/4,speed,speed/6,speed/6,info.Blocker_all{i}*info.step);
        if c > cost
            cost = c;
        end
    end
else
    for i = 1:length(info.human)
        c = getAsyGaussian(s*info.step,(getNum(ori)-1)*pi/4,speed,speed/6,speed/6,info.human{i}*info.step);
        if c > cost
            cost = c;
        end
    end
end
end