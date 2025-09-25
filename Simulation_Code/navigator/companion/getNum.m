function num = getNum(vec)
%getNum Get the sequence from a vector
d = norm(vec);
ori = acosd(vec(1)/d);
if vec(2) < 0
    ori = 360 - ori;
end
num = ori/45 + 1;
end

