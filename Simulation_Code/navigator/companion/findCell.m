function index = findCell(source,array)
%findCell Find where the source matches the array 
index = 0;
for i = 1:length(source)
    if isequal(source{i},array)
        index = i;
        return
    end
end
end

