function mat = pythonCell2Mat(cellArray)
    mat = [];
    for i=1:size(cellArray, 2)
        mat = [mat; [cellArray{i}{1}, cellArray{i}{2}]];
    end
end