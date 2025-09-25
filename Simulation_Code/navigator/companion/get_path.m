function path = get_path(mapWidth, Start, Target, Blocker, Blocker_Ori, Blocker_Sizes, isWithHuman)

% Signature of function getPath
%   getPath(mapWidth, Start, Target, Blocker, Blocker_Ori, Blocker_Sizes, isWithHuman)
% save('test.mat');

Start = double(cell2mat(Start));
Target = double(cell2mat(Target));

Blocker = double(pythonCell2Mat(Blocker));
Blocker_Ori = double(cell2mat(Blocker_Ori));
Blocker_Sizes = double(pythonCell2Mat(Blocker_Sizes));

% save('test1.mat');

path = getPath(mapWidth, Start, Target, Blocker, Blocker_Ori, Blocker_Sizes, isWithHuman);

end