%% 
% ------------------------------
%       一个障碍物
% ------------------------------
clear; clc;

Start = [4.18, 7.5];
Target = [4, 0.5];

mapWidth = 8; 

Blocker_Positions = [2.0, 2.0]; 
Blocker_Orientations = 0; 
Blocker_Sizes = [0.5, 0.5];

isWithHuman = 0;

path = getPath(mapWidth, Start, Target, Blocker_Positions, Blocker_Orientations, Blocker_Sizes, isWithHuman);

disp("done.")

%%
% ------------------------------
%       两个障碍物，变长
% ------------------------------

clear; clc;

Start = [0, 1];
Target = [6, 3.5];

mapWidth = 7; 

Blocker_Positions = [[1, 1.5]; [4.5, 4.5]]; 
Blocker_Orientations = [0, 0]; 
Blocker_Sizes = [[0.5, 0.5]; [1.0, 1.0]];

isWithHuman = 0;

path = getPath(mapWidth, Start, Target, Blocker_Positions, Blocker_Orientations, Blocker_Sizes, isWithHuman);

disp("done.")

%% 
%------------------------------
%      两个障碍物，变长，一个人
%------------------------------
clear; clc;

Start = [0, 1];
Target = [6, 3.5];

mapWidth = 7; 

Blocker_Positions = [[1, 1.5]; [4.5, 4.5]; [3, 3]]; 
Blocker_Orientations = [0, 0, 45]; 
Blocker_Sizes = [[0.5, 0.5]; [1.0, 1.0]; [0.24, 0.12]];

isWithHuman = 1;

path = getPath(mapWidth, Start, Target, Blocker_Positions, Blocker_Orientations, Blocker_Sizes, isWithHuman);

disp("done.")

%%
%------------------------------
%      两个障碍物，变长，一个人，路径上到下
%------------------------------
clear; clc;

Start = [6, 1];
Target = [0, 3.5];

mapWidth = 7; 

Blocker_Positions = [[1, 1.5]; [4.5, 4.5]; [3, 3]]; 
Blocker_Orientations = [0, 0, 45]; 
Blocker_Sizes = [[0.5, 0.5]; [1.0, 1.0]; [0.24, 0.12]];

isWithHuman = 1;

path = getPath(mapWidth, Start, Target, Blocker_Positions, Blocker_Orientations, Blocker_Sizes, isWithHuman);

disp("done.")

%%
%------------------------------
%      两个障碍物，变长，一个人，路径右上到左下
%------------------------------
clear; clc;

Start = [6, 5.5];
Target = [0, 1];

mapWidth = 7; 

Blocker_Positions = [[1, 1.5]; [4.5, 4.5]; [3, 3]]; 
Blocker_Orientations = [0, 0, 45]; 
Blocker_Sizes = [[0.5, 0.5]; [1.0, 1.0]; [0.24, 0.12]];

isWithHuman = 1;

path = getPath(mapWidth, Start, Target, Blocker_Positions, Blocker_Orientations, Blocker_Sizes, isWithHuman);

disp("done.")