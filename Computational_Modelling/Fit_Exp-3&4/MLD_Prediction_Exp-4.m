clc;clear;
addpath('./functions');
load('./fit_parameters.mat');
m1 = x(1);
n1 = x(2);
m2 = x(3);
n2 = x(4);
a = 0.24;
b = 0.12;
c = x(5);
th = x(6);
ncf = x(7);
r = 5;
a_R = 20; % the width of the barrier from top view;
b_R = 200; % the length of the barrier from top view;
width = 7; % How wide is the map
EndPoint = [699;350];
StartPoint = [1;350];
EndPoint_Avatar = [699;300];
StartPoint_Avatar = [2;300];
Target = [3.5,7]; % The position of the target pole
precision = 0.01; % How precise is the map
baseSpeed = 0.87; % m/s
fps = 72; timestep = 1/fps;
minStep = baseSpeed/precision * timestep; % cm/frame
RTsettings = struct('useFD',1,'stepSize',1);
Blocker_c = 6.76;
MLD = [];

%% get the results of choosing different sides;
for i = 1:4 % the position of the barrier;
    Blocker_Pos = -200 + (i-1)*50;
    Blocker = [Blocker_Pos+350,350];
    Blocker_width = 200;

    % Calculate the EnergyMap of Person
    %% first get the avatar's path if there is no avatar;
    EnergyMap0 = GetMap_Object_new(width/precision+1,Target(2)/precision+1,Blocker,Target./precision,a_R,b_R,Blocker_c,m2,n2,a,b,c,precision,ncf);
    EnergyMap0 = rescale(EnergyMap0,0,1,'InputMin',0,'InputMax',th);
    EnergyMap0(1:(Blocker(1)+0.5*b_R),Blocker(2)) = 1;

    SpeedMap0 = 1 - EnergyMap0;
    T0 = fm(SpeedMap0,StartPoint_Avatar,[1 1],struct('implementation','C++','order',2));
    path0 = rayTrace(T0,EndPoint_Avatar,StartPoint_Avatar,RTsettings);
    path0 = sortrows(path0',2);
    path0 = [300,0;path0];

    % delete the repeated line;
    [C,ia] = unique(path0(:,2));
    path00 = path0(ia,:);
    path0_revised_x = interp1(path00(:,2),path00(:,1),1:670);
    path0_revised = [path0_revised_x',(1:670)'];
    path_avatar = [path0_revised(:,1),(700 - path0_revised(:,2))];

    %% then get the participant's path if there is no avatar;
    EnergyMap0 = GetMap_Object_new(width/precision+1,Target(2)/precision+1,Blocker,Target./precision,a_R,b_R,Blocker_c,m2,n2,a,b,c,precision,ncf);
    EnergyMap0 = rescale(EnergyMap0,0,1,'InputMin',0,'InputMax',th);
    EnergyMap0(1:(Blocker(1)+0.5*b_R),Blocker(2)) = 1;

    SpeedMap0 = 1 - EnergyMap0;
    T0 = fm(SpeedMap0,StartPoint,[1 1],struct('implementation','C++','order',2));
    path0 = rayTrace(T0,EndPoint,StartPoint,RTsettings);
    path0 = sortrows(path0',2);
    path0 = [350,0;path0];

    % delete the repeated line;
    [C,ia] = unique(path0(:,2));
    path00 = path0(ia,:);
    path0_revised_x = interp1(path00(:,2),path00(:,1),1:670);
    path0_revised = [path0_revised_x',(1:670)'];
    path_participant = path0_revised;

    %% then calculate the nearest collision point;
    d = path_participant - path_avatar;
    d1 = d(:,1).^2 + d(:,2).^2;
    Block_Person = path_avatar(find(d1==min(d1)),:);
    Block_Ori = 0; % 0 means the avatar is facing the participant, and 90 means it is facing the right side;

    %% Calculate the EnergyMap_Person of participant, considering the avatar;
    EnergyMap_Person = GetMap(width/precision+1,Target(2)/precision+1,Block_Person,Block_Ori,Target/precision,m1,n1,m2,n2,a,b,c,precision,ncf);

    %% Re-calculate the EnergyMap of participant, considering the avatar;
    EnergyMap_Object = GetMap_Object_new(width/precision+1,Target(2)/precision+1,Blocker,Target./precision,a_R,b_R,Blocker_c,m2,n2,a,b,c,precision,ncf);
    EnergyMap_All = EnergyMap_Object + EnergyMap_Person;
    EnergyMap_All = rescale(EnergyMap_All,0,1,'InputMin',0,'InputMax',th);
    EnergyMap_All(1:(Blocker(1)+0.5*b_R),Blocker(2)) = 1;

    SpeedMap = 1 - EnergyMap_All;
    T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
    path = rayTrace(T,EndPoint,StartPoint,RTsettings);
    path2 = sortrows(path',2);
    path2 = [350,0;path2];

    % delete the repeated line;
    [C,ia] = unique(path2(:,2));
    path3 = path2(ia,:);
    path_revised_x = interp1(path3(:,2),path3(:,1),1:670);
    path_revised = [path_revised_x',(1:670)'];
    MLD = [MLD;Blocker_Pos max(path_revised(:,1))-350];

    disp(['Position = ' num2str(Blocker_Pos) '; MLD = ' num2str(max(path_revised(:,1))-350) '......']);
end

dlmwrite('Exp-4_MLD_Fit.csv',MLD);
