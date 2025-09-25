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
Target = [3.5,7]; % The position of the target pole
precision = 0.01; % How precise is the map
baseSpeed = 0.87; % m/s
fps = 72; timestep = 1/fps;
minStep = baseSpeed/precision * timestep; % cm/frame
RTsettings = struct('useFD',1,'stepSize',1);
pathlength = [];
socialEnergy = [];
Time = [];
Blocker_c = 6.76;

%% get the results of choosing different sides;
for i = 1:5 % the position of the barrier;
    Blocker_Pos = 20 + (i-1)*20;
    Blocker = [Blocker_Pos+350,350];
    Blocker_width = 200;

    for side = 1:2 % 1 means go left side, 2 means go right side;

        if side == 1
            EnergyMap = GetMap_Object_new(width/precision+1,Target(2)/precision+1,Blocker,Target./precision,a_R,b_R,Blocker_c,m2,n2,a,b,c,precision,ncf);
            EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
            EnergyMap((Blocker(1)+0.5*b_R):(width/precision+1),Blocker(2)) = 1;
        else
            EnergyMap = GetMap_Object_new(width/precision+1,Target(2)/precision+1,Blocker,Target./precision,a_R,b_R,Blocker_c,m2,n2,a,b,c,precision,ncf);
            EnergyMap = rescale(EnergyMap,0,1,'InputMin',0,'InputMax',th);
            EnergyMap(1:(Blocker(1)-0.5*b_R),Blocker(2)) = 1;
        end

        SpeedMap = 1 - EnergyMap;
        T = fm(SpeedMap,StartPoint,[1 1],struct('implementation','C++','order',2));
        path = rayTrace(T,EndPoint,StartPoint,RTsettings);
        path2 = sortrows(path',2);
        path2 = [350,0;path2];

        % delete the repeated line;
        [C,ia] = unique(path2(:,2));
        path3 = path2(ia,:);
        path_revised_x = interp1(path3(:,2),path3(:,1),1:670);
        path_revised = [path_revised_x',(1:670)'];

        % calculate the path length;
        h = diff(path_revised);
        path_length = 0;
        for j = 1:size(h,1)
            delta_l = sqrt(h(j,1)^2 + h(j,2)^2);
            path_length = path_length+delta_l;
        end
        pathlength = [pathlength;Blocker_Pos,side,path_length];

        % calculate the impact of SIFM;
        socialImpact = 0;
        for k = 1:size(path_revised,1);
            socialImpact = socialImpact + EnergyMap(round(path_revised(k,1)),path_revised(k,2));
        end
        socialEnergy = [socialEnergy;Blocker_Pos,side,socialImpact];

        % calculate the time;
        Time = [Time;Blocker_Pos,side,T(Target(1)/precision,Target(2)/precision)];

        disp(['Position = ' num2str(Blocker_Pos) '; Side = ' num2str(side) '......']);

    end
end

%% calculate the ratio of different choice;
Time_Ratio = [];
socialEnergy_Ratio = [];
pathlength_Ratio = [];
for i = 1:5

    % get the Time_Ratio component;
    subdat_time = Time((i-1)*2+1:i*2,:);
    time_ratio = subdat_time(2,3)/subdat_time(1,3);
    Time_Ratio = [Time_Ratio;subdat_time(1,1), time_ratio];

    % get the socialEnergy_Ratio component;
    subdat_socialEnergy = socialEnergy((i-1)*2+1:i*2,:);
    socialEnergy_ratio = subdat_socialEnergy(2,3)/subdat_socialEnergy(1,3);
    socialEnergy_Ratio = [socialEnergy_Ratio;subdat_socialEnergy(1,1), socialEnergy_ratio];

    % get the pathlength_Ratio component;
    subdat_pathlength = pathlength((i-1)*2+1:i*2,:);
    pathlength_ratio = subdat_pathlength(2,3)/subdat_pathlength(1,3);
    pathlength_Ratio = [pathlength_Ratio;subdat_pathlength(1,1), pathlength_ratio];

end


%% use Time_Ratio to fit;
% [0.992 0.996 0.996 0.964 0.912 0.472 0.088 0.036 0.01 0.01 0.008]
RealP = [0.088 0.036 0.01 0.01 0.008];
RealP2 = 1 - RealP;
Predict_x = Time_Ratio(:,2);
cftool; % use cftool to fit the data;

% Results of the fitting: R square = 0.979; RMSE = 0.005; a = 20.936; 
%                         b = -22.164;

par_a = 20.936;
par_b = -22.164;

modelP = 1./(1+exp(par_a+par_b*Predict_x));
modelPrediction = [Time_Ratio modelP];
dlmwrite('Exp-1_Choice_Fit.csv',modelPrediction);
