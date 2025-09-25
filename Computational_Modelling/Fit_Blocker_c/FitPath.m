clc;clear;
results = [];

for Blocker_c = 0:0.01:10
    e = FindPath(Blocker_c);
    results = [results,[Blocker_c;e]];
    disp(['Loop ' num2str(size(results,2)) '; Blocker_c = ' num2str(Blocker_c) '; RMSE = ' num2str(e) '...']);
end
ind = find(results(size(results,1),:) == min(results(size(results,1),:)));
disp(['Blocker_c = ' num2str(results(1,ind)) '; RMSE=' num2str(results(2,ind))])

save('Blocker_c.mat',"results");

% output the path of model fitting results;
ModelPath_all(:,2) = ModelPath_all(:,2) - 350;
ModelPath_revised = ModelPath_all(:,[1 3 2]);
dlmwrite("Exp-2_Path_Fit.csv",ModelPath_revised);