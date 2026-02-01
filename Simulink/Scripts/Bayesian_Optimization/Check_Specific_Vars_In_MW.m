
%% Check_Specific_Vars_In_MW.m
clear; clc;
modelName = 'Control_Logic';
try
    load_system(modelName);
catch
    addpath(genpath('../..')); 
    load_system(modelName);
end

varsToCheck = {'Ref_Kp_ext', 'Ref_Ki_ext'};
mdlWks = get_param(modelName, 'ModelWorkspace');

fprintf('=== Checking Model Workspace Shadows ===\n');
for i = 1:length(varsToCheck)
    v = varsToCheck{i};
    if mdlWks.hasVariable(v)
        fprintf('[FOUND] %s exists in Model Workspace (Shadowing Base WS).\n', v);
    else
        fprintf('[OK] %s NOT found in Model Workspace.\n', v);
    end
end
