
%% Check_FW_Bindings.m
clear; clc;
modelName = 'Control_Logic';
try
    load_system(modelName);
catch
    addpath(genpath('../..')); % Add parent dirs
    load_system(modelName);
end

fw_blocks = find_system(modelName, 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'FromWorkspace');

fprintf('Found %d From Workspace blocks:\n', length(fw_blocks));
for i = 1:length(fw_blocks)
    blk = fw_blocks{i};
    varName = get_param(blk, 'VariableName');
    fprintf('%d. Block: "%s" -> Var: "%s"\n', i, blk, varName);
end
