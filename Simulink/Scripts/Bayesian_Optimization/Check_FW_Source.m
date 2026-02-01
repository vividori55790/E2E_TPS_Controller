
%% Check_FW_Source.m
% Check the 'Source' parameter of From Workspace blocks
clear; clc;
modelName = 'Control_Logic';
try
    load_system(modelName);
catch
    addpath(genpath('../..')); 
    load_system(modelName);
end

fw_blocks = find_system(modelName, 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'FromWorkspace');

fprintf('Found %d From Workspace blocks:\n', length(fw_blocks));
for i = 1:length(fw_blocks)
    blk = fw_blocks{i};
    source = 'N/A';
    try
        % The parameter name for source usually depends on Simulink version, 
        % often 'DataScope' or similar, but for From Workspace it is usually implied 
        % or there is no specific parameter if it only supports Base/Model via data resolution.
        % Actually, From Workspace blocks don't have a 'Source' parameter in older versions, they just look in the workspace hierarchy.
        % But checking directly can't hurt.
        % Newer versions have 'OBSERVE DATA' or similar? No.
        % Let's check 'VariableName' resolution.
        
        varName = get_param(blk, 'VariableName');
        existsBase = evalin('base', sprintf('exist(''%s'', ''var'')', varName));
        
        % Check Model Workspace
        mdlWks = get_param(modelName, 'ModelWorkspace');
        existsModel = mdlWks.hasVariable(varName);
        
        fprintf('%d. Block: "%s"\n', i, blk);
        fprintf('   VarName: %s\n', varName);
        fprintf('   Exists in Base WS: %d\n', existsBase);
        fprintf('   Exists in Model WS: %d\n', existsModel);
        
        % If exists in Model WS, Model WS takes precedence!
        if existsModel
             val = mdlWks.getVariable(varName);
             fprintf('   [WARNING] Variable shadowed by Model Workspace! Value type: %s\n', class(val));
        end
        
    catch e
        fprintf('   Error checking block: %s\n', e.message);
    end
end
