
%% Inspect_FromWorkspace.m (Fixed Path)
% Inspect 'From Workspace' blocks to see what variable they are reading

clear; clc;

% Add paths
currentPath = fileparts(mfilename('fullpath'));
addpath(currentPath);
addpath(fullfile(currentPath, '..')); 
addpath(fullfile(currentPath, '..', '..')); 
addpath(fullfile(currentPath, '..', '..', 'Modules')); 

modelName = 'Control_Logic';
try
    load_system(modelName);
catch
    warning('Could not load Control_Logic directly. Checking Modules path...');
    addpath(fullfile(currentPath, '..', '..', 'Modules'));
    load_system(modelName);
end

fprintf('=== Inspecting "From Workspace" Blocks in %s ===\n', modelName);

% Find all From Workspace blocks
fw_blocks = find_system(modelName, 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'FromWorkspace');

if isempty(fw_blocks)
    fprintf('No "From Workspace" blocks found.\n');
else
    for i = 1:length(fw_blocks)
        blk = fw_blocks{i};
        varName = get_param(blk, 'VariableName');
        fprintf('Block: %s\n', blk);
        fprintf('    Reading Variable: "%s"\n', varName);
        
        % Check if variable exists in Base Workspace
        % Note: We need to populate Base Workspace first to check properly, 
        % so run Create_Scenario
        evalin('base', 'Create_Scenario');
        
        if evalin('base', sprintf('exist(''%s'', ''var'')', varName))
            fprintf('    Status: Found in Base Workspace.\n');
            val = evalin('base', varName);
            if isa(val, 'timeseries')
                fprintf('    Type: timeseries (Time range: %.4f to %.4f)\n', val.Time(1), val.Time(end));
                fprintf('    Values: Start=%.4f, End=%.4f\n', val.Data(1), val.Data(end));
            else
                 fprintf('    Type: %s\n', class(val));
            end
        else
            fprintf('    Status: NOT FOUND in Base Workspace (Check Model Workspace?)\n');
        end
        fprintf('--------------------------------------------------\n');
    end
end
