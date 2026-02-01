
%% Inspect_Controller_Inputs.m
% Check what is feeding the DAB_Controller_System block

clear; clc;

modelName = 'Control_Logic';
load_system(modelName);

% Find the MATLAB System block
blocks = find_system(modelName, 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'MATLABSystem');
controllerBlock = '';
for i = 1:length(blocks)
    if contains(get_param(blocks{i}, 'System'), 'DAB_Controller_System')
        controllerBlock = blocks{i};
        break;
    end
end

if isempty(controllerBlock)
    fprintf('Error: DAB_Controller_System block not found.\n');
    % Try searching by name? 
    % The user file says "classdef DAB_Controller_System", so the block likely references this system object.
    return;
end

fprintf('Found Controller Block: %s\n', controllerBlock);

% Get Port Connectivity
ports = get_param(controllerBlock, 'PortConnectivity');

% Input 7 (Kp) and 8 (Ki) - 1-based index
inputsToCheck = [7, 8];
inputNames = {'Kp_ext', 'Ki_ext'};

for i = 1:length(inputsToCheck)
    portIdx = inputsToCheck(i);
    srcBlock = ports(portIdx).SrcBlock;
    
    if srcBlock == -1
        fprintf('Input %d (%s) is Unconnected!\n', portIdx, inputNames{i});
    else
        srcBlockPath = getfullname(srcBlock);
        fprintf('Input %d (%s) is connected to: %s\n', portIdx, inputNames{i}, srcBlockPath);
        
        % Check if it is a Constant block
        blockType = get_param(srcBlockPath, 'BlockType');
        fprintf('    Block Type: %s\n', blockType);
        
        if strcmp(blockType, 'Constant')
            val = get_param(srcBlockPath, 'Value');
            fprintf('    Current Value: %s\n', val);
        end
    end
end
