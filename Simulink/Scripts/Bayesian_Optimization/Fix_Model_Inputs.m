
%% Fix_Model_Inputs.m (v2 - Trace From/Goto)
% Inspect and Fix DAB_Controller_System inputs (Hardcoded checks)

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
    fprintf('Loaded model: %s\n', modelName);
catch ME
    fprintf('Error loading model: %s\n', ME.message);
    return;
end

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
    return;
end

fprintf('Found Controller Block: %s\n', controllerBlock);
ports = get_param(controllerBlock, 'PortConnectivity');
inputsToCheck = [7, 8];
inputParams = {'Kp_ext', 'Ki_ext'};
modified = false;

for i = 1:length(inputsToCheck)
    portIdx = inputsToCheck(i);
    paramName = inputParams{i};
    srcBlock = ports(portIdx).SrcBlock;
    
    if srcBlock == -1
        fprintf('  [WARN] Input %d (%s) is Unconnected!\n', portIdx, paramName);
        continue;
    end
    
    srcBlockPath = getfullname(srcBlock);
    blockType = get_param(srcBlockPath, 'BlockType');
    fprintf('  Input %d (%s) connected to %s Block: %s\n', portIdx, paramName, blockType, srcBlockPath);
    
    targetConstBlock = '';
    
    if strcmp(blockType, 'Constant')
        targetConstBlock = srcBlockPath;
    elseif strcmp(blockType, 'From')
        % Helper to find Goto
        tag = get_param(srcBlockPath, 'GotoTag');
        fprintf('    From Tag: %s. Searching for Goto block...\n', tag);
        
        gotoBlocks = find_system(modelName, 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'Goto', 'GotoTag', tag);
        
        if isempty(gotoBlocks)
            fprintf('    [ERROR] No Goto block found for tag %s\n', tag);
            continue;
        elseif length(gotoBlocks) > 1
            fprintf('    [WARN] Multiple Goto blocks found for tag %s. Using first one: %s\n', tag, gotoBlocks{1});
            targetGoto = gotoBlocks{1};
        else
            targetGoto = gotoBlocks{1};
            fprintf('    Found Goto block: %s\n', targetGoto);
        end
        
        % Trace input of Goto block
        gotoPorts = get_param(targetGoto, 'PortConnectivity');
        % Input port is usually index 1
        gotoSrc = gotoPorts(1).SrcBlock;
        if gotoSrc ~= -1
            gotoSrcPath = getfullname(gotoSrc);
            gotoSrcType = get_param(gotoSrcPath, 'BlockType');
            fprintf('    Goto block is fed by %s Block: %s\n', gotoSrcType, gotoSrcPath);
            
            if strcmp(gotoSrcType, 'Constant')
                targetConstBlock = gotoSrcPath;
            end
        else
            fprintf('    [ERROR] Goto block input is unconnected.\n');
        end
    end
    
    % If we found a Constant block feeding the signal
    if ~isempty(targetConstBlock)
        currVal = get_param(targetConstBlock, 'Value');
        fprintf('    -> Check Constant Value: "%s"\n', currVal);
        
        if ~strcmp(currVal, paramName)
            fprintf('       [FIX] Updating Value to "%s"\n', paramName);
            set_param(targetConstBlock, 'Value', paramName);
            modified = true;
        else
            fprintf('       [OK] Value is already "%s"\n', paramName);
        end
    else
        fprintf('    [WARN] Could not trace to a Constant block. Signal might be dynamic or complex.\n');
    end
end

if modified
    fprintf('Saving model changes...\n');
    save_system(modelName);
    fprintf('Model saved.\n');
else
    fprintf('No changes required.\n');
end
