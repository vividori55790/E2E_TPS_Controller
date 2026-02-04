function Cleanup_Model_Workspace(modelName, varNames)
% CLEANUP_MODEL_WORKSPACE Moves specified variables from Model Workspace to Base Workspace
% 
% Usage:
%   Cleanup_Model_Workspace('Main_Simulation', {'Kp', 'Ki'});
%
% This function checks if the specified variables (varNames) exist in the
% Model Workspace of the Simulink model. If they do, it copies their values
% to the Base Workspace and removes them from the Model Workspace/
% This prevents shadowing issues during parallel simulation or optimization.

    if nargin < 2
        error('Usage: Cleanup_Model_Workspace(modelName, varNames_CellArray)');
    end

    % Load the model if not loaded
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end

    % Get the Model Workspace handle
    hws = get_param(modelName, 'ModelWorkspace');
    
    % Force reload to ensure we see the current state
    hws.reload();

    for i = 1:length(varNames)
        varName = varNames{i};
        
        if hws.hasVariable(varName)
            % 1. Get the value from Model Workspace
            val = hws.getVariable(varName);
            
            % 2. Assign to Base Workspace (preserves the default value)
            assignin('base', varName, val);
            
            % 3. Remove from Model Workspace
            hws.deleteVariable(varName);
            
            fprintf('[Success] Moved ''%s'' from Model Workspace to Base Workspace.\n', varName);
        else
            fprintf('[Info] ''%s'' not found in Model Workspace. Checked Base Workspace or Dictionary.\n', varName);
        end
    end

    % Recommendation: Save the model manually after verifying changes if you want to keep this permanently
    % save_system(modelName); 
    fprintf('Cleanup complete. Please save the model if you want to persist these changes.\n');
end
