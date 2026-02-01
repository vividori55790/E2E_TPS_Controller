
%% Debug_Sim.m
% Isolate parameter passing issue by running serial simulations with distinct parameters

clear; clc;

% Add paths
currentPath = fileparts(mfilename('fullpath'));
addpath(currentPath);
addpath(fullfile(currentPath, '..')); 
addpath(fullfile(currentPath, '..', '..')); 
addpath(fullfile(currentPath, '..', '..', 'Modules')); 

% Parameters to test
Kp_set = [0.01, 20.0];
Ki_set = [0.1, 500.0];

modelName = 'Control_Logic';
load_system(modelName);

for i = 1:2
    Kp_val = Kp_set(i);
    Ki_val = Ki_set(i);
    
    fprintf('>>> Test Run %d: Kp=%.4f, Ki=%.4f\n', i, Kp_val, Ki_val);
    
    % Prep Input
    in = Simulink.SimulationInput(modelName);
    evalin('base', 'Create_Scenario');
    
    % Timeseries prep
    SIM_TIME = evalin('base', 'SIM_TIME');
    time_vec = [0, SIM_TIME];
    Ref_Kp_ext = timeseries([Kp_val, Kp_val], time_vec);
    Ref_Ki_ext = timeseries([Ki_val, Ki_val], time_vec);
    Ref_I_ess_cmd = evalin('base', 'Ref_I_ess_cmd');
    
    % [FIX] SimulationInput.setVariable 대신 Base Workspace 직접 제어
    % Timeseries 객체의 경우 setVariable이 캐싱 문제를 일으킬 수 있음
    % in = in.setVariable('Ref_Kp_ext', Ref_Kp_ext);
    % in = in.setVariable('Ref_Ki_ext', Ref_Ki_ext);
    
    in = in.setVariable('Kp_ext', Kp_val);
    in = in.setVariable('Ki_ext', Ki_val);
    
    % [FIX] From Workspace 블록이 Base Workspace를 참조할 경우를 대비해 직접 할당
    assignin('base', 'Ref_Kp_ext', Ref_Kp_ext);
    assignin('base', 'Ref_Ki_ext', Ref_Ki_ext);
    assignin('base', 'Kp_ext', Kp_val);
    assignin('base', 'Ki_ext', Ki_val);
    
    in = in.setModelParameter('SimulationMode', 'normal');
    in = in.setModelParameter('StopTime', num2str(SIM_TIME));
    
    % Run
    simOut = sim(in);
    
    % Check Results
    if isprop(simOut, 'I_Fil')
        I_data = simOut.I_Fil.Data;
        I_mean = mean(I_data);
        I_max = max(I_data);
        fprintf('    [Result] Mean Current: %.4f, Max Current: %.4f\n', I_mean, I_max);
    else
        fprintf('    [Result] I_Fil NOT FOUND\n');
    end
    
    % Check Phi
    if isprop(simOut, 'Phi_Raw')
        Phi_data = simOut.Phi_Raw.Data;
        Phi_mean = mean(Phi_data);
        fprintf('    [Result] Mean Phase: %.4f\n', Phi_mean);
    else
         fprintf('    [Result] Phi_Raw NOT FOUND\n');
    end
    
    % Calculate Cost
    [cost, details] = Calculate_DAB_Cost(simOut);
    fprintf('    [Cost] %.4f (Track=%.4f, SS=%.4f)\n', cost, details.Track, details.SS);
    fprintf('\n');
end
