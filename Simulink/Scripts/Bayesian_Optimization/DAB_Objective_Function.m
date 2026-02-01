function cost = DAB_Objective_Function(params, q)
    % DAB_Objective_Function (Improved with DataQueue)
    % params: Optimization parameters
    % q: (Optional) Parallel.DataQueue for thread-safe logging
    
    try
        %% 1. 기본 설정
        currentFile = mfilename('fullpath');
        [currentDir, ~, ~] = fileparts(currentFile);
        addpath(currentDir);
        addpath(fullfile(currentDir, '..')); 
        addpath(fullfile(currentDir, '..', '..')); 
        addpath(fullfile(currentDir, '..', '..', 'Modules')); 
        
        % [FIX] 모델 로드 명시
        modelName = 'Control_Logic';
        if ~bdIsLoaded(modelName)
            load_system(modelName);
        end

        % [FIX] Model Workspace에 직접 변수 할당 (Shadowing 문제 해결)
        % From Workspace 블록이 Model Workspace를 참조하고 있다면, 여기에 값을 넣어야 시뮬레이션에 반영됨
        mdlWks = get_param(modelName, 'ModelWorkspace');
        
        %% 2. 시뮬레이션 준비
        in = Simulink.SimulationInput(modelName);
        
        % 1. 시나리오 스크립트 실행
        evalin('base', 'Create_Scenario'); 
        
        % 2. Base Workspace 변수 가져오기
        SIM_TIME = evalin('base', 'SIM_TIME');
        Ref_I_ess_cmd = evalin('base', 'Ref_I_ess_cmd');
        
        % 3. 최적화 파라미터를 TimeSeries로 변환
        Kp_val = params.Kp;
        Ki_val = params.Ki;
        
        time_vec = [0, SIM_TIME];
        Kp_vec = [Kp_val, Kp_val];
        Ki_vec = [Ki_val, Ki_val];
        
        Ref_Kp_ext = timeseries(Kp_vec, time_vec);
        Ref_Ki_ext = timeseries(Ki_vec, time_vec);
        
        % [FIX] Model Workspace에 데이터 직접 주입
        mdlWks.assignin('Ref_I_ess_cmd', Ref_I_ess_cmd);
        mdlWks.assignin('Ref_Kp_ext', Ref_Kp_ext);
        mdlWks.assignin('Ref_Ki_ext', Ref_Ki_ext);
        mdlWks.assignin('Kp_ext', Kp_val);
        mdlWks.assignin('Ki_ext', Ki_val);
        
        in = in.setModelParameter('SimulationMode', 'normal'); % 안정성을 위해 Normal 모드 사용
        in = in.setModelParameter('StopTime', num2str(SIM_TIME)); % [FIX] 시뮬레이션 절대 시간 강제 설정
        
        % [DEBUG] 입력 데이터 로깅 (DataQueue 전송)
        if nargin > 1 && ~isempty(q)
            send(q, struct('Type', 'DEBUG', 'Msg', sprintf('Sim Input: Kp=%.4f, Ki=%.4f', Kp_val, Ki_val)));
        end
        
        %% 3. 시뮬레이션 실행 및 평가
        simOut = sim(in);
        [cost, details] = Calculate_DAB_Cost(simOut);
        
        %% 4. 결과 로깅 (DataQueue 전송)
        if nargin > 1 && ~isempty(q)
            % 결과 구조체 생성
            logResult = struct(...
                'Type', 'RESULT', ...
                'Kp', Kp_val, ...
                'Ki', Ki_val, ...
                'Cost', cost, ...
                'Details', details, ...
                'Timestamp', datestr(now, 'HH:MM:SS'));
            
            send(q, logResult);
        end
        
    catch e
        errMsg = sprintf('Error in Objective: %s', e.message);
        fprintf('%s\n', errMsg);
        cost = 1e6;
        
        % 에러 로깅 (DataQueue 전송)
        if nargin > 1 && ~isempty(q)
            send(q, struct('Type', 'ERROR', 'Msg', errMsg));
        end
    end
end
