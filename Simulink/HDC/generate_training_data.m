function data_set = generate_training_data()
    % 1. 설정 로드
    params = config_dab();
    N_samples = 50000; % 학습 데이터 개수
    
    % Vout 범위: Vout_min ~ Vout_max
    % Sobol point 3 (if we had it, but stick to rand for robust simplicity on Vout or re-init sobolset(3))
    % For simplicity in this robust generator, let's use another dimension for Vout?
    % Let's expand Sobol to 3 dimensions for better coverage: Vin, Vout, Pref
    if exist('sobolset', 'file')
        p = sobolset(3, 'Skip', 1000); 
        sobol_points = net(p, N_samples);
    else
        sobol_points = rand(N_samples, 3);
    end
    
    Vin_min = 200; Vin_max = 450;
    Vout_min = params.Vout_min; Vout_max = params.Vout_max;
    P_rated = params.P_rated;
    
    V_vec = Vin_min + (Vin_max - Vin_min) * sobol_points(:,1);
    Vout_vec = Vout_min + (Vout_max - Vout_min) * sobol_points(:,2);
    P_raw = (sobol_points(:,3) - 0.5) * 2; % -1 ~ 1
    P_vec = P_raw * P_rated;
    
    % 3. 병렬 연산 (Parallel Computing)
    % 결과 저장용 변수 선언
    results = zeros(N_samples, 7); % [Vin, Vout, Pref, D1, D2, Phi, Efficiency]
    
    fprintf('Starting Robust Optimization for %d points...\n', N_samples);
    
    % Check for Parallel Computing Toolbox
    if isempty(gcp('nocreate'))
        try
            parpool;
        catch
            fprintf('Parallel pool could not be started. Using serial execution.\n');
        end
    end
    
    parfor i = 1:N_samples
        Vin = V_vec(i);
        Vout = Vout_vec(i);
        Pref = P_vec(i);
        
        % Robust Solver 호출
        try
            % Optimization
            [opt_x, min_loss] = dab_optimizer(Vin, Vout, Pref, params);
            
            % Efficiency Calculation (Real Physics)
            [~, ~, ~, ~, P_in, P_loss_real] = physics_engine(opt_x, Vin, Vout, params.L_nom, params);
            
            % Efficiency Calculation (Bidirectional)
            P_abs = abs(P_in);
            
            if P_abs > P_loss_real && P_abs > 1e-6
                Efficiency = (P_abs - P_loss_real) / P_abs;
            else
                Efficiency = 0;
            end
            
            % Convergence Check: 명령한 전력과 실제 전력의 오차가 크면 실패로 간주
            % (Solver가 P_ref를 못 맞춘 경우)
            P_error_ratio = abs(P_in - Pref) / (abs(Pref) + 1e-3); % 1e-3 prevents div by zero
            
            if P_error_ratio > 0.10 % 10% 이상 차이나면 불량 데이터
                results(i, :) = [Vin, Vout, Pref, NaN, NaN, NaN, NaN];
            else
                % 결과 저장
                results(i, :) = [Vin, Vout, Pref, opt_x(1), opt_x(2), opt_x(3), Efficiency];
            end
            
        catch
            % 수렴 실패 시 NaN 처리
            results(i, :) = [Vin, Vout, Pref, NaN, NaN, NaN, NaN];
        end
    end
    
    % 4. 데이터 정제 및 저장
    valid_idx = ~isnan(results(:,7)); % Eff가 NaN이 아닌 것만 추출
    data_set = results(valid_idx, :);
    
    save('DAB_Robust_Dataset.mat', 'data_set');
    writematrix(data_set, 'DAB_Robust_Dataset.csv');
    fprintf('Data generation complete. %d valid samples saved to .mat and .csv\n', sum(valid_idx));
    
    % Visual check if running interactively
    if usejava('desktop')
        figure;
        scatter3(data_set(:,1), data_set(:,2), data_set(:,6)*100, 10, data_set(:,6), 'filled');
        xlabel('Vin [V]'); ylabel('Power [W]'); zlabel('Efficiency [%]');
        title('Robust Optimized TPS Maps');
        colorbar;
        view(45, 30);
    end
end
