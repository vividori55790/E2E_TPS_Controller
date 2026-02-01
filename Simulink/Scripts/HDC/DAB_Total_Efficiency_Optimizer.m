function DAB_Universal_TPS_Solver
    % -------------------------------------------------------------------------
    % DAB Converter Universal TPS Optimization (Per-Unit Basis)
    % -------------------------------------------------------------------------
    % 개요:
    %   특정 하드웨어 값이 아닌, 전압비(k)와 정규화 전력(P_pu)에 따른
    %   범용적인 최적 제어 맵(Map)을 생성합니다.
    %
    % 입력:
    %   k : Voltage Conversion Ratio (V1 / (n*V2))
    %
    % 출력:
    %   Lookup Table 데이터 (Power vs D1, D2, Phi)
    % -------------------------------------------------------------------------

    clc; clear; close all;

    %% 1. 범용 파라미터 설정 (User Input)
    % k = 1 이면 매칭 상태, k != 1 이면 Mismatch 상태
    % 예: 입력 100V, 출력 80V, n=1 이면 k = 1.25
    % 예: 입력 100V, 출력 100V, n=1 이면 k = 1.0
    k_val = 1.25; 
    
    fprintf('--------------------------------------------------\n');
    fprintf(' 범용 TPS 최적화 솔버 (Voltage Ratio k = %.2f)\n', k_val);
    fprintf('--------------------------------------------------\n');

    %% 2. 전력 스윕 (Power Sweep)
    % 정규화된 전력 0.05 ~ 1.0 (최대 전력) 까지 구간별 최적해 탐색
    P_pu_axis = 0.05 : 0.05 : 0.8; 
    
    % 결과 저장용 배열
    opt_D1  = zeros(size(P_pu_axis));
    opt_D2  = zeros(size(P_pu_axis));
    opt_Phi = zeros(size(P_pu_axis));
    opt_Irms= zeros(size(P_pu_axis));
    
    % fmincon 설정
    options = optimoptions('fmincon', 'Display', 'off', ...
        'Algorithm', 'sqp', 'OptimalityTolerance', 1e-6);
    
    % 초기 추정값
    x0 = [0.4, 0.4, 0.1]; 

    %% 3. 최적화 루프 (Generating Control Map)
    fprintf('전력 구간별 최적해 계산 중...\n');
    
    for i = 1:length(P_pu_axis)
        target_P_pu = P_pu_axis(i);
        
        % 목적함수: 정규화된 RMS 전류 최소화
        % 제약조건: 정규화된 전력 == target_P_pu
        
        [x_opt, fval] = fmincon(...
            @(x) cost_Irms_PU(x, k_val), ...
            x0, [], [], [], [], [0, 0, -0.25], [0.5, 0.5, 0.25], ...
            @(x) constraint_Power_PU(x, k_val, target_P_pu), ...
            options);
        
        % 결과 저장
        opt_D1(i)  = x_opt(1) * 2; % Duty (0~1)
        opt_D2(i)  = x_opt(2) * 2;
        opt_Phi(i) = x_opt(3);
        opt_Irms(i)= fval;
        
        % 다음 단계 수렴 속도를 위해 현재 해를 다음 초기값으로 사용
        x0 = x_opt; 
        
        fprintf(' P_pu: %.2f => D1: %.3f, D2: %.3f, Phi: %.3f \n', ...
            target_P_pu, opt_D1(i), opt_D2(i), opt_Phi(i));
    end

    %% 4. 결과 시각화 및 Lookup Table 생성
    figure('Color','w', 'Position', [100, 100, 800, 600]);
    
    subplot(2,2,1);
    plot(P_pu_axis, opt_D1, 'b-o', 'LineWidth', 1.5); grid on;
    title('Optimal Primary Duty (D_1)');
    xlabel('Normalized Power (P_{pu})'); ylabel('Duty Ratio');
    ylim([0 1]);

    subplot(2,2,2);
    plot(P_pu_axis, opt_D2, 'r-s', 'LineWidth', 1.5); grid on;
    title('Optimal Secondary Duty (D_2)');
    xlabel('Normalized Power (P_{pu})'); ylabel('Duty Ratio');
    ylim([0 1]);

    subplot(2,2,3);
    plot(P_pu_axis, opt_Phi, 'k-^', 'LineWidth', 1.5); grid on;
    title('Optimal Phase Shift (\phi)');
    xlabel('Normalized Power (P_{pu})'); ylabel('Phase (Normalized T)');

    subplot(2,2,4);
    plot(P_pu_axis, opt_Irms, 'm-d', 'LineWidth', 1.5); grid on;
    title('Minimum RMS Current (p.u.)');
    xlabel('Normalized Power (P_{pu})'); ylabel('I_{rms, pu}');

    sgtitle(sprintf('Universal TPS Optimization Map (k = %.2f)', k_val));
    
    %% 5. 시뮬링크용 테이블 데이터 출력
    fprintf('\n[Simulink Lookup Table Data]\n');
    fprintf('Breakpoints (Power): %s\n', mat2str(P_pu_axis, 2));
    fprintf('Table Data (D1):     %s\n', mat2str(opt_D1, 3));
    fprintf('Table Data (D2):     %s\n', mat2str(opt_D2, 3));
    fprintf('Table Data (Phi):    %s\n', mat2str(opt_Phi, 3));
end

%% ------------------------------------------------------------------------
%  정규화된 수식 모델 (Normalized Analytical Model)
%  ------------------------------------------------------------------------

function J = cost_Irms_PU(x, k)
    % 정규화된 RMS 전류 계산
    % V1 = 1, V2' = 1/k 로 두고 계산하면 됨.
    % 혹은 V1 = k, V2' = 1 로 둬도 됨. (비율만 맞으면 됨)
    
    % 편의상 V1_base = 1 로 설정
    v1_mag = 1;
    v2_mag = 1/k; % V2' = V1 / k
    
    [Irms_pu, ~] = calc_PU_DAB(x, v1_mag, v2_mag);
    J = Irms_pu;
end

function [c, ceq] = constraint_Power_PU(x, k, P_target_pu)
    % 정규화된 전력 제약 조건
    v1_mag = 1;
    v2_mag = 1/k;
    
    [~, P_calc_raw] = calc_PU_DAB(x, v1_mag, v2_mag);
    
    % 정규화 기준값 (Base Power)
    % SPS (Single Phase Shift)에서의 이론적 최대 전력
    % P_base = (V1 * V2') / (8 * f * L) * (Normalization Factor)
    % 여기선 수식적으로 P = (v1*v2)/(2*pi*f*L) * phi(pi-phi) 등등 복잡함.
    % 우리의 calc_PU_DAB 함수는 L=1, f=1 일 때의 값을 뱉으므로,
    % SPS 최대 전력(Phi=0.25, D1=1, D2=1 일 때)을 기준으로 정규화해야 함.
    
    % 최대 전력(Reference) 계산 (SPS mode: D=0.5(Full), Phi=0.25)
    [~, P_max_ref] = calc_PU_DAB([0.5, 0.5, 0.25], v1_mag, v2_mag);
    
    P_calc_pu = P_calc_raw / P_max_ref;
    
    c = [];
    ceq = P_calc_pu - P_target_pu;
end

function [Irms, P_avg] = calc_PU_DAB(x, V1, V2)
    % L=1, T=1 (f=1) 로 가정한 정규화 계산
    D1 = x(1)*2; D2 = x(2)*2; Phi = x(3);
    
    N = 200;
    t = linspace(0, 1, N);
    dt = 1/N;
    
    val1 = get_pwm(t, D1, 0);
    val2 = get_pwm(t, D2, Phi);
    
    vL = V1*val1 - V2*val2;
    
    % iL = integral(vL/L)dt, L=1
    iL_temp = cumsum(vL) * dt;
    iL = iL_temp - mean(iL_temp);
    
    Irms = sqrt(mean(iL.^2));
    P_avg = mean((V1*val1) .* iL);
end

function val = get_pwm(t, D, shift)
    t = mod(t - shift, 1);
    val = zeros(size(t));
    val(t >= (0.25 - D/4) & t < (0.25 + D/4)) = 1;
    val(t >= (0.75 - D/4) & t < (0.75 + D/4)) = -1;
end