%% DAB (Dual Active Bridge) Converter Design & Simulation Control
% 이 스크립트는 DAB 컨버터의 시스템 파라미터를 정의하고, 시뮬레이션을 실행하며,
% 결과를 시각화하는 통합 제어 파일입니다.
% PSIM 원본 파일(DAB.py)의 사양을 기반으로 작성되었습니다.

clear; clc; close all;

%% 1. 시스템 파라미터 정의 (System Parameters)
% PSIM 설계 사양을 바탕으로 변수를 선언합니다.
% Simulink 모델(.slx)의 블록 파라미터에 이 변수명들을 입력하세요.

% --- 전력 사양 (Power Specs) ---
V_pri_DC = 400;         % [V] 1차측 입력 전압 (PSIM: VDC_1)
V_batt_nom = 100.8;     % [V] 배터리 공칭 전압 (PSIM: E_nom0)
P_rated = 5000;         % [W] 정격 용량 (가정)

% --- 주요 부품 파라미터 (Components) ---
f_sw = 80e3;            % [Hz] 스위칭 주파수 (PSIM: 80k)
T_sw = 1/f_sw;          % [s] 스위칭 주기
Ts = 10e-9;             % [s] 시뮬레이션 타임스텝 (PSIM: 1E-08)

% 변압기 (Transformer)
N_pri = 4;              % 1차측 권선비 (PSIM: Np)
N_sec = 1;              % 2차측 권선비 (PSIM: Ns)
n_ratio = N_pri/N_sec;  % 권선비 (4:1)

% 인덕터 및 커패시터 (L, C)
L_leakage = 25e-6;      % [H] 누설 인덕턴스 (Modified: 150uH -> 25uH for 5kW Power Output)
                            % 이론적 최대 전력 Pmax = (400*400)/(8*80k*L). 
                            % 150uH일 때 Pmax=1.6kW로 5kW 불가. 25uH면 Pmax=10kW로 충분.
C_dc_link = 500e-6;     % [F] DC 링크 커패시터 (PSIM: C1, C2)
R_wire = 1e-3;          % [Ohm] 선로 저항 (PSIM: Rp, Rs)

% MOSFET (Switching Devices)
Ron_mosfet = 4.8e-3;    % [Ohm] 온 저항 (PSIM: On_Resistance)
R_snubber = 1e5;        % [Ohm] 스너버 저항 (Simulink용)
C_snubber = inf;        % [F] 스너버 커패시터 (무한대=Open)


%% 2. 제어기 게인 설정 (Controller Gains)
% PSIM의 PI 제어기 설정을 가져옵니다.
% 시뮬링크 PI Controller 블록에 사용됩니다.

kp_voltage = 0.1;       % 전압 제어 비례 이득 (PSIM: Gain)
ki_voltage = 100;       % 전압 제어 적분 이득 (Time Constant 변환 필요)
% PSIM Time Constant = 0.001 -> Ki = Kp / Ti = 0.1 / 0.001 = 100

limit_output = 0.5;     % 위상 천이(Alpha) 제한 (Max +/- 0.5)


%% 3. 이론적 검증 (Theoretical Calculation)
% 시뮬레이션 전, 이론적인 최대 전력 전송량을 계산하여 설계를 검증합니다.

V_sec_ref = V_pri_DC / n_ratio; % 1차측 전압을 2차측으로 환산
P_max_theoretical = (V_pri_DC * V_sec_ref) / (8 * f_sw * L_leakage);

fprintf("------------------------------------------------\n");
fprintf(" [Design Verification] \n");
fprintf(" - Switching Freq : %.1f kHz \n", f_sw/1000);
fprintf(" - Transformer Ratio: %d : %d \n", N_pri, N_sec);
fprintf(" - Max Power Transfer: %.2f kW (Theoretical)\n", P_max_theoretical/1000);
fprintf("------------------------------------------------\n");


%% 4. 시뮬레이션 실행 (Run Simulation)
% 'Circuit_Model'이라는 이름의 Simulink 파일이 있다고 가정하고 실행합니다.
% 파일이 없으면 에러가 나므로, try-catch 구문으로 처리합니다.

sim_file_name = 'Circuit_Model'; % .slx 파일 이름

try
    fprintf("시뮬레이션을 시작합니다: %s.slx ...\n", sim_file_name);
    
    % sim() 명령어로 시뮬레이션 실행 및 결과(out) 저장
    % 모델이 없으면 이 부분은 건너뛰고 아래의 'Dummy Data'를 생성합니다.
    simOut = sim(sim_file_name); 
    
    % 결과 데이터 추출 (Simulink Logging 설정 필요)
    time = simOut.tout;
    v_out_data = simOut.V_out.data;
    i_ind_data = simOut.I_L.data;
    
    fprintf("시뮬레이션 완료.\n");
    
catch
    warning("Simulink 모델(%s.slx)을 찾을 수 없습니다.", sim_file_name);
    fprintf("대신, 확인을 위해 *가상의 시뮬레이션 데이터*를 생성하여 그래프를 그립니다.\n");
    
    % Dummy Data Generation (For Preview)
    time = linspace(0, 0.002, 10000); % 2ms
    % 출력 전압: 100V 목표로 서서히 상승하는 형태
    v_out_data = 100.8 * (1 - exp(-time/0.0005)) + 0.5*sin(2*pi*f_sw*time); 
    % 인덕터 전류: 80kHz 고주파 성분
    i_ind_data = 20 * sin(2*pi*f_sw*time) .* (1 - exp(-time/0.0002));
end


%% 5. 결과 시각화 (Visualization)
% Scope를 캡처할 필요 없이, 코드로 깔끔한 보고서용 그래프를 그립니다.

figure('Name', 'DAB Simulation Results', 'Color', 'w', 'Position', [100 100 800 600]);

% (1) 출력 전압 그래프
subplot(2,1,1);
plot(time*1000, v_out_data, 'LineWidth', 2, 'Color', '#0072BD');
grid on;
title('Output Voltage (Battery Voltage)', 'FontSize', 12);
xlabel('Time [ms]'); ylabel('Voltage [V]');
yline(V_batt_nom, '--r', 'Rated Voltage', 'LineWidth', 1.5); % 기준선
xlim([0, time(end)*1000]);

% (2) 인덕터 전류 그래프
subplot(2,1,2);
% 확대된 파형을 보기 위해 마지막 0.1ms만 플롯 (데이터가 충분할 경우)
if time(end) > 0.0001
    start_idx = floor(length(time)*0.9);
    plot(time(start_idx:end)*1000, i_ind_data(start_idx:end), 'LineWidth', 1.5, 'Color', '#D95319');
    xlim([time(start_idx)*1000, time(end)*1000]);
    title('Inductor Current (Zoomed: Last 10%)', 'FontSize', 12);
else
    plot(time*1000, i_ind_data, 'LineWidth', 1.5, 'Color', '#D95319');
    title('Inductor Current', 'FontSize', 12);
end
grid on;
xlabel('Time [ms]'); ylabel('Current [A]');


%% 6. 마무리 및 팁
% - 이 파일을 실행(Run)하면 Workspace에 변수가 로드됩니다.
% - Simulink 모델의 파라미터 창에 숫자 대신 'L_leakage', 'V_pri_DC' 등을 입력하세요.
% - 협업 시 이 .mlx 파일만 공유하면, 모두가 동일한 파라미터 환경에서 시뮬레이션 할 수 있습니다.

%% DAB Converter Average Model Simulation (Integrated Script)


% =========================================================================
% 1. 시스템 파라미터 설정 (SLDD 데이터 기반)
% =========================================================================
fs = 80e3;              % 스위칭 주파수 [Hz]
Ts = 1/fs;              % 샘플링 주기
L_lk = 20e-6;           % 누설 인덕턴스 [H]
n_turn = 3.8;           % 변압기 권선비 (HV:LV)
C_hv = 470e-6;          % HV측 DC 링크 커패시턴스 [F]
R_load = 160;           % HV측 부하 저항 [Ohm] (약 1kW 부하)

V_lv = 100;             % 배터리 전압 (고정 가정)
V_hv_ref = 380;         % 목표 HV 전압 [V]

% 시뮬레이션 시간 설정
T_final = 0.1;          % 100ms 시뮬레이션 (충분한 관찰 시간)
t = 0:Ts:T_final;
n_steps = length(t);

% =========================================================================
% 2. 제어기 게인 (PID Tuner로 조정할 대상)
% =========================================================================
% 전압 루프 (Outer)
Kp_v = 15;            
Ki_v = 150000;
% 전류 루프 (Inner)
Kp_i = 0.05;            
Ki_i = 10;

% =========================================================================
% 3. 상태 변수 초기화
% =========================================================================
V_hv = 340;             % 초기 HV 전압 (정전 상태 가정)
integ_v = 0;            % 전압 적분기
integ_i = 0;            % 전류 적분기

% 결과 저장용 배열
history_Vhv = zeros(1, n_steps);
history_Phi = zeros(1, n_steps);
history_P   = zeros(1, n_steps);
history_Ihv = zeros(1, n_steps);

% =========================================================================
% 4. 시뮬레이션 루프 (Main Loop)
% =========================================================================
for k = 1:n_steps
    % [A] 현재 상태 측정 및 데이터 저장
    history_Vhv(k) = V_hv;
    
    % --- [B] 제어기 시작 (Controller Logic) ---
    % 1. 외곽 전압 제어 루프 (Outer Voltage Loop)
    err_v = V_hv_ref - V_hv;
    integ_v = integ_v + err_v * Ts;
    I_hv_ref = Kp_v * err_v + Ki_v * integ_v;
    I_hv_ref = max(-5, min(5, I_hv_ref)); % HV 전류 제한 (3A~5A)
    
    % 2. 내부 전류 제어 루프 (Inner Current Loop)
    % 실제 흐르는 HV 전류(I_hv)와 지령 비교
    % (평균 모델에서는 P/Vhv가 Ihv임)
    I_hv_actual = history_Ihv(max(1,k-1)); 
    err_i = I_hv_ref - I_hv_actual;
    integ_i = integ_i + err_i * Ts;
    phi_deg = Kp_i * err_i + Ki_i * integ_i;
    phi_deg = max(-90, min(90, phi_deg)); % 위상 제한
    
    history_Phi(k) = phi_deg;
    % --- [B] 제어기 종료 ---

    % --- [C] 플랜 모델링 (DAB Plant Dynamics) ---
    % 1. DAB 전력 전달 수식 (평균 모델 핵심)
    d_ratio = abs(phi_deg) / 180;
    P_dab = (n_turn * V_hv * V_lv / (2 * fs * L_lk)) * d_ratio * (1 - d_ratio);
    if phi_deg < 0, P_dab = -P_dab; end % 방향 결정
    
    history_P(k) = P_dab;
    
    % 2. HV측 전류 및 전압 미분 방정식
    I_dab_hv = P_dab / V_hv;           % DAB가 HV쪽으로 밀어주는 전류
    I_load = V_hv / R_load;            % 부하가 소비하는 전류
    
    history_Ihv(k) = I_dab_hv;
    
    % dV/dt = (I_dab - I_load) / C
    dV_hv = (I_dab_hv - I_load) / C_hv;
    V_hv = V_hv + dV_hv * Ts;          % 오일러 적분
    % --- [C] 플랜 모델링 종료 ---
end

% =========================================================================
% 5. 결과 시각화
% =========================================================================
figure('Color', 'w', 'Position', [100 100 800 600]);

subplot(3,1,1);
plot(t*1000, history_Vhv, 'b', 'LineWidth', 1.5); hold on;
line([0 T_final*1000], [V_hv_ref V_hv_ref], 'Color', 'r', 'LineStyle', '--');
ylabel('HV Voltage [V]'); title('DAB Average Model: Voltage Control');
grid on;

subplot(3,1,2);
plot(t*1000, history_Phi, 'k', 'LineWidth', 1.5);
ylabel('Phase Shift [deg]');
grid on;

subplot(3,1,3);
plot(t*1000, history_P, 'm', 'LineWidth', 1.5);
ylabel('Transfer Power [W]'); xlabel('Time [ms]');
grid on;
