% DAB_Controller_Tuning_Calculator.m
% DAB 컨버터의 소신호 모델링을 기반으로 최적의 PI 제어기(Kp, Ki) 값을 계산하는 스크립트입니다.
%
% [원리]
% 1. SPS(Single Phase Shift) 제어에서의 평균 전력 전달 공식 사용
%    P = (V_HV * V_LV / (w * L)) * phi * (1 - |phi|/pi)
% 2. 동작점(Operating Point)에서의 선형화 (Linearization)
%    G_plant = d(I_out)/d(phi) = Gain_Process
% 3. 목표 대역폭(Bandwidth) 및 위상 여유(Phase Margin) 설정
% 4. PI 제어기 파라미터(Kp, Ki) 역산

clear; clc;

%% 1. 시스템 파라미터 입력 (System Parameters)
% [H/W Spec & Operating Point]
V_HV_Nom = 380;         % HV 공칭 전압 (V)
V_LV_Nom = 96;          % LV 공칭 전압 (V) (배터리 전압)
L_leakage = 90e-6;      % 변압기 누설 인덕턴스 (H) (Refer to Primary or Total Equivalent)
Fs = 80e3;              % 스위칭 주파수 (Hz)
Ts = 1/Fs;              % 스위칭 주기 (s)
Max_Power = 3000;       % 최대 정격 전력 (W) (예시값)

% [Control Target]
% 목표 대역폭 (Bandwidth, f_c): 보통 스위칭 주파수의 1/10 ~ 1/20
Scaling_Factor = 15;    % Fs / f_c 비율 (10~20 추천)
Target_BW_Hz = Fs / Scaling_Factor; 
Phase_Margin_Deg = 60;  % 목표 위상 여유 (도) (보통 45~60도)

%% 2. 소신호 모델링 (Small Signal Modeling)
w_s = 2 * pi * Fs;      % 스위칭 각주파수 (rad/s)

% 동작점(Operating Point) 계산: 정격 전력에서의 위상각(phi) 추정
% P = (V1*V2)/(w*L) * phi * (1 - phi/pi)  (단, phi는 라디안)
% 근사적으로 phi가 작을 때 P ~ (V1*V2)/(w*L) * phi 로 가정 가능
% 정확한 해를 위해 2차 방정식 근의 공식 사용:
% A*phi^2 - A*pi*phi + P*pi = 0  (where A = V1*V2/(w*L))

K_trans = (V_HV_Nom * V_LV_Nom) / (w_s * L_leakage);
% 정격 전력에서의 위상각 (rad)
% 근사식: phi_op = Max_Power / K_trans; 
% (정확한 식은 비선형이지만, 제어기 설계는 Worst Case(소신호 이득이 최소일 때) 기준으로 함)
% 일반적으로 위상이 0에 가까울수록(경부하) 이득이 가장 크고(시스템이 민감), 
% 위상이 커질수록(대전력) 이득이 감소함.
% 안정성을 위해 이득이 가장 큰 '경부하~중부하' 지점을 기준으로 설계하는 것이 안전함.

phi_op_rad = 0; % 동작점: 0 (가장 민감한 지점 = Worst Case Gain)

% [Plant Gain Calculation]
% 미분: d(I_out)/d(phi)  (전류 제어 기준)
% I_out_avg = P / V_out = (V_in / (w*L)) * phi * (1 - phi/pi)
% Gain_Process = d(I_out)/d(phi) = (V_in / (w*L)) * (1 - 2*phi/pi)

% HV측 전류 제어 기준 (HV Current Control)
% Gain_Plant = d(I_HV)/d(phi)
Gain_Plant_HV = (V_LV_Nom / (w_s * L_leakage)) * (1 - 2*abs(phi_op_rad)/pi);

% LV측 전류 제어 기준 (LV Current Control)
% Gain_Plant = d(I_LV)/d(phi)
Gain_Plant_LV = (V_HV_Nom / (w_s * L_leakage)) * (1 - 2*abs(phi_op_rad)/pi);

% 이득이 단위가 [A/rad] 이므로, 제어기가 도(degree)를 출력한다면 변환 필요
% 제어기 출력(deg) -> 플랜트 입력(rad): * (pi/180)
Gain_Plant_Total = Gain_Plant_HV * (pi/180); % [A/deg] (HV 전류 제어 시)
% 만약 LV 전류를 제어한다면 아래 줄 주석 해제
% Gain_Plant_Total = Gain_Plant_LV * (pi/180); % [A/deg]

fprintf('>> [System Model] Plant Gain (Kp_plant): %.4f [A/deg]\n', Gain_Plant_Total);


%% 3. 제어기 튜닝 (Controller Design)
% PI 제어기 전달함수: Gc(s) = Kp + Ki/s = Kp * (s + Ki/Kp) / s
% 전체 루프 이득: L(s) = Gc(s) * G_plant(s) * e^(-s*Td)
% (여기서 e^(-s*Td)는 딜레이, s=j*w_c 근처에서 위상 지연만 고려)

w_c = 2 * pi * Target_BW_Hz; % 목표 교차 각주파수 (rad/s)

% 1) 비례 이득 (Kp) 결정
% 목표: |L(j*w_c)| = 1  => |Gc| * |G_plant| = 1
% 고주파수(w_c)에서 PI 제어기는 거의 Kp로 동작 (Ki/s 영향 미미)
% Kp * Gain_Plant = 1
% Kp = 1 / Gain_Plant

Kp_Cal = 1 / Gain_Plant_Total;

% 하지만 디지털 제어 지연(Digital Delay, Td = 1.5*Ts)과 샘플링 효과 고려 시
% 대역폭이 너무 높으면 위상 여유가 깎임.
% 통상적으로 Kp를 약간 보수적으로(작게) 잡음. (0.5 ~ 0.8 배)
Safety_Factor = 0.5; 
Kp_Final = Kp_Cal * Safety_Factor;

% 2) 적분 이득 (Ki) 결정
% 영점(Zero) 배치법: w_z = Ki / Kp
% 영점을 대역폭(w_c)의 1/10 ~ 1/5 지점에 배치하여 위상 지연을 최소화
% w_z = w_c / 10  (충분히 느리게)
w_z = w_c / 10;
Ki_Final = Kp_Final * w_z;

%% 4. 결과 출력
fprintf('\n======================================================\n');
fprintf(' DUAL ACTIVE BRIDGE (DAB) CONTROLLER TUNING RESULT\n');
fprintf('======================================================\n');
fprintf(' 1. System Specifications:\n');
fprintf('    - V_HV: %.1f V, V_LV: %.1f V\n', V_HV_Nom, V_LV_Nom);
fprintf('    - L: %.2f uH, Fs: %.1f kHz\n', L_leakage*1e6, Fs/1e3);
fprintf('    - Target Bandwidth: %.1f Hz (Fs/%d)\n', Target_BW_Hz, Scaling_Factor);
fprintf('\n');
fprintf(' 2. Modeling Results:\n');
fprintf('    - Plant Gain (Small Signal): %.4f A/deg\n', Gain_Plant_Total);
fprintf('      (Meaning: 1 degree phase shift -> %.2f A current change)\n', Gain_Plant_Total);
fprintf('\n');
fprintf(' 3. Recommended PI Gains (Calculated):\n');
fprintf('    >> Kp (Proportional) : %.5f\n', Kp_Final);
fprintf('    >> Ki (Integral)     : %.5f\n', Ki_Final);
fprintf('======================================================\n');
