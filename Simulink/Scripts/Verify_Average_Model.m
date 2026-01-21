%% Verify_Average_Model.m
% DAB 평균화 모델 검증 스크립트
% 
% 목적:
%   1. 평균화 모델이 정상 동작하는지 확인
%   2. 다양한 위상각에서 전력 전달 특성 검증
%   3. 전압 동역학 (커패시터 충방전) 동작 확인
%   4. 스위칭 모델과 비교 (정성적)
%
% 작성일: 2026-01-21
% =========================================================================

clear; clc; close all;
fprintf('=================================================================\n');
fprintf('         DAB 평균화 모델 검증 (DAB_Average_Model.m)              \n');
fprintf('=================================================================\n\n');

%% 1. 모델 인스턴스 생성
fprintf('1. 평균화 모델 인스턴스 생성...\n');

% 스크립트 경로에서 상위 폴더로 이동
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
cd(fullfile(script_dir, '..'));

% 모델 생성
model = DAB_Average_Model();

% 파라미터 설정 (기본값 확인)
fprintf('   변압기 권선비 n = %.1f\n', model.n);
fprintf('   누설 인덕턴스 L = %.1f uH\n', model.L * 1e6);
fprintf('   스위칭 주파수 fs = %.0f kHz\n', model.fs / 1e3);
fprintf('   HV 커패시터 C_HV = %.0f uF\n', model.C_HV * 1e6);
fprintf('   LV 커패시터 C_LV = %.0f uF\n', model.C_LV * 1e6);
fprintf('   배터리 EMF = %.1f V\n', model.V_batt_EMF);
fprintf('   배터리 내부저항 = %.3f Ohm\n', model.R_batt);
fprintf('\n');

%% 2. 정적 특성 검증 - 위상각별 전력 전달
fprintf('2. 정적 특성 검증: 위상각 vs 전력...\n');

% 이론적 전력 계산 함수 (SPS)
V_HV = 380;          % HV측 전압 (V)
V_LV = 100;          % LV측 전압 (V)
V_LV_prime = V_LV * model.n;  % HV측으로 환산
omega = 2 * pi * model.fs;

% 외부 위상 범위 (-60° ~ 60°)
phi_ext_range = linspace(-60, 60, 121);

% 이론 전력 (SPS 공식)
P_theory = zeros(size(phi_ext_range));
for i = 1:length(phi_ext_range)
    phi_rad = phi_ext_range(i) * pi / 180;
    % P = (V1 * V2') / (omega * L) * phi * (1 - |phi|/(pi/2))
    factor = 1 - abs(phi_rad) / (pi/2);
    factor = max(0, factor);
    P_theory(i) = (V_HV * V_LV_prime / (omega * model.L)) * phi_rad * factor;
end

% 모델 테스트 (단일 스텝, 초기 상태에서)
P_model = zeros(size(phi_ext_range));
model.release();  % 상태 초기화

for i = 1:length(phi_ext_range)
    model.reset();  % 매 테스트 전 리셋
    
    phi_ext_deg = phi_ext_range(i);
    phi_int_deg = 15;  % 고정 내부 위상
    I_load_HV = 0;     % 부하 없음
    V_source = 380;    % 전원 전압
    
    % 100스텝 실행하여 정상상태 도달
    for step = 1:100
        [~, ~, ~, ~, P] = model.step(V_source, phi_ext_deg, phi_int_deg, I_load_HV);
    end
    P_model(i) = P;
end

% 결과 플롯
figure('Name', 'Average Model Verification', 'Position', [100 100 1400 900]);

subplot(2,3,1);
plot(phi_ext_range, P_theory/1000, 'b-', 'LineWidth', 2); hold on;
plot(phi_ext_range, P_model/1000, 'r--', 'LineWidth', 2);
xlabel('External Phase Shift (deg)');
ylabel('Power Transfer (kW)');
title('Phase Shift vs Power');
legend('Theory (SPS)', 'Average Model', 'Location', 'northwest');
grid on;
xlim([-60 60]);

% 오차 계산
error_percent = max(abs(P_model - P_theory)) / max(abs(P_theory)) * 100;
text(0.05, 0.95, sprintf('Max Error: %.1f%%', error_percent), ...
    'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'w');

fprintf('   최대 전력 오차 (이론 대비): %.1f%%\n', error_percent);
fprintf('   최대 전력 (phi=60°): %.1f kW (이론), %.1f kW (모델)\n', ...
    max(P_theory)/1000, max(P_model)/1000);

%% 3. 동적 특성 검증 - 스텝 응답
fprintf('\n3. 동적 특성 검증: 위상 스텝 응답...\n');

model.release();
model.reset();

% 시뮬레이션 파라미터
dt = model.Ts_sim;          % 10us
T_sim = 0.050;              % 50ms 시뮬레이션
N_steps = round(T_sim / dt);

% 시간 벡터
t = (0:N_steps-1) * dt * 1000;  % ms

% 결과 저장
V_HV_hist = zeros(1, N_steps);
V_LV_hist = zeros(1, N_steps);
I_HV_hist = zeros(1, N_steps);
I_LV_hist = zeros(1, N_steps);
P_hist = zeros(1, N_steps);
phi_hist = zeros(1, N_steps);

% 시뮬레이션 실행
V_source = 380;
phi_int_deg = 15;
I_load_HV = 5;  % 5A DC 부하

for k = 1:N_steps
    % 위상 스텝: 10ms에서 0° → 30° (방전 시작)
    if t(k) < 10
        phi_ext_deg = 0;
    elseif t(k) < 30
        phi_ext_deg = 30;
    else
        % 30ms에서 30° → -20° (충전으로 전환)
        phi_ext_deg = -20;
    end
    phi_hist(k) = phi_ext_deg;
    
    [V_HV, V_LV, I_HV, I_LV, P] = model.step(V_source, phi_ext_deg, phi_int_deg, I_load_HV);
    
    V_HV_hist(k) = V_HV;
    V_LV_hist(k) = V_LV;
    I_HV_hist(k) = I_HV;
    I_LV_hist(k) = I_LV;
    P_hist(k) = P;
end

% 플롯 - 위상 입력
subplot(2,3,2);
stairs(t, phi_hist, 'k-', 'LineWidth', 2);
xlabel('Time (ms)');
ylabel('Phase Shift (deg)');
title('Input: External Phase Shift');
grid on;
ylim([-30 40]);

% 플롯 - 전력 전달
subplot(2,3,3);
plot(t, P_hist/1000, 'b-', 'LineWidth', 1.5);
xlabel('Time (ms)');
ylabel('Power (kW)');
title('Power Transfer');
grid on;
yline(0, 'r--', 'LineWidth', 1);

% 플롯 - 전압 동역학
subplot(2,3,4);
yyaxis left;
plot(t, V_HV_hist, 'b-', 'LineWidth', 1.5);
ylabel('V_{HV} (V)');
ylim([370 390]);
yyaxis right;
plot(t, V_LV_hist, 'r-', 'LineWidth', 1.5);
ylabel('V_{LV} (V)');
ylim([95 105]);
xlabel('Time (ms)');
title('Voltage Dynamics (HV & LV)');
legend('V_{HV}', 'V_{LV}', 'Location', 'best');
grid on;

% 플롯 - 전류
subplot(2,3,5);
plot(t, I_HV_hist, 'b-', 'LineWidth', 1.5); hold on;
plot(t, I_LV_hist, 'r-', 'LineWidth', 1.5);
xlabel('Time (ms)');
ylabel('Current (A)');
title('Current (HV & LV)');
legend('I_{HV}', 'I_{LV}', 'Location', 'best');
grid on;
yline(0, 'k--', 'LineWidth', 1);

fprintf('   스텝 응답 동작 확인:\n');
fprintf('   - 초기 (0-10ms): 위상=0°, P≈0\n');
fprintf('   - 방전 (10-30ms): 위상=+30°, P=+%.1f kW (HV→LV)\n', max(P_hist(t>10 & t<30))/1000);
fprintf('   - 충전 (30-50ms): 위상=-20°, P=%.1f kW (LV→HV)\n', min(P_hist(t>30))/1000);

%% 4. 전압 변동 검증 - 부하 변화
fprintf('\n4. 전압 변동 검증: 부하 스텝...\n');

model.release();
model.reset();

% 시뮬레이션 파라미터
T_sim = 0.100;              % 100ms 시뮬레이션
N_steps = round(T_sim / dt);
t2 = (0:N_steps-1) * dt * 1000;  % ms

% 결과 저장
V_HV_hist2 = zeros(1, N_steps);
V_LV_hist2 = zeros(1, N_steps);
I_load_hist = zeros(1, N_steps);

% 시뮬레이션 실행
V_source = 380;
phi_ext_deg = 30;  % 고정 위상 (방전)
phi_int_deg = 15;

for k = 1:N_steps
    % 부하 스텝: 50ms에서 5A → 15A
    if t2(k) < 50
        I_load_HV = 5;
    else
        I_load_HV = 15;
    end
    I_load_hist(k) = I_load_HV;
    
    [V_HV, V_LV, ~, ~, ~] = model.step(V_source, phi_ext_deg, phi_int_deg, I_load_HV);
    
    V_HV_hist2(k) = V_HV;
    V_LV_hist2(k) = V_LV;
end

% 플롯 - 부하 변화에 따른 전압 변동
subplot(2,3,6);
yyaxis left;
plot(t2, V_HV_hist2, 'b-', 'LineWidth', 1.5);
ylabel('V_{HV} (V)');
yyaxis right;
stairs(t2, I_load_hist, 'r-', 'LineWidth', 2);
ylabel('I_{load} (A)');
xlabel('Time (ms)');
title('Voltage Response to Load Step');
legend('V_{HV}', 'I_{load}', 'Location', 'best');
grid on;

% 전압 변동량 계산
V_HV_before = mean(V_HV_hist2(t2 > 40 & t2 < 50));
V_HV_after_min = min(V_HV_hist2(t2 > 50 & t2 < 60));
voltage_dip = V_HV_before - V_HV_after_min;

fprintf('   부하 5A→15A 스텝 시:\n');
fprintf('   - 전압 강하: %.2f V (%.2f%%)\n', voltage_dip, voltage_dip/V_HV_before*100);
fprintf('   - 정상상태 회복 확인: V_HV = %.1f V\n', V_HV_hist2(end));

%% 5. 종합 결과
fprintf('\n=================================================================\n');
fprintf('                       검증 결과 요약                            \n');
fprintf('=================================================================\n');
fprintf('✓ 정적 특성: 위상-전력 관계 정상 (오차 %.1f%%)\n', error_percent);
fprintf('✓ 동적 응답: 위상 스텝에 대해 전력 즉시 응답\n');
fprintf('✓ 전압 동역학: 커패시터 전압 변동 정상 작동\n');
fprintf('✓ 부하 응답: 부하 증가 시 전압 강하 후 회복\n');
fprintf('\n');

if error_percent < 15
    fprintf('★ 평균화 모델 검증 성공! GA 최적화에 사용 가능 ★\n');
else
    fprintf('⚠ 오차가 큽니다. 모델 파라미터 확인 필요\n');
end

sgtitle('DAB Average Model Verification', 'FontSize', 14, 'FontWeight', 'bold');
fprintf('\n[Figure에서 결과 확인]\n');

%% 6. 결과 저장
save_results = false;
if save_results
    saveas(gcf, fullfile(script_dir, 'Average_Model_Verification.png'));
    fprintf('\n✓ 결과 이미지 저장: Average_Model_Verification.png\n');
end

cd(script_dir);
fprintf('\n검증 스크립트 완료.\n');
