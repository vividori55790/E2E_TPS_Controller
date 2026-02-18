function results = Phase3_Online_Parameter_Estimator(varargin)
% =========================================================================
% Phase3_Online_Parameter_Estimator.m
% DAB 컨버터 TPS 제어: 온라인 인덕턴스(L) 추정 및 적응형 LUT 보정
% =========================================================================
%
% 목적:
%   실시간으로 누설 인덕턴스 L을 추정하여 LUT의 정확도를 유지
%   → 고신뢰성 시스템 (전기차 충전기, 위성 등)을 위한 Advanced 단계
%
% 알고리즘:
%   Recursive Least Squares (RLS) + Exponential Moving Average (EMA)
%
% 원리:
%   P = V1*V2'/(2*pi*f*L) * phi * (1 - |phi|/pi)
%   → L = V1*V2'/(2*pi*f*P) * phi * (1 - |phi|/pi)
%   → 나머지 변수는 센서로 측정 가능, L만 미지수
%
% 사용법:
%   results = Phase3_Online_Parameter_Estimator()
%   results = Phase3_Online_Parameter_Estimator('L_True_Drift', @(t) 50e-6*(1+0.15*(t/1)))
%
% 출력:
%   results.L_estimated  — 시간별 추정 인덕턴스
%   results.L_true       — 실제 인덕턴스 (검증용)
%   results.error_pct    — 추정 오차 (%)
%   results.stats        — 수렴 통계
% =========================================================================

fprintf('=========================================================================\n');
fprintf(' Phase 3: Online Parameter Estimation (RLS)\n');
fprintf(' Real-time Inductance (L) Estimation for Adaptive Control\n');
fprintf('=========================================================================\n\n');

%% ==================== 1. 파라미터 파싱 ====================
cfg = HDC_Config();
p = inputParser;
addParameter(p, 'L_Nominal', cfg.sys.L, @isnumeric);
addParameter(p, 'SimTime', cfg.sim.phase3_time, @isnumeric);
addParameter(p, 'EstimateInterval', cfg.est.Interval, @isnumeric);
addParameter(p, 'RLS_ForgettingFactor', cfg.est.RLS_Lambda, @isnumeric);
addParameter(p, 'EMA_Alpha', cfg.est.EMA_Alpha, @isnumeric);
addParameter(p, 'SNR_dB', 40, @isnumeric);
addParameter(p, 'L_True_Drift', [], @(x) isa(x, 'function_handle') || isempty(x));
addParameter(p, 'SavePath', cfg.path.Est_File);
parse(p, varargin{:});
opts = p.Results;

% 시스템 파라미터
L_nom = opts.L_Nominal;
n     = cfg.sys.n;
fs    = cfg.sys.fs;
P_rated = cfg.sys.P_rated;

omega = 2 * pi * fs;

fprintf('[CONFIG] L_nominal = %.1f uH, P_rated = %.0f W\n', L_nom * 1e6, P_rated);
fprintf('[CONFIG] RLS Lambda = %.4f\n', opts.RLS_ForgettingFactor);

% ==================== 2. 노후화 및 전력 시나리오 ====================
if isempty(opts.L_True_Drift)
    L_true_func = @(t) default_L_drift(t, L_nom);
else
    L_true_func = opts.L_True_Drift;
end

P_ref_func = @(t) default_P_scenario(t, P_rated);

%% ==================== 3. RLS 추정기 초기화 ====================

% RLS 상태
theta = 1 / L_nom;    % 초기 파라미터 추정 (1/L)
P_rls = 1e6;          % 초기 공분산 (큰 값 = 높은 불확실성)
lambda = opts.RLS_ForgettingFactor;  % 망각 계수

% EMA 상태 (이동 평균 필터)
L_ema = L_nom;
alpha_ema = opts.EMA_Alpha;

% 간단한 역산 추정기 (비교용)
L_simple = L_nom;

% 센서 노이즈 계산
noise_power = 10^(-opts.SNR_dB / 10);

%% ==================== 4. 시뮬레이션 루프 ====================
dt_est = opts.EstimateInterval;
N_est  = round(opts.SimTime / dt_est);

% 로깅 배열
log.time      = zeros(N_est, 1);
log.L_true    = zeros(N_est, 1);
log.L_rls     = zeros(N_est, 1);
log.L_ema     = zeros(N_est, 1);
log.L_simple  = zeros(N_est, 1);
log.err_rls   = zeros(N_est, 1);
log.err_ema   = zeros(N_est, 1);
log.err_simple = zeros(N_est, 1);
log.P_ref     = zeros(N_est, 1);
log.Vin       = zeros(N_est, 1);
log.Vout      = zeros(N_est, 1);
log.Phi       = zeros(N_est, 1);
log.P_meas    = zeros(N_est, 1);
log.P_cov     = zeros(N_est, 1);   % RLS 공분산 (불확실성)
log.K_gain    = zeros(N_est, 1);   % RLS 칼만 이득

fprintf('[SIM] Running online estimation (%.1f s, %d samples)...\n', opts.SimTime, N_est);
tic;

convergence_idx = 0;  % 수렴 시점

for k = 1:N_est
    t = (k-1) * dt_est;
    log.time(k) = t;
    
    % === 현재 실제 L (Ground Truth) ===
    L_actual = L_true_func(t);
    log.L_true(k) = L_actual;
    
    % === 현재 전력 지령 ===
    P_ref = P_ref_func(t);
    log.P_ref(k) = P_ref;
    
    % === 전압 시뮬레이션 (약간의 변동 추가) ===
    Vin  = 400 + 10 * sin(2*pi*0.5*t);   % HV측 전압 변동
    Vout = 100 + 2 * sin(2*pi*1.0*t);    % LV측 전압 변동
    log.Vin(k)  = Vin;
    log.Vout(k) = Vout;
    
    % === SPS 위상 계산 (실제 전력 전달에 사용) ===
    V2_prime = Vout * n;
    K = Vin * V2_prime / (omega * L_actual);
    
    if abs(P_ref) < 50
        phi = 0;
    else
        disc = 1 - 4 * abs(P_ref) / (K * pi);
        if disc < 0
            phi = sign(P_ref) * pi / 4;
        else
            phi = sign(P_ref) * (1 - sqrt(disc)) * pi / 2;
        end
    end
    phi = max(-pi/2, min(pi/2, phi));
    log.Phi(k) = phi;
    
    % === 실제 전력 (L_actual 기반) + 센서 노이즈 ===
    P_actual = K * phi * (1 - abs(phi) / pi);
    P_noise = P_actual * sqrt(noise_power) * randn();
    P_measured = P_actual + P_noise;
    log.P_meas(k) = P_measured;
    
    % --- 저전력 구간 건너뛰기 (추정 불안정) ---
    if abs(P_measured) < 100 || abs(phi) < 0.01
        log.L_rls(k)    = 1 / theta;
        log.L_ema(k)    = L_ema;
        log.L_simple(k) = L_simple;
        log.err_rls(k)  = (1/theta - L_actual) / L_actual * 100;
        log.err_ema(k)  = (L_ema - L_actual) / L_actual * 100;
        log.err_simple(k) = (L_simple - L_actual) / L_actual * 100;
        log.P_cov(k)    = P_rls;
        log.K_gain(k)   = 0;
        continue;
    end
    
    % =============================================================
    % 방법 1: RLS (Recursive Least Squares) 추정기
    % =============================================================
    % 관측 모델: P_measured = (V1*V2'/(2*pi*f)) * phi * (1-|phi|/pi) * (1/L)
    %          y = h * theta
    % 여기서:
    %   y = P_measured (관측값)
    %   h = (V1*V2'/(2*pi*f)) * phi * (1-|phi|/pi) (라디안 기반)
    %   theta = 1/L (추정 파라미터)
    
    h = (Vin * V2_prime / (omega)) * phi * (1 - abs(phi) / pi);
    y = P_measured;
    
    % 예측 오차
    e = y - h * theta;
    
    % 칼만 이득
    K_gain = P_rls * h / (lambda + h * P_rls * h);
    
    % 파라미터 업데이트
    theta = theta + K_gain * e;
    
    % 공분산 업데이트
    P_rls = (1 / lambda) * (P_rls - K_gain * h * P_rls);
    
    % L 추정값 (1/theta)
    L_rls = 1 / max(theta, 1e-10);  % 음수 방지
    L_rls = max(10e-6, min(200e-6, L_rls));  % 물리적 범위 제한
    
    % =============================================================
    % 방법 2: EMA (Exponential Moving Average) — 간단한 비교용
    % =============================================================
    % L = h / P_measured 직접 역산
    L_instant = h / max(abs(P_measured), 1);
    L_instant = max(10e-6, min(200e-6, abs(L_instant)));
    L_ema = (1 - alpha_ema) * L_ema + alpha_ema * L_instant;
    
    % =============================================================
    % 방법 3: 단순 역산 (노이즈 필터 없음) — Baseline
    % =============================================================
    L_simple = (1 - 0.01) * L_simple + 0.01 * L_instant;
    
    % === 로깅 ===
    log.L_rls(k)    = L_rls;
    log.L_ema(k)    = L_ema;
    log.L_simple(k) = L_simple;
    log.err_rls(k)  = (L_rls - L_actual) / L_actual * 100;
    log.err_ema(k)  = (L_ema - L_actual) / L_actual * 100;
    log.err_simple(k) = (L_simple - L_actual) / L_actual * 100;
    log.P_cov(k)    = P_rls;
    log.K_gain(k)   = K_gain;
    
    % 수렴 판정 (오차 < 2% 최초 시점)
    if convergence_idx == 0 && abs(log.err_rls(k)) < 2.0 && t > 0.1
        convergence_idx = k;
    end
end

elapsed = toc;
fprintf('[SIM] Complete in %.2f s\n\n', elapsed);

%% ==================== 5. 결과 분석 ====================
fprintf('=========================================================================\n');
fprintf(' Phase 3 — Online Estimation Results\n');
fprintf('=========================================================================\n');

% 수렴 분석
if convergence_idx > 0
    t_conv = log.time(convergence_idx);
    fprintf('  RLS Convergence Time: %.1f ms (error < 2%%)\n', t_conv * 1000);
else
    fprintf('  RLS Convergence: NOT reached (error > 2%%)\n');
end

% 정상상태 오차 (마지막 20%)
idx_ss = floor(N_est * 0.8):N_est;
fprintf('\n  Steady-State Performance (last 20%%):\n');
fprintf('    RLS:    Mean Error = %.3f%%, Std = %.3f%%\n', ...
    mean(abs(log.err_rls(idx_ss))), std(log.err_rls(idx_ss)));
fprintf('    EMA:    Mean Error = %.3f%%, Std = %.3f%%\n', ...
    mean(abs(log.err_ema(idx_ss))), std(log.err_ema(idx_ss)));
fprintf('    Simple: Mean Error = %.3f%%, Std = %.3f%%\n', ...
    mean(abs(log.err_simple(idx_ss))), std(log.err_simple(idx_ss)));

% 노후화 구간별 분석
fprintf('\n  Aging Scenario Analysis:\n');
periods = {
    'Normal (0-0.5s)',    0,   0.5;
    'Gradual (0.5-1.0s)', 0.5, 1.0;
    'Sudden (1.0-1.5s)',  1.0, 1.5;
    'Temp Drift (1.5-2.0s)', 1.5, 2.0;
};

for ip = 1:size(periods, 1)
    idx = log.time >= periods{ip,2} & log.time < periods{ip,3};
    if any(idx)
        fprintf('    [%s] L_true=%.1f±%.1f uH, RLS_err=%.2f%%, EMA_err=%.2f%%\n', ...
            periods{ip,1}, ...
            mean(log.L_true(idx))*1e6, std(log.L_true(idx))*1e6, ...
            mean(abs(log.err_rls(idx))), mean(abs(log.err_ema(idx))));
    end
end

%% ==================== 6. 시각화 ====================
fprintf('\n[PLOT] Generating visualization...\n');
visualize_phase3_results(log, L_nom, convergence_idx);

%% ==================== 7. 결과 저장 ====================
results = struct();
results.log = log;
results.params.L_nominal = L_nom;
results.params.lambda = lambda;
results.params.alpha_ema = alpha_ema;
results.params.estimate_interval = dt_est;
results.stats.convergence_time_s = log.time(max(convergence_idx, 1));
results.stats.ss_error_rls_pct = mean(abs(log.err_rls(idx_ss)));
results.stats.ss_error_ema_pct = mean(abs(log.err_ema(idx_ss)));
results.stats.elapsed_sim = elapsed;
results.meta.timestamp = datestr(now);

save(opts.SavePath, 'results', '-v7.3');
fprintf('\n[SAVE] Results saved to: %s\n', opts.SavePath);

%% ==================== 8. MCU 구현 가이드 출력 ====================
fprintf('\n=========================================================================\n');
fprintf(' MCU Implementation Guide (C / Embedded System)\n');
fprintf('=========================================================================\n');
fprintf(' // Phase 3 Online Estimator — MCU 이식 코드\n');
fprintf(' // 1ms or 100ms 주기로 호출\n');
fprintf(' \n');
fprintf(' // --- RLS 추정기 (핵심) ---\n');
fprintf(' static float theta = %.6f;     // 1/L_nom\n', 1/L_nom);
fprintf(' static float P_cov = 1e6;       // 공분산\n');
fprintf(' const float lambda = %.4f;      // 망각 계수\n', lambda);
fprintf(' \n');
fprintf(' float h = (Vin * Vout * n) / (2*PI*fs) * phi * (1 - fabs(phi)/PI);\n');
fprintf(' float e = P_measured - h * theta;\n');
fprintf(' float K = P_cov * h / (lambda + h * P_cov * h);\n');
fprintf(' theta += K * e;\n');
fprintf(' P_cov = (1/lambda) * (P_cov - K * h * P_cov);\n');
fprintf(' float L_estimated = 1.0f / fmaxf(theta, 1e-10f);\n');
fprintf(' \n');
fprintf(' // --- EMA 백업 추정기 (더 간단) ---\n');
fprintf(' float L_inst = h / fmaxf(fabsf(P_measured), 1.0f);\n');
fprintf(' L_ema = 0.98f * L_ema + 0.02f * L_inst;\n');
fprintf('=========================================================================\n');

end

% =====================================================================
% === 서브 함수들 ===
% =====================================================================

function L = default_L_drift(t, L_nom)
% 기본 인덕턴스 드리프트 시나리오
% 4단계: 정상 → 서서히 → 갑자기 → 온도 변동
if t < 0.5
    % Phase A: 정상 운전
    L = L_nom;
elseif t < 1.0
    % Phase B: 서서히 증가 (열화)
    progress = (t - 0.5) / 0.5;
    L = L_nom * (1 + 0.10 * progress);  % 0% → 10%
elseif t < 1.5
    % Phase C: 갑작스러운 변화 (절연 열화)
    L = L_nom * 1.15;  % 15% 증가 (갑자기)
else
    % Phase D: 온도 드리프트 (주기적 변동)
    L = L_nom * (1.15 + 0.05 * sin(2*pi*2*(t-1.5)));  % ±5% 추가 변동
end
end

function P_ref = default_P_scenario(t, P_rated)
% 다양한 부하 조건 시나리오 (정격 비례)
if t < 0.1
    P_ref = P_rated * 0.1;      % 경부하
elseif t < 0.4
    P_ref = P_rated * 0.6;      % 중부하
elseif t < 0.6
    P_ref = P_rated * 0.2;      % 경부하
elseif t < 0.9
    P_ref = P_rated * 0.8;      % 중부하
elseif t < 1.1
    P_ref = P_rated * 0.4;      % 중부하 (L 변화)
elseif t < 1.4
    P_ref = P_rated * 1.0;      % 최대 부하
elseif t < 1.6
    P_ref = P_rated * 0.3;      % 경부하
else
    P_ref = P_rated * (0.6 + 0.2 * sin(2*pi*1.5*t));  % 동적 부하
end
end

function visualize_phase3_results(log, L_nom, conv_idx)
% =========================================================================
% Phase 3 시각화
% =========================================================================

t = log.time;

figure('Name', 'Phase 3 — Online Parameter Estimation', 'Position', [50 50 1500 1000]);

% --- 1. 인덕턴스 추정 비교 ---
subplot(3,2,1);
plot(t, log.L_true*1e6, 'k-', 'LineWidth', 2.5); hold on;
plot(t, log.L_rls*1e6, 'b-', 'LineWidth', 1.5);
plot(t, log.L_ema*1e6, 'r-', 'LineWidth', 1);
plot(t, log.L_simple*1e6, 'g--', 'LineWidth', 1);
% 수평선 그리기 (호환성 개선: yline -> plot)
xl = xlim;
plot(xl, [L_nom L_nom]*1e6, 'k--', 'LineWidth', 1.0, 'DisplayName', 'L_{nom}');
if conv_idx > 0
    t_conv = t(conv_idx);
    plot([t_conv t_conv], ylim, 'b-.', 'LineWidth', 1.0, 'DisplayName', sprintf('Conv=%.0fms', t_conv*1000));
end
xlabel('Time (s)'); ylabel('Inductance (\muH)');
title('(a) Inductance Estimation: L_{true} vs Estimates');
legend('L_{true}', 'L_{RLS}', 'L_{EMA}', 'L_{Simple}', 'Location', 'best');
grid on;

% --- 2. 추정 오차 (%) ---
subplot(3,2,2);
plot(t, log.err_rls, 'b-', 'LineWidth', 1.5); hold on;
plot(t, log.err_ema, 'r-', 'LineWidth', 1);
plot(t, log.err_simple, 'g--', 'LineWidth', 1);

% 오차 범위선 (plot 사용)
xl = xlim;
plot(xl, [2 2], 'k--', 'HandleVisibility', 'off');
plot(xl, [-2 -2], 'k--', 'HandleVisibility', 'off');
plot(xl, [0 0], 'k-', 'LineWidth', 0.5, 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Error (%)');
title('(b) Estimation Error (%)');
legend('RLS', 'EMA', 'Simple', 'Location', 'best');
ylim([-15 15]);
grid on;

% 노후화 구간 표시
for xi = [0.5, 1.0, 1.5]
    xline(xi, '--r', '', 'Alpha', 0.3);
end

% --- 3. 전력 시나리오 ---
subplot(3,2,3);
plot(t, log.P_ref, 'b-', 'LineWidth', 1.5); hold on;
plot(t, log.P_meas, 'r-', 'LineWidth', 0.5, 'Color', [1 0.3 0.3 0.3]);
xlabel('Time (s)'); ylabel('Power (W)');
title('(c) Power Reference vs Measured');
legend('P_{ref}', 'P_{meas} (noisy)', 'Location', 'best');
grid on;

% --- 4. RLS 내부 상태 ---
subplot(3,2,4);
yyaxis left;
plot(t, log.P_cov, 'b-', 'LineWidth', 1.5);
ylabel('P (Covariance)');
yyaxis right;
plot(t, log.K_gain, 'r-', 'LineWidth', 1);
ylabel('K (Kalman Gain)');
xlabel('Time (s)');
title('(d) RLS Internal States: Covariance & Gain');
grid on;

% --- 5. 전압 & 위상 ---
subplot(3,2,5);
yyaxis left;
plot(t, log.Vin, 'b-', 'LineWidth', 1);
ylabel('V_{in} (V)');
yyaxis right;
plot(t, log.Phi * 180/pi, 'r-', 'LineWidth', 1);
ylabel('\phi (deg)');
xlabel('Time (s)');
title('(e) Input Voltage & Phase');
grid on;

% --- 6. 오차 히스토그램 (정상상태) ---
subplot(3,2,6);
idx_ss = t >= max(t)*0.5;
histogram(log.err_rls(idx_ss), 50, 'FaceColor', [0.2 0.4 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
hold on;
histogram(log.err_ema(idx_ss), 50, 'FaceColor', [0.8 0.3 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
xlabel('Error (%)'); ylabel('Count');
title('(f) Error Distribution (Last 50%)');
legend('RLS', 'EMA', 'Location', 'best');

% 통계 주석
rls_mu = mean(log.err_rls(idx_ss));
rls_std = std(log.err_rls(idx_ss));
ema_mu = mean(log.err_ema(idx_ss));
ema_std = std(log.err_ema(idx_ss));
text(0.05, 0.95, sprintf('RLS: \\mu=%.2f%%, \\sigma=%.2f%%', rls_mu, rls_std), ...
    'Units', 'normalized', 'VerticalAlignment', 'top', 'FontWeight', 'bold', 'Color', 'b');
text(0.05, 0.85, sprintf('EMA: \\mu=%.2f%%, \\sigma=%.2f%%', ema_mu, ema_std), ...
    'Units', 'normalized', 'VerticalAlignment', 'top', 'FontWeight', 'bold', 'Color', 'r');
grid on;

sgtitle(sprintf('Phase 3 — Online Parameter Estimation (RLS vs EMA)\nL_{nom} = %.0f \\muH, \\lambda = %.4f', ...
    L_nom*1e6, 0.995), 'FontSize', 14, 'FontWeight', 'bold');
drawnow;
end
