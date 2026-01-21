% =========================================================================
% GA_Optimize_Controller.m (v2.0)
% 유전 알고리즘을 사용한 DAB 제어기 파라미터 최적화
% =========================================================================
% 개선사항:
%   - Kp, Ki 범위 100배 확장
%   - 실제 Simulink 출력 (out.Current_Low, out.I_cmd_out 등) 직접 사용
%   - 적분기 포화, 위상 변동 등 추가 평가 지표
%   - 수렴 시 조기 종료, 병렬 처리 지원
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  DAB Controller GA Optimization v2.0\n');
fprintf('  Using Genetic Algorithm for Kp, Ki Tuning\n');
fprintf('=================================================================\n\n');

%% ==================== GA Settings ====================
% 최적화 대상: [Kp_ext, Ki_ext, Kpv, Kiv]
% Kp_ext, Ki_ext: 전류 PI (양수)
% Kpv, Kiv: 전압 PI (음수, UPS 모드용)

% 하한: [Kp_ext_min, Ki_ext_min, Kpv_min, Kiv_min]
lb = [0.1, 1, -50.0, -5000];

% 상한: [Kp_ext_max, Ki_ext_max, Kpv_max, Kiv_max]
ub = [50.0, 5000, -0.1, -10];

nvars = 4;  % 변수 개수: 4개

% GA 옵션 설정 (개선됨)
options = optimoptions('ga', ...
    'Display', 'iter', ...
    'PopulationSize', 30, ...           % 개체 수 증가
    'MaxGenerations', 50, ...           % 최대 세대 증가
    'MaxStallGenerations', 10, ...      % 10세대 동안 개선 없으면 종료
    'FunctionTolerance', 1e-6, ...      % 수렴 허용오차
    'EliteCount', 3, ...                % 엘리트 수
    'CrossoverFraction', 0.8, ...       % 교차 비율
    'MutationFcn', {@mutationadaptfeasible}, ...
    'UseParallel', false, ...           % 병렬 처리 (true로 변경 시 parpool 필요)
    'PlotFcn', {@gaplotbestf, @gaplotmeanscores, @gaplotstopping}, ...
    'OutputFcn', @ga_output_callback);   % 진행 상황 출력

%% ==================== Global Variables ====================
global best_result sim_count;
best_result = struct('Kp_ext', 0, 'Ki_ext', 0, 'Kpv', 0, 'Kiv', 0, 'cost', inf, 'metrics', []);
sim_count = 0;

%% ==================== Initial Setup (한 번만) ====================
fprintf('=== 초기 설정 ===\n');

% Scripts 폴더 확인 및 경로 설정
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

% 상위 폴더로 이동하여 Config 로드
cd(script_dir);
addpath('..');
addpath('../Modules');
addpath('../Data');

% 시나리오 생성
fprintf('시나리오 생성 중...\n');
run('Create_Scenario.m');

% Config 로드
config = DAB_System_Config();

% Base Workspace에 필요한 파라미터 할당
setup_base_workspace(config);

% 모델 경로로 이동
modules_dir = fullfile(script_dir, '..', 'Modules');
cd(modules_dir);

% 모델 로드
fprintf('Simulink 모델 로드 중...\n');
load_system('Control_Logic');

% Normal 모드 설정 (Rapid Accel 빌드 오류 회피)
fprintf('Normal 시뮬레이션 모드 설정...\n');
set_param('Control_Logic', 'SimulationMode', 'normal');
fprintf('✓ Normal 모드 활성화 (안정적)\n');

% 초기 테스트 시뮬레이션 (모델 정상 동작 확인)
fprintf('초기 테스트 시뮬레이션...\n');
try
    sim('Control_Logic', 'StopTime', '0.1');
    
    % Base Workspace에서 'out' 변수 가져오기 (To Workspace 블록 출력)
    if evalin('base', 'exist(''out'', ''var'')')
        out = evalin('base', 'out');
        fprintf('\n✓ Base Workspace에서 out 변수 발견\n');
        
        % out 구조체의 필드 확인
        fprintf('\n사용 가능한 출력 필드:\n');
        out_fields = fieldnames(out);
        for i = 1:length(out_fields)
            fprintf('  - out.%s\n', out_fields{i});
        end
        fprintf('\n');
        
        % 핵심 필드 확인
        required_fields = {'Current_Low', 'I_cmd_out', 'Integrator_out', 'Phi_ext_out', 'Status_flag'};
        for i = 1:length(required_fields)
            if isfield(out, required_fields{i})
                fprintf('✓ %s 확인됨\n', required_fields{i});
            else
                fprintf('✗ %s 없음\n', required_fields{i});
            end
        end
    else
        fprintf('✗ Base Workspace에 out 변수가 없습니다.\n');
        fprintf('  To Workspace 블록이 out으로 설정되어 있는지 확인하세요.\n');
    end
    
    fprintf('\n✓ 모델 테스트 완료\n');
catch ME
    fprintf('✗ 모델 테스트 실패: %s\n', ME.message);
    fprintf('\n=== 상세 오류 정보 ===\n');
    for i = 1:length(ME.cause)
        fprintf('원인 %d: %s\n', i, ME.cause{i}.message);
    end
    fprintf('\n스택 트레이스:\n');
    for i = 1:min(5, length(ME.stack))
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    fprintf('\n모델을 직접 열어서 확인해보세요:\n');
    fprintf('  >> open_system(''Control_Logic'')\n');
    return;
end

cd(script_dir);

%% ==================== Run GA ====================
fprintf('\n=================================================================\n');
fprintf('  유전 알고리즘 최적화 시작\n');
fprintf('=================================================================\n');
fprintf('  Kp 범위: [%.4f, %.1f]  (100배 확장)\n', lb(1), ub(1));
fprintf('  Ki 범위: [%.4f, %.1f]  (100배 확장)\n', lb(2), ub(2));
fprintf('  Population: %d, Max Generations: %d\n', ...
    options.PopulationSize, options.MaxGenerations);
fprintf('  조기 종료: %d 세대 연속 무개선 시\n', options.MaxStallGenerations);
fprintf('  초반 무시 구간: 0 ~ 100ms (과도 응답)\n');
fprintf('=================================================================\n\n');

tic;

try
    [x_opt, fval, exitflag, output] = ga(@(x) fitness_function(x, script_dir), ...
        nvars, [], [], [], [], lb, ub, [], options);
catch ME
    % Ctrl+C로 중단되었거나 오류 발생
    fprintf('\n\n=================================================================\n');
    fprintf('  최적화 중단됨 (Ctrl+C 또는 오류)\n');
    fprintf('=================================================================\n');
    fprintf('오류 메시지: %s\n', ME.message);
    
    % 현재까지 찾은 최적값 사용
    global best_result;
    if ~isinf(best_result.cost)
        fprintf('\n현재까지 찾은 최적값:\n');
        fprintf('  Kp_ext: %.6f\n', best_result.Kp_ext);
        fprintf('  Ki_ext: %.6f\n', best_result.Ki_ext);
        fprintf('  Kpv: %.6f\n', best_result.Kpv);
        fprintf('  Kiv: %.6f\n', best_result.Kiv);
        fprintf('  Cost: %.6f\n', best_result.cost);
        x_opt = [best_result.Kp_ext, best_result.Ki_ext, best_result.Kpv, best_result.Kiv];
        fval = best_result.cost;
    else
        fprintf('최적값을 찾지 못했습니다. 기본값 사용.\n');
        x_opt = [2.0, 30.0, -4.0, -400.0];  % 기본값
        fval = inf;
    end
    output.generations = 0;
end

elapsed = toc;

%% ==================== Results ====================
fprintf('\n=================================================================\n');
fprintf('  최적화 완료!\n');
fprintf('=================================================================\n');
fprintf('  최적 Kp_ext: %.6f (전류 비례)\n', x_opt(1));
fprintf('  최적 Ki_ext: %.6f (전류 적분)\n', x_opt(2));
fprintf('  최적 Kpv:    %.6f (전압 비례, UPS)\n', x_opt(3));
fprintf('  최적 Kiv:    %.6f (전압 적분, UPS)\n', x_opt(4));
fprintf('  최적 비용 함수 값: %.6f\n', fval);
fprintf('  총 소요 시간: %.1f 분\n', elapsed/60);
fprintf('  총 시뮬레이션 횟수: %d\n', sim_count);
fprintf('  세대 수: %d\n', output.generations);
fprintf('=================================================================\n');

% 최적 결과 상세 출력
if ~isinf(best_result.cost)
    fprintf('\n=== 최적 결과 상세 ===\n');
    fprintf('  추종 오차 (RMS): %.4f A\n', best_result.metrics.tracking_error);
    fprintf('  오버슈트: %.2f %%\n', best_result.metrics.overshoot);
    fprintf('  정상상태 오차: %.2f %%\n', best_result.metrics.ss_error);
    fprintf('  적분기 포화율: %.1f %%\n', best_result.metrics.integrator_saturation);
end

%% ==================== Apply Optimal Values ====================
fprintf('\n최적값을 DAB_Controller_System.m에 적용하시겠습니까?\n');
fprintf('  Kp_ext: %.6f -> %.6f\n', 2.0, x_opt(1));
fprintf('  Ki_ext: %.6f -> %.6f\n', 30.0, x_opt(2));
fprintf('  Kpv:    %.6f -> %.6f\n', -4.0, x_opt(3));
fprintf('  Kiv:    %.6f -> %.6f\n', -400.0, x_opt(4));

user_input = input('적용하려면 Y를 입력하세요: ', 's');
if strcmpi(user_input, 'Y')
    apply_optimal_values(x_opt(1), x_opt(2), x_opt(3), x_opt(4), script_dir);
    fprintf('✓ 최적값 적용 완료!\n');
else
    fprintf('적용을 취소했습니다.\n');
    fprintf('수동 적용: DAB_Controller_System.m 에서\n');
    fprintf('  Kp_ext = %.6f;\n', x_opt(1));
    fprintf('  Ki_ext = %.6f;\n', x_opt(2));
    fprintf('  Kpv = %.6f;\n', x_opt(3));
    fprintf('  Kiv = %.6f;\n', x_opt(4));
end

%% ==================== Verify with Optimal Values ====================
fprintf('\n최적 파라미터로 검증 시뮬레이션 실행 중...\n');
run_verification(x_opt, script_dir);

%% ==================== Save Results ====================
results.optimal_Kp_ext = x_opt(1);
results.optimal_Ki_ext = x_opt(2);
results.optimal_Kpv = x_opt(3);
results.optimal_Kiv = x_opt(4);
results.cost = fval;
results.elapsed_time = elapsed;
results.generations = output.generations;
results.sim_count = sim_count;
results.best_metrics = best_result.metrics;
results.timestamp = datetime('now');

save('GA_Optimization_Results.mat', 'results');
fprintf('\n결과가 GA_Optimization_Results.mat에 저장되었습니다.\n');

%% ==================== Fitness Function ====================
function cost = fitness_function(x, script_dir)
    % x = [Kp_ext, Ki_ext, Kpv, Kiv]
    Kp_ext = x(1);
    Ki_ext = x(2);
    Kpv = x(3);
    Kiv = x(4);
    
    global best_result sim_count;
    sim_count = sim_count + 1;
    
    % 유효성 검사 (Kp_ext, Ki_ext는 양수, Kpv, Kiv는 음수)
    if Kp_ext <= 0 || Ki_ext <= 0 || Kpv >= 0 || Kiv >= 0
        cost = 1e8;
        return;
    end
    
    try
        % Modules 폴더로 이동
        modules_dir = fullfile(script_dir, '..', 'Modules');
        cd(modules_dir);
        
        % === From Workspace를 통한 개별 파라미터 전달 ===
        % 각 제어계수를 별도의 timeseries로 생성
        SIM_TIME = evalin('base', 'SIM_TIME');
        
        % 시간 벡터 (시작, 끝)
        t_params = [0; SIM_TIME];
        
        % 개별 파라미터 timeseries 생성 및 Base Workspace에 할당
        ts_Kp = timeseries(repmat(Kp_ext, 2, 1), t_params);
        ts_Kp.Name = 'Kp_ext';
        assignin('base', 'Kp_ext', ts_Kp);
        
        ts_Ki = timeseries(repmat(Ki_ext, 2, 1), t_params);
        ts_Ki.Name = 'Ki_ext';
        assignin('base', 'Ki_ext', ts_Ki);
        
        ts_Kpv = timeseries(repmat(Kpv, 2, 1), t_params);
        ts_Kpv.Name = 'Kpv';
        assignin('base', 'Kpv', ts_Kpv);
        
        ts_Kiv = timeseries(repmat(Kiv, 2, 1), t_params);
        ts_Kiv.Name = 'Kiv';
        assignin('base', 'Kiv', ts_Kiv);
        
        % UseExternalParams 플래그도 설정 (MATLAB System 블록에서 사용)
        assignin('base', 'UseExternalParams', true);
        
        % 시뮬레이션 (결과는 Base Workspace의 'out'에 저장됨)
        sim('Control_Logic', 'StopTime', num2str(SIM_TIME));
        
        % Base Workspace에서 out 가져오기
        out = evalin('base', 'out');
        
        cd(script_dir);
        
        % 실제 Simulink 출력으로 성능 평가
        [cost, metrics] = evaluate_performance_v2(out);
        
        % 최적 결과 업데이트
        if cost < best_result.cost
            best_result.Kp_ext = Kp_ext;
            best_result.Ki_ext = Ki_ext;
            best_result.Kpv = Kpv;
            best_result.Kiv = Kiv;
            best_result.cost = cost;
            best_result.metrics = metrics;
            fprintf('  *** 새로운 최적값! Kp=%.3f, Ki=%.1f, Kpv=%.2f, Kiv=%.1f, Cost=%.4f ***\n', ...
                Kp_ext, Ki_ext, Kpv, Kiv, cost);
        end
        
        % 진행 상황 출력 (10회마다)
        if mod(sim_count, 10) == 0
            fprintf('  [%d] Kp=%.3f, Ki=%.1f, Kpv=%.2f, Kiv=%.1f -> Cost=%.4f\n', ...
                sim_count, Kp_ext, Ki_ext, Kpv, Kiv, cost);
        end
        
    catch ME
        cd(script_dir);
        fprintf('  [오류] Kp=%.3f, Ki=%.1f, Kpv=%.2f, Kiv=%.1f: %s\n', ...
            Kp_ext, Ki_ext, Kpv, Kiv, ME.message);
        cost = 1e7;  % 큰 패널티
    end
end

%% ==================== Performance Evaluation v2 ====================
function [cost, metrics] = evaluate_performance_v2(out)
    % 실제 Simulink 출력 사용:
    %   out.Current_Low      - LV 전류 (피드백)
    %   out.I_cmd_out        - 전류 지령
    %   out.Integrator_out   - 적분기 상태
    %   out.Phi_ext_out      - 외부 위상
    %   out.Status_flag      - 모드 (0:ESS, 1:UPS)
    
    IGNORE_TIME = 0.1;  % 첫 100ms 무시 (과도 응답)
    
    metrics = struct();
    
    try
        % === 핵심 데이터 추출 (timeseries 형식) ===
        % Current_Low (LV 전류)
        if isfield(out, 'Current_Low') && isa(out.Current_Low, 'timeseries')
            t = out.Current_Low.Time;
            I_LV = out.Current_Low.Data(:);
        else
            cost = 1e8;
            metrics.error = 'Current_Low not found or not timeseries';
            return;
        end
        
        % I_cmd_out (전류 지령)
        if isfield(out, 'I_cmd_out') && isa(out.I_cmd_out, 'timeseries')
            I_ref = out.I_cmd_out.Data(:);
        else
            cost = 1e8;
            metrics.error = 'I_cmd_out not found or not timeseries';
            return;
        end
        
        % Integrator_out (적분기 상태)
        if isfield(out, 'Integrator_out') && isa(out.Integrator_out, 'timeseries')
            integrator = out.Integrator_out.Data(:);
        else
            integrator = zeros(size(t));
        end
        
        % Phi_ext_out (외부 위상)
        if isfield(out, 'Phi_ext_out') && isa(out.Phi_ext_out, 'timeseries')
            phi_ext = out.Phi_ext_out.Data(:);
        else
            phi_ext = zeros(size(t));
        end
        
        % Status_flag (모드)
        if isfield(out, 'Status_flag') && isa(out.Status_flag, 'timeseries')
            mode = out.Status_flag.Data(:);
        else
            mode = zeros(size(t));
        end
        
        % === 무시 구간 이후 데이터만 사용 ===
        valid_idx = t >= IGNORE_TIME;
        t_valid = t(valid_idx);
        I_LV_valid = I_LV(valid_idx);
        I_ref_valid = I_ref(valid_idx);
        integrator_valid = integrator(valid_idx);
        phi_valid = phi_ext(valid_idx);
        
        if length(I_LV_valid) < 10
            cost = 1e8;
            metrics.error = 'Insufficient data after ignore period';
            return;
        end
        
        % === 1. 추종 오차 (Tracking Error) ===
        error_signal = I_ref_valid - I_LV_valid;
        tracking_error = sqrt(mean(error_signal.^2));  % RMS Error
        metrics.tracking_error = tracking_error;
        
        % === 2. 오버슈트 (Overshoot) ===
        overshoot = calculate_overshoot_v2(t_valid, I_LV_valid, I_ref_valid);
        metrics.overshoot = overshoot;
        
        % === 3. 정상상태 오차 (Steady-State Error) ===
        % 마지막 10% 구간의 평균
        ss_start_idx = round(0.9 * length(t_valid));
        I_ss = mean(I_LV_valid(ss_start_idx:end));
        I_ref_final = I_ref_valid(end);
        
        if abs(I_ref_final) > 0.1
            ss_error = abs(I_ss - I_ref_final) / abs(I_ref_final) * 100;
        else
            ss_error = abs(I_ss - I_ref_final) * 10;  % 작은 값일 때 스케일링
        end
        metrics.ss_error = ss_error;
        
        % === 4. 정착 시간 (Settling Time) ===
        settling_time = calculate_settling_time_v2(t_valid, I_LV_valid, I_ref_valid);
        metrics.settling_time = settling_time;
        
        % === 5. 적분기 포화율 (Integrator Saturation) ===
        % 적분기가 Anti-windup 한계(±3.0)에 도달한 비율
        INTEGRATOR_MAX = 3.0;
        saturation_count = sum(abs(integrator_valid) >= INTEGRATOR_MAX * 0.95);
        integrator_saturation = saturation_count / length(integrator_valid) * 100;
        metrics.integrator_saturation = integrator_saturation;
        
        % === 6. 위상 변동 (Phase Variation) ===
        % 위상의 급격한 변화는 불안정을 나타냄
        phi_diff = diff(phi_valid);
        phase_variation = std(phi_diff);
        metrics.phase_variation = phase_variation;
        
        % === 7. 진동 검출 (Oscillation Detection) ===
        % 영점 교차 횟수로 진동 감지
        zero_crossings = sum(abs(diff(sign(error_signal))) > 0);
        oscillation_index = zero_crossings / length(error_signal) * 100;
        metrics.oscillation_index = oscillation_index;
        
        % === 비용 함수 (가중합) ===
        w_track = 15.0;         % 추종 오차 (가장 중요)
        w_overshoot = 8.0;      % 오버슈트
        w_ss = 20.0;            % 정상상태 오차 (매우 중요)
        w_settling = 3.0;       % 정착 시간
        w_saturation = 5.0;     % 적분기 포화
        w_oscillation = 2.0;    % 진동
        
        cost = w_track * tracking_error + ...
               w_overshoot * overshoot + ...
               w_ss * ss_error + ...
               w_settling * settling_time * 10 + ...  % 초 단위를 스케일링
               w_saturation * integrator_saturation / 10 + ...
               w_oscillation * oscillation_index;
        
        % === 불안정 시스템 큰 패널티 ===
        if any(abs(I_LV_valid) > 100)  % 전류 폭주
            cost = cost + 1e6;
        end
        if any(isnan(I_LV_valid)) || any(isinf(I_LV_valid))
            cost = cost + 1e7;
        end
        if tracking_error > 50  % 추종 실패
            cost = cost + 1e5;
        end
        
    catch ME
        cost = 1e8;
        metrics.error = ME.message;
    end
end

%% ==================== Helper Functions ====================
function overshoot = calculate_overshoot_v2(t, I_out, I_ref)
    % 오버슈트 계산 (개선됨)
    overshoot = 0;
    
    % 레퍼런스 변화 감지
    ref_diff = diff(I_ref);
    ref_changes = find(abs(ref_diff) > 0.5);
    
    for i = 1:length(ref_changes)
        idx = ref_changes(i);
        
        % 변화 후 충분한 데이터 확보
        window_end = min(idx + 500, length(I_out));  % ~5ms window
        if window_end - idx < 50
            continue;
        end
        
        % 새로운 목표값 (변화 후 안정된 값)
        I_ref_new = I_ref(min(idx + 20, length(I_ref)));
        I_window = I_out(idx:window_end);
        
        if abs(I_ref_new) < 0.1
            continue;  % 0 근처는 무시
        end
        
        if I_ref_new > 0
            peak = max(I_window);
            os = (peak - I_ref_new) / abs(I_ref_new) * 100;
        else
            peak = min(I_window);
            os = (I_ref_new - peak) / abs(I_ref_new) * 100;
        end
        
        overshoot = max(overshoot, max(0, os));
    end
end

function settling = calculate_settling_time_v2(t, I_out, I_ref)
    % 정착 시간 계산 (5% 밴드 기준)
    settling = 0;
    tolerance = 0.05;  % 5%
    
    ref_diff = diff(I_ref);
    ref_changes = find(abs(ref_diff) > 0.5);
    
    for i = 1:length(ref_changes)
        idx = ref_changes(i);
        
        if idx + 100 > length(I_ref)
            continue;
        end
        
        I_ref_new = I_ref(min(idx + 20, length(I_ref)));
        band = max(abs(I_ref_new) * tolerance, 0.2);  % 최소 0.2A 밴드
        
        % 정착 지점 찾기
        for j = idx:length(I_out)-50
            remaining = I_out(j:end);
            if all(abs(remaining - I_ref_new) <= band)
                local_settling = t(j) - t(idx);
                settling = max(settling, local_settling);
                break;
            end
        end
    end
    
    % 정착하지 못한 경우 큰 값
    if settling == 0 && ~isempty(ref_changes)
        settling = 1.0;  % 1초 (큰 패널티)
    end
end

function [state, options, optchanged] = ga_output_callback(options, state, flag)
    % GA 진행 상황 콜백
    optchanged = false;
    
    if strcmp(flag, 'iter')
        if mod(state.Generation, 5) == 0
            fprintf('\n=== 세대 %d 완료 ===\n', state.Generation);
            fprintf('  현재 최적: %.4f\n', state.Best(end));
            fprintf('  평균 비용: %.4f\n', mean(state.Score));
        end
    end
end

function setup_base_workspace(config)
    % Base Workspace에 필요한 파라미터 할당
    
    % Plant Parameters
    assignin('base', 'V_pri_DC', config.hv.V_nom);
    assignin('base', 'V_batt_nom', config.battery.V_nom);
    assignin('base', 'P_rated', config.plant.P_rated);
    assignin('base', 'f_sw', config.plant.fs);
    assignin('base', 'Ts', 1/config.plant.fs);
    assignin('base', 'N_pri', config.plant.n);
    assignin('base', 'N_sec', 1);
    assignin('base', 'L_leakage', config.plant.L);
    assignin('base', 'C_dc_link', config.plant.C_out);
    assignin('base', 'R_wire', 1e-3);
    assignin('base', 'Ron_mosfet', 5e-3);
    assignin('base', 'R_snubber', 1e5);
    assignin('base', 'C_snubber', inf);
    assignin('base', 'I_ess_cmd', config.test.I_ref);
    assignin('base', 'kp_voltage', 5);
    assignin('base', 'ki_voltage', 5000);
    
    % TPS Table 로드
    tps_paths = {'TPS_Table.mat', '../TPS_Table.mat', fullfile('..', 'TPS_Table.mat')};
    for i = 1:length(tps_paths)
        if isfile(tps_paths{i})
            load(tps_paths{i}, 'TPS_Table');
            assignin('base', 'TPS_Table', TPS_Table);
            break;
        end
    end
    
    fprintf('✓ Base Workspace 파라미터 설정 완료\n');
end

function apply_optimal_values(Kp_ext, Ki_ext, Kpv, Kiv, script_dir)
    % DAB_Controller_System.m에 최적값 적용 (기본값 변경)
    
    controller_file = fullfile(script_dir, '..', 'DAB_Controller_System.m');
    
    if ~isfile(controller_file)
        fprintf('제어기 파일을 찾을 수 없습니다: %s\n', controller_file);
        return;
    end
    
    % 파일 읽기
    fid = fopen(controller_file, 'r');
    content = fread(fid, '*char')';
    fclose(fid);
    
    % Kp_ext 값 교체
    content = regexprep(content, ...
        'Kp_ext = [\d.-]+;', ...
        sprintf('Kp_ext = %.6f;', Kp_ext));
    
    % Ki_ext 값 교체
    content = regexprep(content, ...
        'Ki_ext = [\d.-]+;', ...
        sprintf('Ki_ext = %.6f;', Ki_ext));
    
    % Kpv 값 교체
    content = regexprep(content, ...
        'Kpv = [\d.-]+;', ...
        sprintf('Kpv = %.6f;', Kpv));
    
    % Kiv 값 교체
    content = regexprep(content, ...
        'Kiv = [\d.-]+;', ...
        sprintf('Kiv = %.6f;', Kiv));
    
    % 파일 쓰기
    fid = fopen(controller_file, 'w');
    fwrite(fid, content);
    fclose(fid);
    
    fprintf('✓ DAB_Controller_System.m 기본값 업데이트 완료\n');
end

function run_verification(x_opt, script_dir)
    % 최적 파라미터로 검증 시뮬레이션 실행
    Kp_ext = x_opt(1);
    Ki_ext = x_opt(2);
    Kpv = x_opt(3);
    Kiv = x_opt(4);
    
    fprintf('\n=================================================================\n');
    fprintf('  검증 시뮬레이션\n');
    fprintf('  Kp_ext=%.4f, Ki_ext=%.2f, Kpv=%.4f, Kiv=%.2f\n', ...
        Kp_ext, Ki_ext, Kpv, Kiv);
    fprintf('=================================================================\n');
    
    modules_dir = fullfile(script_dir, '..', 'Modules');
    cd(modules_dir);
    
    % === From Workspace를 통한 파라미터 전달 ===
    SIM_TIME = evalin('base', 'SIM_TIME');
    
    % 시간 벡터 (시작, 끝)
    t_params = [0; SIM_TIME];
    
    % 개별 파라미터 timeseries 생성 및 Base Workspace에 할당
    ts_Kp = timeseries(repmat(Kp_ext, 2, 1), t_params);
    ts_Kp.Name = 'Kp_ext';
    assignin('base', 'Kp_ext', ts_Kp);
    
    ts_Ki = timeseries(repmat(Ki_ext, 2, 1), t_params);
    ts_Ki.Name = 'Ki_ext';
    assignin('base', 'Ki_ext', ts_Ki);
    
    ts_Kpv = timeseries(repmat(Kpv, 2, 1), t_params);
    ts_Kpv.Name = 'Kpv';
    assignin('base', 'Kpv', ts_Kpv);
    
    ts_Kiv = timeseries(repmat(Kiv, 2, 1), t_params);
    ts_Kiv.Name = 'Kiv';
    assignin('base', 'Kiv', ts_Kiv);
    
    % UseExternalParams 플래그 설정
    assignin('base', 'UseExternalParams', true);
    
    fprintf('✓ From Workspace로 제어기 파라미터 설정 완료 (개별 변수)\n');
    
    % 시뮬레이션 (결과는 Base Workspace의 'out'에 저장됨)
    sim('Control_Logic', 'StopTime', num2str(SIM_TIME));
    
    % Base Workspace에서 out 가져오기
    out = evalin('base', 'out');
    
    cd(script_dir);
    
    % 결과 플롯
    figure('Name', 'GA Optimization Verification', 'Position', [50 50 1600 1000]);
    
    % timeseries 데이터 추출 (To Workspace 출력)
    % out.Current_Low.Data, out.Current_Low.Time 형태
    
    % 시간축 추출 (ms로 변환) - 첫 번째 유효한 timeseries에서 가져옴
    if isfield(out, 'Current_Low') && isa(out.Current_Low, 'timeseries')
        t = out.Current_Low.Time * 1000;  % ms로 변환
    elseif isfield(out, 'Voltage_High') && isa(out.Voltage_High, 'timeseries')
        t = out.Voltage_High.Time * 1000;
    else
        t = (0:0.001:1)' * 1000;  % fallback
    end
    
    % 1. 전류 추종 (핵심)
    subplot(3,3,1);
    if isfield(out, 'Current_Low') && isfield(out, 'I_cmd_out')
        I_lv = out.Current_Low.Data(:);
        I_ref = out.I_cmd_out.Data(:);
        t_curr = out.Current_Low.Time * 1000;
        plot(t_curr, I_lv, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_curr, I_ref, 'r--', 'LineWidth', 2);
        xlabel('Time (ms)'); ylabel('Current (A)');
        title('Current Tracking (out.Current\_Low vs out.I\_cmd\_out)');
        legend('I_{LV}', 'I_{ref}', 'Location', 'best');
        grid on;
        
        % 100ms 이후 RMS 오차 표시
        valid_idx = out.Current_Low.Time >= 0.1;
        if any(valid_idx)
            error_sig = I_ref(valid_idx) - I_lv(valid_idx);
            rms_err = sqrt(mean(error_sig.^2));
            text(0.02, 0.95, sprintf('RMS Error (>100ms): %.3f A', rms_err), ...
                'Units', 'normalized', 'FontSize', 9, 'BackgroundColor', 'w');
        end
    end
    
    % 2. HV 전압
    subplot(3,3,2);
    if isfield(out, 'Voltage_High') && isa(out.Voltage_High, 'timeseries')
        plot(out.Voltage_High.Time * 1000, out.Voltage_High.Data, 'k-', 'LineWidth', 1.5);
        xlabel('Time (ms)'); ylabel('Voltage (V)');
        title('High-Side Voltage (out.Voltage\_High)');
        grid on;
    end
    
    % 3. LV 전압
    subplot(3,3,3);
    if isfield(out, 'Voltage_Low') && isa(out.Voltage_Low, 'timeseries')
        plot(out.Voltage_Low.Time * 1000, out.Voltage_Low.Data, 'Color', [0 0.5 0], 'LineWidth', 1.5);
        xlabel('Time (ms)'); ylabel('Voltage (V)');
        title('Low-Side Voltage (out.Voltage\_Low)');
        grid on;
    end
    
    % 4. 외부 위상
    subplot(3,3,4);
    if isfield(out, 'Phi_ext_out') && isa(out.Phi_ext_out, 'timeseries')
        plot(out.Phi_ext_out.Time * 1000, out.Phi_ext_out.Data, 'm-', 'LineWidth', 1.5);
        xlabel('Time (ms)'); ylabel('Phase (deg)');
        title('External Phase Shift (out.Phi\_ext\_out)');
        grid on;
    end
    
    % 5. 적분기 상태 (Anti-windup 확인)
    subplot(3,3,5);
    if isfield(out, 'Integrator_out') && isa(out.Integrator_out, 'timeseries')
        plot(out.Integrator_out.Time * 1000, out.Integrator_out.Data, 'g-', 'LineWidth', 1.5);
        hold on;
        yline(3.0, 'r--', 'LineWidth', 1.5);
        yline(-3.0, 'r--', 'LineWidth', 1.5);
        xlabel('Time (ms)'); ylabel('Integrator State');
        title('PI Integrator (out.Integrator\_out) - Clamp ±3.0');
        legend('Integrator', 'Clamp Limit', 'Location', 'best');
        grid on;
    end
    
    % 6. 모드 상태
    subplot(3,3,6);
    if isfield(out, 'Status_flag') && isa(out.Status_flag, 'timeseries')
        stairs(out.Status_flag.Time * 1000, out.Status_flag.Data, 'r-', 'LineWidth', 2);
        xlabel('Time (ms)'); ylabel('Mode');
        title('Operating Mode (out.Status\_flag)');
        ylim([-0.5 1.5]);
        yticks([0 1]);
        yticklabels({'ESS (0)', 'UPS (1)'});
        grid on;
    end
    
    % 7. Duty Ratio D1
    subplot(3,3,7);
    if isfield(out, 'D1_out') && isa(out.D1_out, 'timeseries')
        plot(out.D1_out.Time * 1000, out.D1_out.Data, 'c-', 'LineWidth', 1.5);
        xlabel('Time (ms)'); ylabel('D1');
        title('Primary Duty (out.D1\_out)');
        grid on;
    end
    
    % 8. Duty Ratio D2
    subplot(3,3,8);
    if isfield(out, 'D2_out') && isa(out.D2_out, 'timeseries')
        plot(out.D2_out.Time * 1000, out.D2_out.Data, 'Color', [1 0.5 0], 'LineWidth', 1.5);
        xlabel('Time (ms)'); ylabel('D2');
        title('Secondary Duty (out.D2\_out)');
        grid on;
    end
    
    % 9. 추적 오차
    subplot(3,3,9);
    if isfield(out, 'Current_Low') && isfield(out, 'I_cmd_out')
        I_lv = out.Current_Low.Data(:);
        I_ref = out.I_cmd_out.Data(:);
        t_curr = out.Current_Low.Time * 1000;
        error_sig = I_ref - I_lv;
        plot(t_curr, error_sig, 'b-', 'LineWidth', 1);
        xlabel('Time (ms)'); ylabel('Error (A)');
        title('Tracking Error (I_{ref} - I_{LV})');
        
        % 100ms 무시 구간 표시
        xline(100, 'r--', 'LineWidth', 2);
        text(110, max(error_sig)*0.9, '← 무시 구간', 'FontSize', 9, 'Color', 'r');
        grid on;
    end
    
    sgtitle(sprintf('GA Optimized Controller Verification (Kp=%.4f, Ki=%.2f, Kpv=%.2f, Kiv=%.2f)', ...
        Kp_ext, Ki_ext, Kpv, Kiv), 'FontSize', 14);
    
    fprintf('\n✓ 검증 완료! Figure에서 결과를 확인하세요.\n');
    fprintf('  - 빨간 점선은 100ms 무시 구간을 나타냅니다.\n');
    fprintf('  - 적분기가 ±3.0 한계에서 Clamping 되는지 확인하세요.\n');
end
