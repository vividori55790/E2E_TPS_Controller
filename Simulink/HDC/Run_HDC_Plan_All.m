 % =========================================================================
% Run_HDC_Plan_All.m
% DAB 컨버터 TPS 효율 최적화 — 전체 3 Phase 통합 실행 스크립트
% =========================================================================
%
% HDC_Plan.md에 정의된 3단계 로드맵을 순차적으로 실행합니다:
%
%   Phase 1: [Offline] 최적 효율 LUT 재생성 (The Golden Map)
%            - PSO + fmincon으로 Irms 최소화
%            - ZVS 마진 확보
%
%   Phase 2: [Runtime] FF + PI 제어기 검증
%            - LUT Feedforward + PI Feedback 결합
%            - 데드타임 보상
%            - 노후화 시나리오 시뮬레이션
%
%   Phase 3: [Advanced] 온라인 파라미터 추정 (RLS)
%            - 실시간 인덕턴스 L 추정
%            - 4단계 노후화 추적 성능 검증
%
% 사용법:
%   >> Run_HDC_Plan_All           % 전체 실행 (빠른 모드)
%   >> Run_HDC_Plan_All(2)        % Phase 2만 실행
%   >> Run_HDC_Plan_All([1 3])    % Phase 1, 3 실행
%   >> Run_HDC_Plan_All(0)        % 전체 실행 (고해상도)
%
% =========================================================================

function Run_HDC_Plan_All(phases)

if nargin < 1
    phases = [1, 2, 3];  % 전체 실행
end

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════════╗\n');
fprintf('║     DAB Converter TPS Control: Efficiency Optimization          ║\n');
fprintf('║     3-Phase Roadmap Execution                                   ║\n');
fprintf('║                                                                 ║\n');
fprintf('║     Phase 1: Optimal LUT Generation (The Golden Map)            ║\n');
fprintf('║     Phase 2: FF + PI Controller Deployment                      ║\n');
fprintf('║     Phase 3: Online Parameter Estimation (RLS)                  ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════════╝\n\n');

% HDC 디렉토리로 이동 (상대 경로 호환)
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
old_dir = pwd;

% HDC 폴더에서 상위 폴더가 필요한 경우 path 추가
addpath(fullfile(script_dir, '..'));       % Simulink 폴더 (DAB_System_Config 등)
addpath(fullfile(script_dir, '..', 'System'));  % System 폴더
addpath(script_dir);                       % 현재 HDC 폴더

% HDC Config 로드
cfg = HDC_Config();

% 병렬 컴퓨팅 풀 시작 (Phase 1 가속화)
if isempty(gcp('nocreate'))
    try
        parpool; 
        fprintf('[PARALLEL] Pool started with %d workers.\n', gcp().NumWorkers);
    catch
        fprintf('[PARALLEL] Failed to start pool. Running in serial mode.\n');
    end
else
    fprintf('[PARALLEL] Pool already active (%d workers).\n', gcp().NumWorkers);
end

% 결과 저장용
all_results = struct();
total_start = tic;

%% ======================== PHASE 1 ========================
if any(phases == 1) || any(phases == 0)
    fprintf('\n');
    fprintf('┌─────────────────────────────────────────────┐\n');
    fprintf('│  PHASE 1: Optimal Efficiency LUT Generation │\n');
    fprintf('└─────────────────────────────────────────────┘\n\n');
    
    if any(phases == 0)
        % 고해상도 모드 (Config * 1.5배)
        grid_size = round(cfg.opt.GridSize * 1.5);
        pso_swarm = round(cfg.opt.PSO_SwarmSize * 1.5);
        pso_iter  = round(cfg.opt.PSO_MaxIter * 1.5);
        fprintf('[MODE] High-Resolution (Grid=%d)\n', grid_size);
    else
        % 기본 설정 (Config 따름)
        grid_size = cfg.opt.GridSize;
        pso_swarm = cfg.opt.PSO_SwarmSize;
        pso_iter  = cfg.opt.PSO_MaxIter;
        fprintf('[MODE] Standard Config (Grid=%d)\n', grid_size);
    end
    
    try
        all_results.phase1 = Phase1_Generate_Optimal_LUT(...
            'GridSize', grid_size, ...
            'PSO_SwarmSize', pso_swarm, ...
            'PSO_MaxIter', pso_iter, ...
            'ZVS_Margin', cfg.opt.ZVS_Margin, ...
            'P_Range', cfg.opt.P_Range, ...
            'L_val', cfg.sys.L, ...
            'SavePath', cfg.path.LUT_File);
        
        fprintf('\n[PHASE 1] ✅ SUCCESS\n');
    catch ME
        fprintf('\n[PHASE 1] ❌ ERROR: %s\n', ME.message);
        fprintf('  Line: %d in %s\n', ME.stack(1).line, ME.stack(1).name);
        all_results.phase1 = struct('error', ME.message);
    end
end

%% ======================== PHASE 2 ========================
if any(phases == 2) || any(phases == 0)
    fprintf('\n');
    fprintf('┌─────────────────────────────────────────────┐\n');
    fprintf('│  PHASE 2: FF + PI Controller Deployment     │\n');
    fprintf('└─────────────────────────────────────────────┘\n\n');
    
    try
        all_results.phase2 = Phase2_Deploy_FF_PI_Controller(...
            'LUT_Path', cfg.path.LUT_File, ...
            'Kp_phi', cfg.ctrl.Kp_phi, ...
            'Ki_phi', cfg.ctrl.Ki_phi, ...
            'DeadTime_ns', cfg.ctrl.DeadTime_ns, ...
            'SimTime', cfg.sim.phase2_time, ...
            'SavePath', cfg.path.Ctrl_File);
        
        fprintf('\n[PHASE 2] ✅ SUCCESS\n');
    catch ME
        fprintf('\n[PHASE 2] ❌ ERROR: %s\n', ME.message);
        fprintf('  Line: %d in %s\n', ME.stack(1).line, ME.stack(1).name);
        all_results.phase2 = struct('error', ME.message);
    end
end

%% ======================== PHASE 3 ========================
if any(phases == 3) || any(phases == 0)
    fprintf('\n');
    fprintf('┌─────────────────────────────────────────────┐\n');
    fprintf('│  PHASE 3: Online Parameter Estimation (RLS) │\n');
    fprintf('└─────────────────────────────────────────────┘\n\n');
    
    try
        all_results.phase3 = Phase3_Online_Parameter_Estimator(...
            'SimTime', cfg.sim.phase3_time, ...
            'EstimateInterval', cfg.est.Interval, ...
            'RLS_ForgettingFactor', cfg.est.RLS_Lambda, ...
            'EMA_Alpha', cfg.est.EMA_Alpha, ...
            'L_Nominal', cfg.sys.L, ...
            'SavePath', cfg.path.Est_File);
        
        fprintf('\n[PHASE 3] ✅ SUCCESS\n');
    catch ME
        fprintf('\n[PHASE 3] ❌ ERROR: %s\n', ME.message);
        fprintf('  Line: %d in %s\n', ME.stack(1).line, ME.stack(1).name);
        all_results.phase3 = struct('error', ME.message);
    end
end

%% ======================== 최종 SUMMARY ========================
total_elapsed = toc(total_start);

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════════╗\n');
fprintf('║                    EXECUTION SUMMARY                            ║\n');
fprintf('╠═══════════════════════════════════════════════════════════════════╣\n');
fprintf('║  Total Elapsed Time: %8.1f seconds (%.1f min)              ║\n', ...
    total_elapsed, total_elapsed/60);
fprintf('║                                                                 ║\n');

if isfield(all_results, 'phase1')
    if isfield(all_results.phase1, 'error')
        fprintf('║  Phase 1: ❌ FAILED — %s\n', all_results.phase1.error);
    else
        fprintf('║  Phase 1: ✅ LUT Generated (%d points, ZVS=%.0f%%)       ║\n', ...
            all_results.phase1.stats.total_points, ...
            all_results.phase1.stats.zvs_success / all_results.phase1.stats.total_points * 100);
    end
end

if isfield(all_results, 'phase2')
    if isfield(all_results.phase2, 'error')
        fprintf('║  Phase 2: ❌ FAILED — %s\n', all_results.phase2.error);
    else
        fprintf('║  Phase 2: ✅ Controller Deployed & Verified              ║\n');
    end
end

if isfield(all_results, 'phase3')
    if isfield(all_results.phase3, 'error')
        fprintf('║  Phase 3: ❌ FAILED — %s\n', all_results.phase3.error);
    else
        fprintf('║  Phase 3: ✅ RLS Estimator OK (err=%.2f%%)               ║\n', ...
            all_results.phase3.stats.ss_error_rls_pct);
    end
end

fprintf('║                                                                 ║\n');
fprintf('║  Output Files:                                                  ║\n');
fprintf('║    HDC/Phase1_LUT_Results.mat      — 3D Optimal LUT            ║\n');
fprintf('║    HDC/Phase2_Controller_Data.mat   — Simulink-ready ctrl data  ║\n');
fprintf('║    HDC/Phase3_Estimator_Results.mat — RLS estimation results    ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════════╝\n\n');

% 워크스페이스에 결과 내보내기
assignin('base', 'HDC_Plan_Results', all_results);

if isfield(all_results, 'phase2') && ~isfield(all_results.phase2, 'error')
    assignin('base', 'lut_data', all_results.phase2);
    fprintf('[INFO] Simulink Ready Variable: lut_data (Assign this to ''HDC_Data'' parameter)\n');
end
fprintf('[INFO] Results saved to workspace variable: HDC_Plan_Results\n');

end
