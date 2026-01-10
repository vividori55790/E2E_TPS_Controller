% =========================================================
% [Universal] DAB 컨버터 최적 위상 데이터 생성기 (p.u. 정규화)
% 적용 대상: ESS(충방전), UPS(전압보상), 계통연계(Grid)
% 특징: 변압기 권선비(n) 고려, 양방향 전력 흐름 지원
% =========================================================

clear; clc;

% --- 1. 시스템 하드웨어 스펙 설정 ---
n = 4.0;          % 변압기 권선비 (Primary : Secondary = n : 1)
                  % 예: 1차측(배터리) 200V, 2차측(DC링크) 100V라면 n=2가 유리

L = 50e-6;        % 1차측 환산 누설 인덕턴스 (H)
fs = 100e3;       % 스위칭 주파수 (100kHz)

% --- 2. 정격(Base) 값 설정 (p.u. 기준) ---
V1_nom = 200;     % 1차측 정격 전압 (V) - 예: 배터리 정격
V2_nom = 100;     % 2차측 정격 전압 (V) - 예: DC 링크 정격
P_rated = 2000;   % 정격 전력 (W)

V_base = V1_nom;  % p.u. 기준 전압
P_base = P_rated; % p.u. 기준 전력

% --- 3. 동작 범위 설정 (시나리오 스윕) ---
% ESS/UPS 특성상 배터리 전압(V1)은 변동이 심함 (예: SOC에 따른 변화)
V1_range = 150:10:250;  % 1차측 전압 가변 범위 (V)
V2_fixed = 100;         % 2차측 전압 (V) - 보통 DC 링크는 일정하게 제어됨

% 전력 스윕: 충전(-P) <-> 대기(0) <-> 방전(+P)
% ESS는 양방향, UPS는 주로 방전(+), 계통은 양방향
P_range = -2000:200:2000; 

data_pu = []; 

fprintf('데이터 생성 시작... 권선비 n=%.1f 적용\n', n);

options = optimoptions('fmincon','Display','off', 'Algorithm', 'sqp');

for V1 = V1_range
    for P_ref = P_range
        
        % 0전력(대기)은 최적화 불필요
        if abs(P_ref) < 1e-3
            new_row = [V1/V_base, V2_fixed/V_base, 0, 0, 0, 0];
            data_pu = [data_pu; new_row];
            continue;
        end

        % --- 4. 최적화 실행 ---
        % x = [D1(Global), D2(Primary Inner), D3(Secondary Inner)]
        x0 = [0.01, 0.0, 0.0]; 
        
        % 목적함수: RMS 전류 최소화 (효율 극대화)
        obj_fun = @(x) cost_function_current(x);
        
        % 제약조건: 전력 전달량 맞추기 (권선비 n 포함)
        nonl_con = @(x) power_constraint_universal(x, V1, V2_fixed, L, fs, n, P_ref);
        
        % 위상각 제한 (-0.5 ~ 0.5 duty cycle)
        lb = [-0.5, -0.5, -0.5];
        ub = [ 0.5,  0.5,  0.5];
        
        try
            [x_opt, fval, exitflag] = fmincon(obj_fun, x0, [],[],[],[], lb, ub, nonl_con, options);
            
            if exitflag > 0
                % --- 5. p.u. 정규화 및 저장 ---
                % 전압 변환 비율 (Voltage Conversion Ratio) M = V2' / V1 = (n*V2) / V1
                % 이 지표가 DAB 제어 특성을 결정하는 핵심임
                
                V1_pu = V1 / V_base;
                V2_pu = (n * V2_fixed) / V_base; % 1차측으로 환산된 2차측 전압 기준
                P_pu  = P_ref / P_base;
                
                % [V1(pu), V2_referred(pu), P(pu), D1, D2, D3]
                new_row = [V1_pu, V2_pu, P_pu, x_opt(1), x_opt(2), x_opt(3)];
                data_pu = [data_pu; new_row];
            end
        catch
            % 해를 못 찾는 영역 (전력 한계 초과 등)
        end
    end
end

filename = 'dab_universal_data.csv';
csvwrite(filename, data_pu);
fprintf('완료! %s 파일 생성됨. (데이터 %d개)\n', filename, size(data_pu, 1));


% =========================================================
% [보조 함수] 목적 함수: 전류 스트레스(RMS) 대리 모델
% =========================================================
function cost = cost_function_current(x)
    % 단순화된 모델: 위상차의 절댓값 합이 작을수록 무효전력(손실)이 작음
    % 실제 연구용으로는 "TPS RMS Current Formula"를 적용해야 함
    D1=x(1); D2=x(2); D3=x(3);
    cost = D1^2 + D2^2 + D3^2;
end

% =========================================================
% [보조 함수] 제약 조건: 일반화된 DAB 전력 전송 공식
% =========================================================
function [c, ceq] = power_constraint_universal(x, V1, V2, L, fs, n, P_ref)
    c = [];
    
    % TPS 전력 전달 공식 (기본형 근사)
    % P = (V1 * (n*V2)) / (2*fs*L) * Phi * (1 - |Phi|)
    % 여기서 Phi는 등가적인 총 위상차
    % 주의: TPS에서는 D1, D2, D3가 복합적으로 작용함.
    % 아래 식은 D1(Global Shift)이 지배적이고 D2, D3가 보조한다고 가정한 모델임.
    
    % V2_prime = n * V2 (2차측 전압을 1차측으로 환산)
    V2_prime = n * V2;
    
    D_global = x(1);
    
    % 전력 수식 상수 K
    K = (V1 * V2_prime) / (2 * fs * L);
    
    % 전력 계산 (방향성 포함: D_global이 음수면 전력도 음수)
    P_calc = K * D_global * (1 - abs(D_global));
    
    % 오차 (이 값이 0이 되어야 함)
    ceq = P_calc - P_ref;
end