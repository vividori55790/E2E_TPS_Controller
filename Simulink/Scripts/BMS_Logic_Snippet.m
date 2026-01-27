% =========================================================================
% [Code Snippet] Phase 2: Smart BMS Logic (CC-CV & Reserve)
% 
% 아래의 코드를 DAB_Controller_System.m의 'stepImpl' 메서드 내부에
% '--- 4. 모드별 기준 전류/피드백 선택 ---' 섹션 직전에 삽입하세요.
% =========================================================================

% --- [New] 3.5. Smart BMS Logic (CC-CV & Reserve Strategy) ---

% (1) CV 모드 전압 제한 (109.2V)
% 충전 중(I_ess_cmd < 0)이고, 배터리 전압이 설정값(V_Max_Limit) 이상이면 CV 제어 개입
V_Max_Limit = 109.2; % Li-ion 96V Pack Max Voltage
I_Cutoff = 0.5;      % 충전 종료 전류 (A)
Kp_cv = 1.0;         % CV 제어 비례 이득 (임의 설정, 필요시 튜닝)

I_bms_cmd = I_ess_cmd; % 기본값은 사용자 지령 그대로 통과

if I_ess_cmd < 0 % 충전 지령일 때만 검사
    if V_LV >= V_Max_Limit
        % CV 제어: 전압 오차에 비례하여 충전 전류 제한 (0에 가깝게)
        % err_v가 양수(아직 여유 있음)면 I_limit는 음수(충전 계속)
        % err_v가 0 또는 음수(과전압)면 I_limit는 0 또는 양수(충전 중단/방전)
        err_v_bat = V_Max_Limit - V_LV;
        I_cv_limit = Kp_cv * err_v_bat; 
        
        % 원래 지령과 CV 제한값 중 '덜 충전하는(0에 가까운)' 값 선택
        % 둘 다 음수일 때 max()를 취하면 0에 더 가까운 놈이 선택됨
        I_bms_cmd = max(I_ess_cmd, I_cv_limit);
        
        % (2) Cut-off: 전류가 너무 작으면 아예 0으로 (Floating)
        if abs(I_bms_cmd) < I_Cutoff
            I_bms_cmd = 0;
        end
    end
end

% (2) User-Defined Reserve Strategy (Software Defined Power)
% 사용자가 설정한 비율(Reserve_Ratio)만큼 에너지를 남겨둠
% Reserve_Ratio: 0~1 (예: 0.2 = 20%) -> Double Click Block Parameter로 추가 예정

% 전압 기반 SOC 추정 (96V Pack: 100%~109.2V, 0%~75V 가정)
% 단순 선형 근사: V_threshold = V_min + (V_max - V_min) * Ratio
V_Cell_Min = 75.0;  % 방전 종지 전압
V_Cell_Max = 109.2; % 만충 전압

% [User Property] obj.Reserve_Ratio (0.0 ~ 1.0)
% 코드 적용 시 properties(Nontunable) 섹션에 'Reserve_Ratio = 0.2;' 추가 필요
SOC_Threshold_ESS = V_Cell_Min + (V_Cell_Max - V_Cell_Min) * obj.Reserve_Ratio;
SOC_Threshold_UPS = 80.0; % 절대 하한선 (5% - 배터리 보호용 마지노선)

if I_bms_cmd > 0 % 방전 지령일 때만 검사
    if obj.PowerLossState == 0 % ESS Mode (정상)
        if V_LV < SOC_Threshold_ESS
            % 사용자가 정의한 Reserve 구간 도달 -> 방전 중단
            I_bms_cmd = 0; 
        end
    else % UPS Mode (비상)
        if V_LV < SOC_Threshold_UPS
            % 비상시라도 배터리 파손 방지를 위해 최소한의 보호 수행
            I_bms_cmd = 0; 
        end
    end
end

% 최종적으로 BMS가 수정한 지령을 사용하도록 업데이트
I_ess_cmd = I_bms_cmd; 

% -------------------------------------------------------------------------
