classdef DAB_Average_Model < matlab.System
    % DAB_Average_Model
    % DAB 컨버터 평균화 모델 (스위칭 없음, 고속 시뮬레이션)
    %
    % v2.0 개선사항:
    %   - 커패시터 동역학 추가 (전압 변동 시뮬레이션 가능)
    %   - HV/LV 양측 전압이 동적으로 변화
    %   - 배터리 모델 (EMF + 내부저항) 포함
    %
    % 평균화 모델 원리:
    %   - 스위칭 리플이 없는 평균값으로 동작
    %   - 전력 전달: P = (V1 * V2 / (n * omega * L)) * phi * (1 - |phi|/pi)
    %   - 스위칭 모델 대비 시뮬레이션 속도 10~100배 향상
    %
    % 입력: V_source (HV측 전원), phi_ext_deg (외부 위상), phi_int_deg (내부 위상), I_load_HV
    % 출력: V_HV, V_LV, I_HV, I_LV, P_transfer
    %
    % 스위칭 모델과 동치 조건:
    %   - 동일한 DC 전압, 인덕턴스, 변압기 권선비 사용
    %   - 위상각 입력이 동일할 때 평균 전력/전류가 일치
    
    % =========================================================================
    % 1. Plant Parameters
    % =========================================================================
    properties(Nontunable)
        % 변압기 및 인덕턴스
        n = 4.0;                % 변압기 권선비 (N_HV / N_LV)
        L = 50e-6;              % 누설 인덕턴스 (H)
        fs = 80e3;              % 스위칭 주파수 (Hz)
        
        % 효율
        efficiency = 0.95;      % 전체 시스템 효율 (0~1)
        
        % 커패시터 (전압 동역학용)
        C_HV = 1000e-6;         % HV측 커패시터 (F) - DC 링크
        C_LV = 500e-6;          % LV측 커패시터 (F) - 배터리측
        
        % 배터리 모델 파라미터
        V_batt_EMF = 100;       % 배터리 EMF (V) - Open Circuit Voltage
        R_batt = 0.1;           % 배터리 내부 저항 (Ohm)
        
        % 저항 손실 모델링
        R_eq = 0.01;            % 등가 직렬 저항 (Ohm)
        
        % 시뮬레이션 타임스텝 (외부에서 설정)
        Ts_sim = 10e-6;         % 시뮬레이션 타임스텝 (s)
    end
    
    % =========================================================================
    % 2. Internal States (전압 상태변수)
    % =========================================================================
    properties(DiscreteState)
        V_HV_cap;               % HV측 커패시터 전압 (상태변수)
        V_LV_cap;               % LV측 커패시터 전압 (상태변수)
    end
    
    % =========================================================================
    % 3. Methods
    % =========================================================================
    methods(Access = protected)
        
        function setupImpl(obj)
            % 초기화
            obj.V_HV_cap = 380;     % 초기 HV 전압 (정상 상태 가정)
            obj.V_LV_cap = 100;     % 초기 LV 전압 (배터리 EMF)
        end
        
        function [V_HV, V_LV, I_HV, I_LV, P_transfer] = stepImpl(obj, V_source, phi_ext_deg, phi_int_deg, I_load_HV)
            % =================================================================
            % DAB 평균화 모델 - 핵심 전력 전달 + 커패시터 동역학
            % =================================================================
            %
            % 회로 구성:
            %   [V_source] ---(I_source)--- [C_HV, V_HV] ---(DAB)--- [C_LV, V_LV] ---(R_batt)--- [V_batt_EMF]
            %                                    |
            %                               (I_load_HV)
            %
            % 상태 방정식:
            %   dV_HV/dt = (I_source - I_HV - I_load_HV) / C_HV
            %   dV_LV/dt = (I_LV - I_batt) / C_LV
            %   I_batt = (V_LV - V_batt_EMF) / R_batt
            % =================================================================
            
            % 현재 커패시터 전압 (상태변수)
            V_HV_now = obj.V_HV_cap;
            V_LV_now = obj.V_LV_cap;
            
            % ===== 1. 평균 전력 계산 (SPS/DPS) =====
            % 라디안 변환
            phi_ext = phi_ext_deg * pi / 180;
            phi_int = phi_int_deg * pi / 180;
            
            % 각주파수
            omega = 2 * pi * obj.fs;
            
            % LV 전압을 HV측으로 환산
            V_LV_prime = V_LV_now * obj.n;
            
            % 내부 위상 듀티 (0 ~ 0.5)
            D = abs(phi_int) / pi;
            D = min(0.5, D);  % 최대 0.5
            
            % === SPS 기본 공식 + 내부 위상 듀티 영향 ===
            if abs(phi_ext) < 0.001
                % 위상이 거의 0일 때
                P_transfer = 0;
            else
                % P = (V1 * V2') / (omega * L) * phi * (1 - |phi|/(pi/2))
                % 내부 위상이 클수록 순환전류 증가, 유효 전력 감소
                factor = (1 - D) * (1 - abs(phi_ext) / (pi/2));
                factor = max(0, min(1, factor));  % 0~1 범위
                
                P_transfer = (V_HV_now * V_LV_prime / (omega * obj.L)) * phi_ext * factor;
            end
            
            % 효율 적용
            if P_transfer > 0
                % HV → LV (방전): 손실 후 LV에 도달
                P_LV = P_transfer * obj.efficiency;
                P_HV = P_transfer;
            else
                % LV → HV (충전): 손실 후 HV에 도달
                P_LV = P_transfer;
                P_HV = P_transfer * obj.efficiency;
            end
            
            % ===== 2. DAB 전류 계산 =====
            % P = V * I → I = P / V
            if abs(V_HV_now) > 1
                I_HV = P_HV / V_HV_now;  % HV에서 나가는 전류 (방전 시 양수)
            else
                I_HV = 0;
            end
            
            if abs(V_LV_now) > 0.1
                I_LV = -P_LV / V_LV_now;  % LV로 들어오는 전류 (방전 시 음수 = 충전)
            else
                I_LV = 0;
            end
            
            % ===== 3. 배터리 전류 (LV측 부하) =====
            % I_batt = (V_LV - V_EMF) / R_batt
            % 양수: 방전 (LV → 배터리), 음수: 충전 (배터리 → LV)
            I_batt = (V_LV_now - obj.V_batt_EMF) / obj.R_batt;
            
            % ===== 4. 커패시터 전압 동역학 (Forward Euler) =====
            % dV_HV/dt = (I_source - I_HV - I_load_HV) / C_HV
            % I_source: V_source에서 공급되는 전류 (전압원-커패시터 연결)
            % 간단한 모델: V_source와 V_HV 사이의 저항성 연결로 근사
            R_source = 0.1;  % 전원 임피던스 (Ohm)
            I_source = (V_source - V_HV_now) / R_source;
            
            dV_HV = (I_source - I_HV - I_load_HV) / obj.C_HV;
            V_HV_next = V_HV_now + dV_HV * obj.Ts_sim;
            
            % dV_LV/dt = (I_LV - I_batt) / C_LV
            % I_LV: DAB에서 LV로 흐르는 전류 (음수 = 충전)
            % I_batt: 배터리로 흐르는 전류 (음수 = LV에서 빠져나감)
            dV_LV = (-I_LV - I_batt) / obj.C_LV;
            V_LV_next = V_LV_now + dV_LV * obj.Ts_sim;
            
            % 전압 제한 (안전 범위)
            V_HV_next = max(100, min(500, V_HV_next));  % HV: 100~500V
            V_LV_next = max(50, min(150, V_LV_next));   % LV: 50~150V
            
            % ===== 5. 상태 업데이트 =====
            obj.V_HV_cap = V_HV_next;
            obj.V_LV_cap = V_LV_next;
            
            % ===== 6. 출력 =====
            V_HV = V_HV_next;
            V_LV = V_LV_next;
        end
        
        function resetImpl(obj)
            obj.V_HV_cap = 380;
            obj.V_LV_cap = 100;
        end
        
        function num = getNumInputsImpl(~)
            num = 4;  % V_source, phi_ext_deg, phi_int_deg, I_load_HV
        end
        
        function num = getNumOutputsImpl(~)
            num = 5;  % V_HV, V_LV, I_HV, I_LV, P_transfer
        end
        
        function [name1, name2, name3, name4] = getInputNamesImpl(~)
            name1 = 'V_source';
            name2 = 'phi_ext';
            name3 = 'phi_int';
            name4 = 'I_load_HV';
        end
        
        function [name1, name2, name3, name4, name5] = getOutputNamesImpl(~)
            name1 = 'V_HV';
            name2 = 'V_LV';
            name3 = 'I_HV';
            name4 = 'I_LV';
            name5 = 'P_transfer';
        end
        
        function [sz1, sz2, sz3, sz4, sz5] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
            sz3 = [1 1];
            sz4 = [1 1];
            sz5 = [1 1];
        end
        
        function [dt1, dt2, dt3, dt4, dt5] = getOutputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
            dt4 = 'double';
            dt5 = 'double';
        end
        
        function [c1, c2, c3, c4, c5] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
            c3 = false;
            c4 = false;
            c5 = false;
        end
        
        function [f1, f2, f3, f4, f5] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
            f3 = true;
            f4 = true;
            f5 = true;
        end
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, name)
            % DiscreteState 속성의 크기, 데이터 타입, 복잡성 명세
            if strcmp(name, 'V_HV_cap')
                sz = [1 1];
                dt = 'double';
                cp = false;
            elseif strcmp(name, 'V_LV_cap')
                sz = [1 1];
                dt = 'double';
                cp = false;
            else
                error('Unknown discrete state: %s', name);
            end
        end
    end
end
