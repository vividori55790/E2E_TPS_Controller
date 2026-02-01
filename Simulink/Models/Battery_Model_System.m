classdef Battery_Model_System < matlab.System
    % Battery_Model_System
    % 리튬이온 배터리 1차 Thevenin 등가 회로 모델 (MIL/Simulink용)
    %
    % [기능]
    % 1. SoC (State of Charge) 실시간 계산 (Coulomb Counting)
    % 2. OCV (Open Circuit Voltage) 비선형 곡선 반영 (SoC 기반)
    % 3. 내부 저항(R0) 및 분극 임피던스(R1, C1)에 의한 동적 전압 모의
    %
    % 입력: I_load (부하 전류, [A], +:방전, -:충전)
    % 출력: V_term (단자 전압, [V]), SoC (0~1)
    
    properties(Nontunable)
        % 배터리 팩 사양 (96V 시스템 기준)
        Capacity_Ah = 50;       % 용량 (Ah)
        V_nom = 96;             % 공칭 전압 (V)
        Initial_SoC = 0.5;      % 초기 SoC (0~1)
        Ts_sample = 1e-5;       % 샘플링 타임 (s)
        
        % 등가 회로 파라미터 (단순화된 고정값 사용, 필요 시 룩업테이블로 확장이 용이)
        R0 = 0.05;              % 직렬 내부 저항 (Ohm)
        R1 = 0.02;              % 분극 저항 (Ohm)
        C1 = 1000;              % 분극 커패시턴스 (F) - 시정수 tau = R1*C1
    end
    
    properties(DiscreteState)
        SoC_State;              % 현재 SoC 상태
        V_C1_State;             % 분극 전압 (RC 회로 전압)
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            % 초기화
            obj.SoC_State = obj.Initial_SoC;
            obj.V_C1_State = 0;
        end
        
        function [V_term, SoC_out] = stepImpl(obj, I_load)
            % 1. SoC 계산 (Coulomb Counting)
            % I_load: 양수(+)가 방전, 음수(-)가 충전
            
            % [Solver 안정화] 초기 0.001초 동안은 전압을 96V로 고정하여
            % Simscape Solver가 초기 수렴에 실패하는 것을 방지함.
            if getCurrentTime(obj) < 0.001
                V_term = obj.V_nom; 
                SoC_out = obj.Initial_SoC;
                % 상태변수 유지
                obj.SoC_State = obj.Initial_SoC;
                obj.V_C1_State = 0;
                return;
            end
            
            % SoC(t) = SoC(t-1) - (I * dt) / (Capacity * 3600)
            dSoC = (I_load * obj.Ts_sample) / (obj.Capacity_Ah * 3600);
            
            % SoC 업데이트 및 포화 (0~100%)
            current_SoC = obj.SoC_State - dSoC;
            current_SoC = max(0, min(1, current_SoC));
            obj.SoC_State = current_SoC;
            
            % 2. OCV (Open Circuit Voltage) 계산
            % 리튬이온 배터리 특성 곡선 (Lookup Table 근사)
            % 0% -> 3.0V/cell, 100% -> 4.2V/cell (24직렬 가정: 72V ~ 100.8V)
            % 간단한 1차 또는 3차 룩업
            OCV = lookupOCV(obj, current_SoC);
            
            % 3. 분극 전압(V_C1) 동적 계산 (Discrete 1st Order LPF 형태)
            % dV_c1/dt = (I/C1) - (V_c1 / (R1*C1))
            %离散化: V_c1(k) = V_c1(k-1)*exp(-dt/tau) + I*R1*(1 - exp(-dt/tau))
            tau = obj.R1 * obj.C1;
            alpha = exp(-obj.Ts_sample / tau);
            
            % R1 양단에는 I_load가 흐른다고 가정 (단순화)
            % V_C1 업데이트
            v_c1_new = obj.V_C1_State * alpha + (I_load * obj.R1) * (1 - alpha);
            obj.V_C1_State = v_c1_new;
            
            % 4. 최종 단자 전압(Terminal Voltage) 계산
            % V_term = OCV - V_R0 - V_C1
            % 방전(I>0) 시 전압 강하, 충전(I<0) 시 전압 상승
            V_drop_R0 = I_load * obj.R0;
            
            V_term = OCV - V_drop_R0 - obj.V_C1_State;
            
            % 출력 할당
            SoC_out = current_SoC;
        end
        
        function ocv_val = lookupOCV(obj, soc)
            % OCV-SoC 커브 (예시: NMC 배터리 특성)
            % SoC Points: 0, 0.1, ..., 0.9, 1.0
            soc_pts = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
            
            % Cell Voltage Points (V)
            cell_v_pts = [3.0, 3.3, 3.45, 3.55, 3.62, 3.68, 3.75, 3.85, 3.95, 4.1, 4.2];
            
            % Pack Voltage (24S 가정 -> Nom 96V)
            % 96V / 3.7V(Nom) ~= 26 series? 
            % 여기서는 V_nom(96)에 맞춰 스케일링
            scale_factor = obj.V_nom / 3.7; 
            pack_pts = cell_v_pts * scale_factor;
            
            % 선형 보간 (Linear Interpolation)
            ocv_val = interp1(soc_pts, pack_pts, soc, 'linear', 'extrap');
        end

        function resetImpl(obj)
            obj.SoC_State = obj.Initial_SoC;
            obj.V_C1_State = 0;
        end
        
        function [sz1, sz2] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
        end
        
        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
        end
        
        function [cp1, cp2] = isOutputComplexImpl(~)
            cp1 = false;
            cp2 = false;
        end
        
        function [fs1, fs2] = isOutputFixedSizeImpl(~)
            fs1 = true;
            fs2 = true;
        end
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, name)
            switch name
                case 'SoC_State'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'V_C1_State'
                    sz = [1 1]; dt = 'double'; cp = false;
                otherwise
                    error(['Unknown state: ', name]);
            end
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', ...
                                   'SampleTime', obj.Ts_sample, ...
                                   'OffsetTime', 0);
        end
    end
end
