classdef DAB_PWM_Generator < matlab.System
    % DAB_PWM_Generator
    % DAB 컨버터 PWM 신호 생성기
    %
    % 기능:
    %   - 위상 입력(음수/양수 모두 허용)을 받아 PWM 신호 생성
    %   - 외부 포트로 데드타임 설정 가능
    %   - 8개의 게이트 신호 출력 (Primary 4개 + Secondary 4개)
    %
    % 입력:
    %   - phi1_deg: Primary Bridge Leg A 기준 위상 (S1, 항상 0도)
    %   - phi3_deg: Primary Bridge Leg B 위상 (S3, SPS: 180도)
    %   - phi5_deg: Secondary Bridge Leg A 위상 (S5, 음수 허용: -90~+90)
    %   - phi7_deg: Secondary Bridge Leg B 위상 (S7, phi5+180)
    %   - DeadTime_ns: 데드타임 (나노초, 예: 200ns)
    %
    % 출력:
    %   - G1, G2: Primary Leg A 상/하단 게이트 신호
    %   - G3, G4: Primary Leg B 상/하단 게이트 신호
    %   - G5, G6: Secondary Leg A 상/하단 게이트 신호
    %   - G7, G8: Secondary Leg B 상/하단 게이트 신호
    %
    % PWM 동작:
    %   - 각 레그는 50% 듀티 사이클
    %   - 상단 스위치: 위상 기준 0~180도에서 ON (데드타임 적용)
    %   - 하단 스위치: 위상 기준 180~360도에서 ON (데드타임 적용)
    %   - 데드타임: 상/하단 동시 OFF 구간
    %
    % 음수 위상 처리:
    %   - 입력된 음수 위상은 내부에서 0~360도로 정규화
    %   - 예: -30도 → 330도로 변환하여 PWM 생성

    % =========================================================================
    % 1. Public Properties
    % =========================================================================
    properties(Nontunable)
        Fs = 80e3;              % 스위칭 주파수 (Hz) - Controller와 동일해야 함
        Ts_sim = 1e-7;          % 시뮬레이션 샘플 시간 (초) - Simulink 솔버와 동일
                                % 80kHz 기준: 주기당 125 샘플 (12.5µs / 100ns)
        
        % 게이트 드라이버 출력 전압 레벨
        V_Gate_ON = 18;         % ON 상태 전압 (V)
        V_Gate_OFF = -4;        % OFF 상태 전압 (V)
    end

    % =========================================================================
    % 2. Internal States
    % =========================================================================
    properties(DiscreteState)
        PhaseCounter;           % 위상 카운터 (0~360도 범위)
    end

    % =========================================================================
    % 3. Methods
    % =========================================================================
    methods(Access = protected)
        
        function setupImpl(obj)
            % 초기화 (시뮬레이션 시작 시 1회 실행)
            obj.PhaseCounter = 0;
        end

        function [G1, G2, G3, G4, G5, G6, G7, G8] = ...
                stepImpl(obj, phi1_deg, phi3_deg, phi5_deg, phi7_deg, DeadTime_ns)
            
            % --- 1. 파라미터 계산 ---
            Ts_sw = 1 / obj.Fs;                  % 스위칭 주기 (초)
            
            % 데드타임을 도(degree)로 변환
            DeadTime_sec = DeadTime_ns * 1e-9;   % 나노초 → 초
            DeadTime_deg = (DeadTime_sec / Ts_sw) * 360;  % 초 → 도
            
            % --- 2. 위상 카운터 업데이트 ---
            % 샘플 시간 동안 진행되는 위상 (도)
            phase_increment = (obj.Ts_sim / Ts_sw) * 360;
            
            obj.PhaseCounter = mod(obj.PhaseCounter + phase_increment, 360);
            current_phase = obj.PhaseCounter;
            
            % --- 3. 입력 위상 정규화 (음수 → 0~360도) ---
            % 음수 위상을 0~360 범위로 변환
            phi1_norm = mod(phi1_deg, 360);
            if phi1_norm < 0, phi1_norm = phi1_norm + 360; end
            
            phi3_norm = mod(phi3_deg, 360);
            if phi3_norm < 0, phi3_norm = phi3_norm + 360; end
            
            phi5_norm = mod(phi5_deg, 360);
            if phi5_norm < 0, phi5_norm = phi5_norm + 360; end
            
            phi7_norm = mod(phi7_deg, 360);
            if phi7_norm < 0, phi7_norm = phi7_norm + 360; end
            
            % --- 4. 각 스위치별 상대 위상 계산 ---
            % 현재 위상에서 각 스위치 기준 위상을 뺀 상대 위상
            phase_S1 = mod(current_phase - phi1_norm + 360, 360);
            phase_S3 = mod(current_phase - phi3_norm + 360, 360);
            phase_S5 = mod(current_phase - phi5_norm + 360, 360);
            phase_S7 = mod(current_phase - phi7_norm + 360, 360);
            
            % --- 5. PWM 신호 생성 (50% 듀티, 데드타임 적용) ---
            % 상단 스위치 (G1, G3, G5, G7): DT ~ (180-DT) 도에서 ON
            % 하단 스위치 (G2, G4, G6, G8): (180+DT) ~ (360-DT) 도에서 ON
            % 
            % 데드타임 구간:
            %   - 0 ~ DT: 둘 다 OFF
            %   - (180-DT) ~ (180+DT): 둘 다 OFF
            %   - (360-DT) ~ 360: 둘 다 OFF
            
            DT = DeadTime_deg;
            
            % 게이트 전압 레벨
            V_ON = obj.V_Gate_ON;    % 18V
            V_OFF = obj.V_Gate_OFF;  % -4V
            
            % Primary Leg A (S1, S2)
            if (phase_S1 >= DT) && (phase_S1 < (180 - DT))
                G1 = V_ON;
            else
                G1 = V_OFF;
            end
            if (phase_S1 >= (180 + DT)) && (phase_S1 < (360 - DT))
                G2 = V_ON;
            else
                G2 = V_OFF;
            end
            
            % Primary Leg B (S3, S4)
            if (phase_S3 >= DT) && (phase_S3 < (180 - DT))
                G3 = V_ON;
            else
                G3 = V_OFF;
            end
            if (phase_S3 >= (180 + DT)) && (phase_S3 < (360 - DT))
                G4 = V_ON;
            else
                G4 = V_OFF;
            end
            
            % Secondary Leg A (S5, S6)
            if (phase_S5 >= DT) && (phase_S5 < (180 - DT))
                G5 = V_ON;
            else
                G5 = V_OFF;
            end
            if (phase_S5 >= (180 + DT)) && (phase_S5 < (360 - DT))
                G6 = V_ON;
            else
                G6 = V_OFF;
            end
            
            % Secondary Leg B (S7, S8)
            if (phase_S7 >= DT) && (phase_S7 < (180 - DT))
                G7 = V_ON;
            else
                G7 = V_OFF;
            end
            if (phase_S7 >= (180 + DT)) && (phase_S7 < (360 - DT))
                G8 = V_ON;
            else
                G8 = V_OFF;
            end
        end

        function resetImpl(obj)
            % 리셋 신호가 들어오거나 재시작 시
            obj.PhaseCounter = 0;
        end
        
        % --- 출력 개수 지정 ---
        function num = getNumOutputsImpl(~)
            num = 8;
        end
        
        % --- 입력 개수 지정 ---
        function num = getNumInputsImpl(~)
            num = 5;
        end
        
        % --- 출력 크기 지정 ---
        function [sz1,sz2,sz3,sz4,sz5,sz6,sz7,sz8] = getOutputSizeImpl(~)
            sz1 = [1 1]; sz2 = [1 1]; sz3 = [1 1]; sz4 = [1 1];
            sz5 = [1 1]; sz6 = [1 1]; sz7 = [1 1]; sz8 = [1 1];
        end
        
        % --- 출력 데이터 타입 지정 ---
        function [dt1,dt2,dt3,dt4,dt5,dt6,dt7,dt8] = getOutputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double'; dt3 = 'double'; dt4 = 'double';
            dt5 = 'double'; dt6 = 'double'; dt7 = 'double'; dt8 = 'double';
        end
        
        % --- 출력 복소수 여부 ---
        function [cp1,cp2,cp3,cp4,cp5,cp6,cp7,cp8] = isOutputComplexImpl(~)
            cp1 = false; cp2 = false; cp3 = false; cp4 = false;
            cp5 = false; cp6 = false; cp7 = false; cp8 = false;
        end
        
        % --- 고정 크기 출력 ---
        function [fs1,fs2,fs3,fs4,fs5,fs6,fs7,fs8] = isOutputFixedSizeImpl(~)
            fs1 = true; fs2 = true; fs3 = true; fs4 = true;
            fs5 = true; fs6 = true; fs7 = true; fs8 = true;
        end
        
        % --- DiscreteState 사양 지정 (Simulink 필수) ---
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, name)
            % 각 DiscreteState 속성의 크기, 타입, 복소수 여부 지정
            switch name
                case 'PhaseCounter'
                    sz = [1 1]; dt = 'double'; cp = false;
                otherwise
                    error(['Unknown state: ', name]);
            end
        end
    end
end
