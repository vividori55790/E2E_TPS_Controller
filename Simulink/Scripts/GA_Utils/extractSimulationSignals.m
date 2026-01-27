function [t, i_out, phi_out, v_high_out, i_ind_out] = extractSimulationSignals(simOut)
% extractSimulationSignals: Simulink 시뮬레이션 결과에서 주요 신호 추출
%
% [Inputs]
%   simOut: Simulink 시뮬레이션 결과 객체 (Simulink.SimulationOutput)
%
% [Outputs]
%   t: 시간 벡터
%   i_out: 출력 전류 (Current_Low, I_LV 등)
%   phi_out: 위상 제어 입력 (Phi_ext_out)
%   v_high_out: 고전압측 전압 (Voltage_High)
%   i_ind_out: 인덕터 전류 (Inductor_Current)

    % 1. 전류 추출
    i_names = {'Current_Low', 'I_LV', 'I_out', 'I_ess'};
    i_out = [];
    t = [];
    
    for k = 1:length(i_names)
        if isprop(simOut, i_names{k})
            raw = simOut.(i_names{k});
            i_out = getSignalData(raw);
            t = getSignalTime(raw, simOut);
            break;
        end
    end
    
    if isempty(i_out) && isprop(simOut, 'yout')
        try 
            i_out = getSignalData(simOut.yout.get('Current_Low')); 
            t = getSignalTime(simOut.yout.get('Current_Low'), simOut);
        catch
        end
    end

    % 2. 위상 추출 (Phi_ext_out)
    p_names = {'Phi_ext_out', 'phi_ext_out', 'Phi_ext', 'phi'};
    phi_out = [];
    for k = 1:length(p_names)
        if isprop(simOut, p_names{k})
            phi_out = getSignalData(simOut.(p_names{k}));
            break;
        end
    end
    
    % 3. Voltage High 추출
    v_high_out = [];
    if isprop(simOut, 'Voltage_High')
        v_high_out = getSignalData(simOut.Voltage_High);
    elseif isprop(simOut, 'V_High')
        v_high_out = getSignalData(simOut.V_High);
    end
    
    % 4. Inductor Current 추출
    i_ind_out = [];
    if isprop(simOut, 'Inductor_Current')
        i_ind_out = getSignalData(simOut.Inductor_Current);
    elseif isprop(simOut, 'I_L')
        i_ind_out = getSignalData(simOut.I_L);
    end
    
    % Data Check & Padding
    if isempty(i_out)
        % 데이터가 없을 경우 에러 처리 대신 빈 배열 반환하여 상위 함수에서 처리 유도
        warning('No simulation data found (Current_Low).');
        return;
    end
    
    n_t = length(t);
    if isempty(phi_out), phi_out = zeros(n_t, 1); end
    if isempty(v_high_out), v_high_out = zeros(n_t, 1); end
    if isempty(i_ind_out), i_ind_out = zeros(n_t, 1); end
    
    i_out = i_out(:); 
    phi_out = phi_out(:); 
    v_high_out = v_high_out(:); 
    i_ind_out = i_ind_out(:);
    t = t(:);
end

function data = getSignalData(raw)
    if isa(raw, 'timeseries')
        data = squeeze(raw.Data);
    elseif isstruct(raw)
        data = squeeze(raw.signals.values);
    else
        data = squeeze(raw);
    end
end

function t = getSignalTime(raw, simOut)
    if isa(raw, 'timeseries')
        t = raw.Time;
    elseif isstruct(raw)
        t = raw.time;
    else
        t = simOut.tout;
    end
end
