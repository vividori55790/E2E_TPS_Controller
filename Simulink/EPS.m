function [t1, t4, t5, t8, ...
          mode_active, alpha_side_mode_out, ...
          phi_int_out, phi_ext_out, ...
          SOC_percent] = ...
    EPS( ...
        V_HV, I_HV, ...
        V_LV, I_LV, ...
        I_DAB_out, P_load_threshold)   

%% ================= 기본 파라미터 =================
fs       = 80e3;
Ts       = 1 / fs;
deg2time = Ts / 360;

V_HV_ref  = 380;
V_HV_band = 2;

Kpv = -2.0;
Kiv = -200;

Kp_ext = 4;
Ki_ext = 60;

I_LV_min_limit = -15;      % 배터리 방전 전류 하한
n_transformer  = 380 / 95;

%% Soft-start (ESS용)
softStart_enable = true;
Iref_ramp_rate   = 0.05;   % [A/sample]

%% ESS 조건/제한
P_DAB_max        = 1000;   % [W]

%% 충전 전류 (0.25C @ 4Ah)
battery_capacity_Ah = 4;        % [Ah]
C_rate_charge       = 0.25;     % 0.25C
I_charge_CC         = battery_capacity_Ah * C_rate_charge;   % = 1 A

%% SOC 기준 (ESS 방전 보호용만 사용)
V_LV_min = 85;
V_LV_max = 105;
SOC_min_for_discharge = 20;     % [%]

%% CV 제어 (충전 모드)
V_LV_ref = 105;
Kp_cv    = 0.03;
Ki_cv    = 0.005;

%% ================= persistent =================
persistent power_loss_state integrator_ext integrator_v previous_mode
persistent Iref_soft
persistent integrator_ups integrator_discharge integrator_charge
persistent integrator_cv

if isempty(power_loss_state);      power_loss_state = 0; end
if isempty(integrator_ext);        integrator_ext = 0; end
if isempty(integrator_v);          integrator_v   = 0; end
if isempty(previous_mode);         previous_mode  = -1; end
if isempty(Iref_soft);             Iref_soft      = 0; end
if isempty(integrator_ups);        integrator_ups = 0; end
if isempty(integrator_discharge);  integrator_discharge = 0; end
if isempty(integrator_charge);     integrator_charge    = 0; end
if isempty(integrator_cv);         integrator_cv        = 0; end

%% ================= UPS 판정 =================
if abs(V_HV - V_HV_ref) > V_HV_band
    power_loss_state = 1;   % UPS
else
    power_loss_state = 0;   % ESS
end

%% ================= 모드 전환 초기화 =================
if power_loss_state ~= previous_mode
    integrator_ext       = 0;
    integrator_v         = 0;
    integrator_ups       = 0;
    integrator_discharge = 0;
    integrator_charge    = 0;
    integrator_cv        = 0;
    Iref_soft            = 0;
end

%% ================= 제어 기준 =================
Iref = 0;
I_fb = 0;

%% ================= SOC 계산 (ESS 보호용) =================
V_LV_safe = max(V_LV, 1);
if V_LV_safe >= V_LV_min && V_LV_safe <= V_LV_max
    SOC_percent = 100 * (V_LV_safe - V_LV_min) / (V_LV_max - V_LV_min);
else
    SOC_percent = 0;
end

%% ================= UPS / ESS 분기 =================
if power_loss_state == 1
    %% ---------- UPS 모드 ----------
    mode_active = 1;

    % V_LV 저전압 시 UPS 방전 금지
    if V_LV <= 85
        Iref      = 0;
        Iref_soft = 0;
        I_fb      = I_DAB_out;
        integrator_v   = 0;
        integrator_ups = 0;
        integrator_ext = 0;
    else
        err_v       = V_HV_ref - V_HV;
        integrator_v = integrator_v + err_v / fs;
        Iref        = Kpv * err_v + Kiv * integrator_v;
        I_fb        = I_DAB_out;
        Iref_soft   = Iref;        % UPS는 램프 없이 바로 사용
    end

else
    %% ---------- ESS 모드 ----------
    V_HV_safe   = max(V_HV, 1);
    I_threshold = P_load_threshold / V_HV_safe;
    I_load      = I_HV;

    if I_load > I_threshold
        %% ---- ESS 방전 (mode 2) ----
        mode_active = 2;

        % 부하 부족분을 HV측 전류 기준으로 계산
        I_dab_ref = I_load - I_threshold;

        % SOC 20% 이하 방전 금지 (85~105 V 구간에서만)
        if V_LV_safe >= V_LV_min && V_LV_safe <= V_LV_max ...
                && SOC_percent <= SOC_min_for_discharge
            I_dab_ref = 0;
        end

        % 전력 제한 1 kW
        P_DAB = I_dab_ref * V_HV_safe;
        P_DAB = min(P_DAB, P_DAB_max);
        I_dab_ref = P_DAB / V_HV_safe;

        Iref = -I_dab_ref;     % LV 기준: 방전이므로 음수
        I_fb = -I_DAB_out;

    else
        %% ---- ESS 충전 (3.1: CC, 3.2: CV) ----
        if V_LV_safe >= V_LV_min && V_LV_safe <= V_LV_max
            % CC 충전 (mode 3.1)
            mode_active = 3.1;
            I_available        = I_threshold - I_load;
            I_charge_available = I_available * V_HV_safe / max(V_LV, 1);
            Iref = min(I_charge_available, I_charge_CC);
        else
            % CV 충전 (mode 3.2)
            mode_active = 3.2;
            I_available        = I_threshold - I_load;
            I_charge_available = I_available * V_HV_safe / max(V_LV, 1); %#ok<NASGU>

            err_cv       = V_LV_ref - V_LV;
            integrator_cv = integrator_cv + err_cv / fs;
            Iref_cv      = Kp_cv * err_cv + Ki_cv * integrator_cv;
            Iref         = max(0, min(I_charge_CC, Iref_cv));
        end

        I_fb = I_LV;
    end

    %% ESS에서만 Soft-start
    if softStart_enable
        delta = Iref - Iref_soft;
        step  = sign(delta) * min(abs(delta), Iref_ramp_rate);
        Iref_soft = Iref_soft + step;
    else
        Iref_soft = Iref;
    end
end

%% ================= 배터리 출력 전류 하한 제한 =================
if mode_active == 1 || mode_active == 2
    I_LV_ref = Iref * n_transformer;
    if I_LV_ref < I_LV_min_limit
        Iref = I_LV_min_limit / n_transformer;
    end
end

%% ================= 외부 위상 PI =================
err_ext = Iref_soft - I_fb;

switch mode_active
    case 1
        Kp = Kp_ext; Ki = Ki_ext; integrator = integrator_ups;
    case 2
        Kp = Kp_ext; Ki = Ki_ext; integrator = integrator_discharge;
    case {3.1, 3.2}
        Kp = Kp_ext; Ki = Ki_ext; integrator = integrator_charge;
    otherwise
        Kp = Kp_ext; Ki = Ki_ext; integrator = integrator_ext;
end

integrator   = integrator + err_ext / fs;
phi_ext_deg  = Kp * err_ext + Ki * integrator;

switch mode_active
    case 1
        integrator_ups       = integrator;
    case 2
        integrator_discharge = integrator;
    case {3.1,3.2}
        integrator_charge    = integrator;
    otherwise
        integrator_ext       = integrator;
end

phi_ext_deg = max(-90, min(90, phi_ext_deg));
phi_ext_out = phi_ext_deg;

%% ================= 내부 위상 (EPS) =================
k = V_HV / (n_transformer * V_LV_safe);
if k > 1
    alpha_side_mode = 1;
    phi_int_deg     = 180 * (1 - 1/k);
else
    alpha_side_mode = 2;
    phi_int_deg     = 180 * (1 - k);
end

phi_int_deg = min(max(phi_int_deg, -90), 90);
phi_int_out = phi_int_deg;
alpha_side_mode_out = double(alpha_side_mode);

%% ================= 게이트 타이밍 =================
t_int = phi_int_deg      * deg2time;
t_ext = abs(phi_ext_deg) * deg2time;

if phi_ext_deg >= 0
    t1b = 0;     t5b = t_ext;
else
    t1b = t_ext; t5b = 0;
end

if alpha_side_mode == 1
    t1 = t1b;
    t4 = t1b + t_int;
    t5 = t5b;
    t8 = t5b;
else
    t1 = t1b;
    t4 = t1b;
    t5 = t5b;
    t8 = t5b + t_int;
end

previous_mode = power_loss_state;
end
