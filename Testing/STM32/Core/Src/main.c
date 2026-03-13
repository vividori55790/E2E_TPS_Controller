/* main.c — DAB Converter STM32 Firmware (Redesigned)
 * Protocol: dab_protocol.h
 * HW: STM32G474RE, HRTIM PWM, ADC, USART2 */
#include "main.h"
#include "dab_protocol.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ═══ Peripheral handles (CubeMX) ═══ */
ADC_HandleTypeDef hadc2;
HRTIM_HandleTypeDef hhrtim1;
UART_HandleTypeDef huart2;

/* ═══ Runtime State ═══ */
typedef struct {
    float Kp_ext, Ki_ext, Phi_Manual, DeadTime_ns, D1_ext_in, D2_ext_in;
    float I_ess_cmd, Kpv, Kiv, Time_Hour, V_Grid_sim;
    uint8_t Mode_Select, Ctrl_Mode_Ext, EMS_Mode_Select;
} RuntimeInputs;

typedef struct {
    float Fs, L_leakage, n_transformer, MaxSlewRate, DeadZone;
    float GainSched_Thr, GainSched_Factor, SoftStartTime, MaxPhaseRate;
    float Flux_Kp, Flux_Ki, Flux_Limit;
    uint8_t EnableFF, EnableFluxBal;
} DABParams;

typedef struct {
    float V_HV_Ref, V_HV_Band, V_Ref_Ramp_Rate;
    float V_Pack_Max, V_CV_Start, V_ESS_Limit, V_UPS_Limit, V_Min_Limit;
    float I_Charge_Limit, I_Discharge_Limit, K_CV, InitDelay;
    float ESS_Charge_Rate, ESS_Discharge_Rate;
    float TOU_OffPk_Start, TOU_OffPk_End;
    float TOU_OnPk1_Start, TOU_OnPk1_End, TOU_OnPk2_Start, TOU_OnPk2_End;
    float SOC_Max_Thr, SOC_Min_Thr;
    float P_load_threshold, P_DAB_max, n_transformer, I_LV_min_limit;
    float I_ESS_Max, I_UPS_Max;
} EMSParams;

typedef struct {
    float V_HV, I_HV, V_LV, I_LV, I_L_pri, temp, V_Grid;
} Sensors;

/* Globals */
static RuntimeInputs rt = {
    .Kp_ext=5,.Ki_ext=50,.Mode_Select=0,.Phi_Manual=0,
    .Ctrl_Mode_Ext=0,.DeadTime_ns=200,.D1_ext_in=0.5f,.D2_ext_in=0.5f,
    .I_ess_cmd=0,.Kpv=2,.Kiv=10,.EMS_Mode_Select=0,.Time_Hour=12,.V_Grid_sim=380
};
static DABParams dab = {
    .Fs=80e3f,.L_leakage=90e-6f,.n_transformer=4,.EnableFF=1,
    .MaxSlewRate=50000,.DeadZone=0,.GainSched_Thr=2,.GainSched_Factor=1,
    .SoftStartTime=0.2f,.MaxPhaseRate=30000,.EnableFluxBal=1,
    .Flux_Kp=0.005f,.Flux_Ki=0.05f,.Flux_Limit=0.02f
};
static EMSParams ems = {
    .V_HV_Ref=380,.V_HV_Band=30,.V_Ref_Ramp_Rate=500,
    .V_Pack_Max=109.2f,.V_CV_Start=108,.V_ESS_Limit=93,.V_UPS_Limit=87,.V_Min_Limit=84,
    .I_Charge_Limit=30,.I_Discharge_Limit=50,.K_CV=5,.InitDelay=0.001f,
    .ESS_Charge_Rate=10,.ESS_Discharge_Rate=10,
    .TOU_OffPk_Start=23,.TOU_OffPk_End=8,
    .TOU_OnPk1_Start=11,.TOU_OnPk1_End=12,.TOU_OnPk2_Start=13,.TOU_OnPk2_End=17,
    .SOC_Max_Thr=95,.SOC_Min_Thr=20,.P_load_threshold=500,.P_DAB_max=1000,
    .n_transformer=4,.I_LV_min_limit=-15,.I_ESS_Max=10,.I_UPS_Max=30
};
static Sensors sensor = {.temp=25};
static uint8_t running = 0;

/* EMS/DAB computed outputs (dummy) */
static float ems_Pcmd=0, ems_SOC=50, dab_phi=0, dab_D1=0.5f, dab_D2=0.5f;
static float dab_Pfb=0, dab_err=0, dab_integ=0;
static uint8_t ems_pwm_en=0, ems_mode=0, ems_grid=1;

/* UART RX */
static uint8_t rx_byte, rx_buf[256];
static uint16_t rx_idx=0, rx_expect=0;
static volatile uint8_t rx_ready=0;
static uint8_t rx_cmd_type=0;

/* ═══ Helpers ═══ */
static uint32_t rng=0xDEADBEEF;
static float randf(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return(float)(rng&0xFFFF)/65535.0f;}

static uint8_t xor_checksum(uint8_t*d,uint16_t len){
    uint8_t c=0;for(uint16_t i=0;i<len;i++)c^=d[i];return c;
}

/* ═══ Dummy Sensor Generation ═══ */
static void Generate_Dummy(void){
    if(running){
        sensor.V_Grid = rt.V_Grid_sim + (randf()-0.5f)*2;
        sensor.V_HV = sensor.V_Grid;  /* grid-tied approximation */
        sensor.V_LV = 48.0f + (randf()-0.5f)*0.5f;
        float P_load = ems.P_load_threshold * 0.8f + randf()*200;
        float P_ess = (P_load > ems.P_load_threshold) ? (P_load - ems.P_load_threshold)*0.5f : 0;
        sensor.I_HV = (P_load - P_ess) / fmaxf(sensor.V_HV,1);
        sensor.I_LV = P_load / fmaxf(sensor.V_LV,1);
        sensor.I_L_pri = sensor.I_LV * 0.1f;
        sensor.temp += (2.0f-(sensor.temp-30)*0.01f)*0.05f;
        /* dummy EMS/DAB outputs */
        ems_Pcmd = P_load; ems_SOC = 50+randf()*5; ems_pwm_en=1;
        ems_mode = (sensor.V_Grid < (ems.V_HV_Ref-ems.V_HV_Band)) ? 1 : 2;
        ems_grid = (ems_mode!=1);
        dab_phi = rt.Phi_Manual + randf()*2;
        dab_D1 = rt.D1_ext_in; dab_D2 = rt.D2_ext_in;
        dab_Pfb = P_load; dab_err = randf()*0.1f; dab_integ = randf()*0.01f;
    } else {
        sensor.V_HV = 380+(randf()-0.5f)*2;
        sensor.V_LV = 48+(randf()-0.5f)*0.5f;
        sensor.I_HV = sensor.I_LV = sensor.I_L_pri = 0;
        sensor.V_Grid = rt.V_Grid_sim;
        sensor.temp += (25-sensor.temp)*0.002f;
        ems_Pcmd=0;ems_pwm_en=0;ems_mode=0;ems_grid=0;
        dab_phi=0;dab_Pfb=0;dab_err=0;dab_integ=0;
    }
    if(sensor.temp<20)sensor.temp=20;
    if(sensor.temp>120)sensor.temp=120;
}

/* ═══ Send Telemetry ═══ */
static uint8_t tx_buf[128];
static void Send_Telemetry(void){
    TelemPacket t;
    t.start=PKT_TELEM;
    t.V_HV=sensor.V_HV; t.I_HV=sensor.I_HV;
    t.V_LV=sensor.V_LV; t.I_LV=sensor.I_LV;
    t.I_L_pri=sensor.I_L_pri; t.temp=sensor.temp; t.V_Grid=sensor.V_Grid;
    t.ems_Power_cmd=ems_Pcmd; t.ems_SOC_pct=ems_SOC;
    t.ems_PWM_En=ems_pwm_en; t.ems_mode=ems_mode; t.ems_GridSW=ems_grid;
    t.dab_phi=dab_phi; t.dab_D1=dab_D1; t.dab_D2=dab_D2;
    t.dab_Pfb=dab_Pfb; t.dab_err=dab_err; t.dab_integ=dab_integ;
    t.status=0;
    if(running) t.status|=ST_RUNNING;
    if(sensor.V_HV>450) t.status|=ST_OV;
    if(sensor.I_HV>12) t.status|=ST_OC;
    if(sensor.temp>100) t.status|=ST_OT;
    t.checksum = xor_checksum((uint8_t*)&t, sizeof(TelemPacket)-2);
    t.end=PKT_END;
    memcpy(tx_buf,&t,sizeof(TelemPacket));
    HAL_UART_Transmit(&huart2, tx_buf, sizeof(TelemPacket), 50);
}

/* ═══ Process Command ═══ */
static void Process_Command(void){
    if(rx_buf[rx_expect-1]!=PKT_END) return;
    switch(rx_cmd_type){
    case CMD_SET_RUNTIME: {
        CmdRuntime *p=(CmdRuntime*)rx_buf;
        rt.Kp_ext=p->Kp_ext; rt.Ki_ext=p->Ki_ext;
        rt.Mode_Select=p->Mode_Select; rt.Phi_Manual=p->Phi_Manual;
        rt.Ctrl_Mode_Ext=p->Ctrl_Mode_Ext; rt.DeadTime_ns=p->DeadTime_ns;
        rt.D1_ext_in=p->D1_ext_in; rt.D2_ext_in=p->D2_ext_in;
        rt.I_ess_cmd=p->I_ess_cmd; rt.Kpv=p->Kpv; rt.Kiv=p->Kiv;
        rt.EMS_Mode_Select=p->EMS_Mode_Select;
        rt.Time_Hour=p->Time_Hour; rt.V_Grid_sim=p->V_Grid_sim;
    } break;
    case CMD_SET_DAB_PARAMS: {
        CmdDABParams *p=(CmdDABParams*)rx_buf;
        dab.Fs=p->Fs; dab.L_leakage=p->L_leakage; dab.n_transformer=p->n_transformer;
        dab.EnableFF=p->EnableFF; dab.MaxSlewRate=p->MaxSlewRate; dab.DeadZone=p->DeadZone;
        dab.GainSched_Thr=p->GainSched_Thr; dab.GainSched_Factor=p->GainSched_Factor;
        dab.SoftStartTime=p->SoftStartTime; dab.MaxPhaseRate=p->MaxPhaseRate;
        dab.EnableFluxBal=p->EnableFluxBal;
        dab.Flux_Kp=p->Flux_Kp; dab.Flux_Ki=p->Flux_Ki; dab.Flux_Limit=p->Flux_Limit;
    } break;
    case CMD_SET_EMS_PARAMS: {
        CmdEMSParams *p=(CmdEMSParams*)rx_buf;
        ems.V_HV_Ref=p->V_HV_Ref; ems.V_HV_Band=p->V_HV_Band;
        ems.V_Ref_Ramp_Rate=p->V_Ref_Ramp_Rate; ems.V_Pack_Max=p->V_Pack_Max;
        ems.V_CV_Start=p->V_CV_Start; ems.V_ESS_Limit=p->V_ESS_Limit;
        ems.V_UPS_Limit=p->V_UPS_Limit; ems.V_Min_Limit=p->V_Min_Limit;
        ems.I_Charge_Limit=p->I_Charge_Limit; ems.I_Discharge_Limit=p->I_Discharge_Limit;
        ems.K_CV=p->K_CV; ems.InitDelay=p->InitDelay;
        ems.ESS_Charge_Rate=p->ESS_Charge_Rate; ems.ESS_Discharge_Rate=p->ESS_Discharge_Rate;
        ems.TOU_OffPk_Start=p->TOU_OffPk_Start; ems.TOU_OffPk_End=p->TOU_OffPk_End;
        ems.TOU_OnPk1_Start=p->TOU_OnPk1_Start; ems.TOU_OnPk1_End=p->TOU_OnPk1_End;
        ems.TOU_OnPk2_Start=p->TOU_OnPk2_Start; ems.TOU_OnPk2_End=p->TOU_OnPk2_End;
        ems.SOC_Max_Thr=p->SOC_Max_Thr; ems.SOC_Min_Thr=p->SOC_Min_Thr;
        ems.P_load_threshold=p->P_load_threshold; ems.P_DAB_max=p->P_DAB_max;
        ems.n_transformer=p->n_transformer; ems.I_LV_min_limit=p->I_LV_min_limit;
        ems.I_ESS_Max=p->I_ESS_Max; ems.I_UPS_Max=p->I_UPS_Max;
    } break;
    case CMD_RUN_CTRL: {
        CmdRunCtrl *p=(CmdRunCtrl*)rx_buf;
        running = p->run;
        if(running){
            HAL_HRTIM_WaveformOutputStart(&hhrtim1,
                HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
            HAL_HRTIM_WaveformCountStart(&hhrtim1,
                HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_B);
        } else {
            HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
            HAL_HRTIM_WaveformCountStop(&hhrtim1,
                HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_B);
        }
    } break;
    case CMD_REQ_TELEM: Send_Telemetry(); break;
    }
}

/* ═══ UART Receive Callback (state machine) ═══ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance!=USART2) return;
    if(rx_idx==0){
        if(rx_byte==PKT_START){rx_buf[rx_idx++]=rx_byte;}
    } else if(rx_idx==1){
        rx_buf[rx_idx++]=rx_byte;
        rx_cmd_type=rx_byte;
        rx_expect=pkt_size(rx_cmd_type);
        if(rx_expect==0) rx_idx=0; /* unknown cmd */
    } else {
        rx_buf[rx_idx++]=rx_byte;
        if(rx_idx>=rx_expect){
            rx_ready=rx_cmd_type;
            rx_idx=0;
        }
    }
    HAL_UART_Receive_IT(&huart2,(uint8_t*)&rx_byte,1);
}

/* ═══ CubeMX Functions (keep from existing project) ═══ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_ADC2_Init(void);

int main(void){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_HRTIM1_Init();
    MX_ADC2_Init();

    sensor.temp = 25.0f;
    HAL_UART_Receive_IT(&huart2,(uint8_t*)&rx_byte,1);

    uint32_t last_dummy=HAL_GetTick(), last_telem=HAL_GetTick();

    while(1){
        uint32_t now=HAL_GetTick();
        if(now-last_dummy>=10){ last_dummy=now; Generate_Dummy(); }
        if(now-last_telem>=50){ last_telem=now; Send_Telemetry(); }
        if(rx_ready){ uint8_t cmd=rx_ready; rx_ready=0; Process_Command(); (void)cmd; }
    }
}

/* ═══════════════════════════════════════════════════════
 * NOTE: Copy SystemClock_Config, MX_GPIO_Init,
 * MX_USART2_UART_Init, MX_HRTIM1_Init, MX_ADC2_Init,
 * and Error_Handler from your existing CubeMX-generated
 * main.c. Those functions are hardware-specific and
 * should not be modified manually.
 * ═══════════════════════════════════════════════════════ */
void Error_Handler(void){ __disable_irq(); while(1){} }
