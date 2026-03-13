/*  dab_protocol.h — DAB Converter Communication Protocol
 *  Packet format: [0xAA][CMD][payload...][0x55]  (PC→STM32)
 *                 [0xBB][payload...][checksum][0x55] (STM32→PC)
 */
#ifndef DAB_PROTOCOL_H
#define DAB_PROTOCOL_H
#include <stdint.h>

#define PKT_START  0xAA
#define PKT_TELEM  0xBB
#define PKT_END    0x55

/* Command types */
#define CMD_SET_RUNTIME     0x01
#define CMD_SET_DAB_PARAMS  0x02
#define CMD_SET_EMS_PARAMS  0x03
#define CMD_RUN_CTRL        0x10
#define CMD_REQ_TELEM       0x20

/* Status flags */
#define ST_RUNNING  0x01
#define ST_OV       0x02
#define ST_OC       0x04
#define ST_OT       0x08

#pragma pack(push,1)

/* ── 0x01: Runtime Control Inputs (frequently changed) ── */
typedef struct {
    uint8_t start;          /* 0xAA */
    uint8_t cmd;            /* 0x01 */
    /* DAB Controller inputs */
    float   Kp_ext;
    float   Ki_ext;
    uint8_t Mode_Select;    /* 0=Auto, 1=Manual */
    float   Phi_Manual;     /* deg */
    uint8_t Ctrl_Mode_Ext;  /* 0=SPS,1=EPS,2=DPS,3=TPS */
    float   DeadTime_ns;
    float   D1_ext_in;
    float   D2_ext_in;
    /* EMS inputs */
    float   I_ess_cmd;
    float   Kpv;
    float   Kiv;
    uint8_t EMS_Mode_Select;/* 0=Auto, 1=Manual */
    float   Time_Hour;
    float   V_Grid_sim;     /* simulated grid voltage */
    uint8_t end;            /* 0x55 */
} CmdRuntime;               /* 52 bytes */

/* ── 0x02: DAB Controller Block Parameters ── */
typedef struct {
    uint8_t start; uint8_t cmd;
    float Fs;               /* Hz */
    float L_leakage;        /* H */
    float n_transformer;
    uint8_t EnableFF;
    float MaxSlewRate;      /* W/s */
    float DeadZone;         /* A */
    float GainSched_Thr;
    float GainSched_Factor;
    float SoftStartTime;    /* s */
    float MaxPhaseRate;     /* deg/s */
    uint8_t EnableFluxBal;
    float Flux_Kp;
    float Flux_Ki;
    float Flux_Limit;
    uint8_t end;
} CmdDABParams;             /* 53 bytes */

/* ── 0x03: EMS Block Parameters ── */
typedef struct {
    uint8_t start; uint8_t cmd;
    float V_HV_Ref;         float V_HV_Band;
    float V_Ref_Ramp_Rate;  float V_Pack_Max;
    float V_CV_Start;       float V_ESS_Limit;
    float V_UPS_Limit;      float V_Min_Limit;
    float I_Charge_Limit;   float I_Discharge_Limit;
    float K_CV;             float InitDelay;
    float ESS_Charge_Rate;  float ESS_Discharge_Rate;
    float TOU_OffPk_Start;  float TOU_OffPk_End;
    float TOU_OnPk1_Start;  float TOU_OnPk1_End;
    float TOU_OnPk2_Start;  float TOU_OnPk2_End;
    float SOC_Max_Thr;      float SOC_Min_Thr;
    float P_load_threshold; float P_DAB_max;
    float n_transformer;    float I_LV_min_limit;
    float I_ESS_Max;        float I_UPS_Max;
    uint8_t end;
} CmdEMSParams;             /* 115 bytes */

/* ── 0x10: Run Control ── */
typedef struct {
    uint8_t start; uint8_t cmd;
    uint8_t run;  /* 1=start, 0=stop */
    uint8_t end;
} CmdRunCtrl;                /* 4 bytes */

/* ── Telemetry (STM32→PC) ── */
typedef struct {
    uint8_t start;           /* 0xBB */
    /* Sensors */
    float V_HV, I_HV, V_LV, I_LV, I_L_pri, temp, V_Grid;
    /* EMS outputs */
    float ems_Power_cmd, ems_SOC_pct;
    uint8_t ems_PWM_En, ems_mode, ems_GridSW;
    /* DAB outputs */
    float dab_phi, dab_D1, dab_D2, dab_Pfb, dab_err, dab_integ;
    /* System */
    uint8_t status, checksum, end;
} TelemPacket;               /* 67 bytes */

#pragma pack(pop)

/* Packet size lookup */
static inline uint16_t pkt_size(uint8_t cmd) {
    switch(cmd) {
        case CMD_SET_RUNTIME:    return sizeof(CmdRuntime);
        case CMD_SET_DAB_PARAMS: return sizeof(CmdDABParams);
        case CMD_SET_EMS_PARAMS: return sizeof(CmdEMSParams);
        case CMD_RUN_CTRL:       return sizeof(CmdRunCtrl);
        case CMD_REQ_TELEM:      return 3;
        default: return 0;
    }
}

#endif
