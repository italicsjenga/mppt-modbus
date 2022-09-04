pub struct OffsetsRam {}

impl OffsetsRam {
    // scaling values
    pub const V_PU_HI: usize = 0x0000;
    pub const V_PU_LO: usize = 0x0001;
    pub const I_PU_HI: usize = 0x0002;
    pub const I_PU_LO: usize = 0x0003;
    pub const VER_SW: usize = 0x0004;
    // filtered ADC
    pub const ADC_VB_F_MED: usize = 0x0018;
    pub const ADC_VBTERM_F: usize = 0x0019;
    pub const ADC_VBS_F: usize = 0x001A;
    pub const ADC_VA_F: usize = 0x001B;
    pub const ADC_IB_F_SHADOW: usize = 0x001C;
    pub const ADC_IA_F_SHADOW: usize = 0x001D;
    pub const ADC_P12_F: usize = 0x001E;
    pub const ADC_P3_F: usize = 0x001F;
    pub const ADC_PMETER_F: usize = 0x0020;
    pub const ADC_P18_F: usize = 0x0021;
    pub const ADC_V_REF: usize = 0x0022;
    // temperatures
    pub const T_HS: usize = 0x0023;
    pub const T_RTS: usize = 0x0024;
    pub const T_BATT: usize = 0x0025;
    // status
    pub const ADC_VB_F_1M: usize = 0x0026;
    pub const ADC_IB_F_1M: usize = 0x0027;
    pub const VB_MIN: usize = 0x0028;
    pub const VB_MAX: usize = 0x0029;
    pub const HOURMETER_HI: usize = 0x002A;
    pub const HOURMETER_LO: usize = 0x002B;
    pub const FAULT_ALL: usize = 0x002C;
    pub const ALARM_HI: usize = 0x002E;
    pub const ALARM_LO: usize = 0x002F;
    pub const DIP_ALL: usize = 0x0030;
    pub const LED_STATE: usize = 0x0031;
    // charger
    pub const CHARGE_STATE: usize = 0x0032;
    pub const VB_REF: usize = 0x0033;
    pub const AHC_R_HI: usize = 0x0034;
    pub const AHC_R_LO: usize = 0x0035;
    pub const AHC_T_HI: usize = 0x0036;
    pub const AHC_T_LO: usize = 0x0037;
    pub const KWHC_R: usize = 0x0038;
    pub const KWHC_T: usize = 0x0039;
    // MPPT
    pub const POWER_OUT_SHADOW: usize = 0x003A;
    pub const POWER_IN_SHADOW: usize = 0x003B;
    pub const SWEEP_PIN_MAX: usize = 0x003C;
    pub const SWEEP_VMP: usize = 0x003D;
    pub const SWEEP_VOC: usize = 0x003E;
    // logger - today's values
    pub const VB_MIN_DAILY: usize = 0x0040;
    pub const VB_MAX_DAILY: usize = 0x0041;
    pub const VA_MAX_DAILY: usize = 0x0042;
    pub const AHC_DAILY: usize = 0x0043;
    pub const WHC_DAILY: usize = 0x0044;
    pub const FLAGS_DAILY: usize = 0x0045;
    pub const POUT_MAX_DAILY: usize = 0x0046;
    pub const TB_MIN_DAILY: usize = 0x0047;
    pub const TB_MAX_DAILY: usize = 0x0048;
    pub const FAULT_DAILY: usize = 0x0049;
    pub const ALARM_DAILY_HI: usize = 0x004B;
    pub const ALARM_DAILY_LO: usize = 0x004C;
    pub const TIME_AB_DAILY: usize = 0x004D;
    pub const TIME_EQ_DAILY: usize = 0x004E;
    pub const TIME_FL_DAILY: usize = 0x004F;
    // manual control
    pub const IB_REF_SLAVE: usize = 0x0058;
    pub const VB_REF_SLAVE: usize = 0x0059;
    pub const VA_REF_FIXED: usize = 0x005A;
    pub const VA_REF_FIXED_PCT: usize = 0x005B;
}

pub struct OffsetsEeprom {}

impl OffsetsEeprom {
    pub const EV_absorp: usize = 0xE000;
    pub const EV_float: usize = 0xE001;
    pub const Et_absorp: usize = 0xE002;
    pub const Et_absorp_ext: usize = 0xE003;
    pub const EV_absorp_ext: usize = 0xE004;
    pub const EV_float_cancel: usize = 0xE005;
    pub const Et_float_exit_cum: usize = 0xE006;
    pub const EV_eq: usize = 0xE007;
    pub const Et_eqcalendar: usize = 0xE008;
    pub const Et_eq_above: usize = 0xE009;
    pub const Et_eq_reg: usize = 0xE00A;
    pub const Et_batt_service: usize = 0xE00B;
    pub const EV_tempcomp: usize = 0xE00D;
    pub const EV_hvd: usize = 0xE00E;
    pub const EV_hvr: usize = 0xE00F;
    pub const Evb_ref_lim: usize = 0xE010;
    pub const ETb_max: usize = 0xE011;
    pub const ETb_min: usize = 0xE012;
    pub const EV_soc_g_gy: usize = 0xE015;
    pub const EV_soc_gy_g: usize = 0xE016;
    pub const EV_soc_y_yr: usize = 0xE017;
    pub const EV_soc_yr_r: usize = 0xE018;
    pub const Emodbus_id: usize = 0xE019;
    pub const Emeterbus_id: usize = 0xE01A;
    pub const EIb_lim: usize = 0xE01D;
    pub const EVa_ref_fixed_init: usize = 0xE020;
    pub const EVa_ref_fixed_pct_init: usize = 0xE021;
}
