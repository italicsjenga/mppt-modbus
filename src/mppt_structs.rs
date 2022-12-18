use crate::datatypes::*;
use serde::{Deserialize, Serialize};
use std::fmt::{self, Debug, Display};

#[derive(Serialize, Deserialize, Debug)]
pub struct MpptRam {
    // scaling values
    pub v_pu: f32,
    pub i_pu: f32,
    pub ver_sw: u16,
    // filtered ADC
    pub adc_vb_f_med: f32,
    pub adc_vbterm_f: f32,
    pub adc_vbs_f: f32,
    pub adc_va_f: f32,
    pub adc_ib_f_shadow: f32,
    pub adc_ia_f_shadow: f32,
    pub adc_p12_f: f32,
    pub adc_p3_f: f32,
    pub adc_pmeter_f: f32,
    pub adc_p18_f: f32,
    pub adc_v_ref: f32,
    // temperatures
    pub t_hs: u16,
    pub t_rts: u16,
    pub t_batt: u16,
    // status
    pub adc_vb_f_1m: f32,
    pub adc_ib_f_1m: f32,
    pub vb_min: f32,
    pub vb_max: f32,
    pub hourmeter_hi: u16,
    pub hourmeter_lo: u16,
    pub fault_all: u16,
    pub alarm_hi: u16,
    pub alarm_lo: u16,
    pub dip_all: u16,
    pub led_state: u16,
    // charger
    pub charge_state: u16,
    pub vb_ref: f32,
    pub ahc_r_hi: u16,
    pub ahc_r_lo: u16,
    pub ahc_t_hi: u16,
    pub ahc_t_lo: u16,
    pub kwhc_r: u16,
    pub kwhc_t: u16,
    // MpptRam
    pub power_out_shadow: f32,
    pub power_in_shadow: f32,
    pub sweep_pin_max: f32,
    pub sweep_vmp: f32,
    pub sweep_voc: f32,
    // logger - today's values
    pub vb_min_daily: f32,
    pub vb_max_daily: f32,
    pub va_max_daily: f32,
    pub ahc_daily: f32,
    pub whc_daily: u16,
    pub flags_daily: u16,
    pub pout_max_daily: f32,
    pub tb_min_daily: u16,
    pub tb_max_daily: u16,
    pub fault_daily: u16,
    pub alarm_daily_hi: u16,
    pub alarm_daily_lo: u16,
    pub time_ab_daily: u16,
    pub time_eq_daily: u16,
    pub time_fl_daily: u16,
    // manual control
    pub ib_ref_slave: f32,
    pub vb_ref_slave: f32,
    pub va_ref_fixed: f32,
    pub va_ref_fixed_pct: f32,
}

impl MpptRam {
    pub fn get_fake() -> Self {
        Self {
            v_pu: 0.,
            i_pu: 0.,
            ver_sw: 0,
            adc_vb_f_med: 0.,
            adc_vbterm_f: 0.,
            adc_vbs_f: 0.,
            adc_va_f: 0.,
            adc_ib_f_shadow: 0.,
            adc_ia_f_shadow: 0.,
            adc_p12_f: 0.,
            adc_p3_f: 0.,
            adc_pmeter_f: 0.,
            adc_p18_f: 0.,
            adc_v_ref: 0.,
            t_hs: 0,
            t_rts: 0,
            t_batt: 0,
            adc_vb_f_1m: 0.,
            adc_ib_f_1m: 0.,
            vb_min: 0.,
            vb_max: 0.,
            hourmeter_hi: 0,
            hourmeter_lo: 0,
            fault_all: 0,
            alarm_hi: 0,
            alarm_lo: 0,
            dip_all: 0,
            led_state: 0,
            charge_state: 0,
            vb_ref: 0.,
            ahc_r_hi: 0,
            ahc_r_lo: 0,
            ahc_t_hi: 0,
            ahc_t_lo: 0,
            kwhc_r: 0,
            kwhc_t: 0,
            power_out_shadow: 0.,
            power_in_shadow: 0.,
            sweep_pin_max: 0.,
            sweep_vmp: 0.,
            sweep_voc: 0.,
            vb_min_daily: 0.,
            vb_max_daily: 0.,
            va_max_daily: 0.,
            ahc_daily: 0.,
            whc_daily: 0,
            flags_daily: 0,
            pout_max_daily: 0.,
            tb_min_daily: 0,
            tb_max_daily: 0,
            fault_daily: 0,
            alarm_daily_hi: 0,
            alarm_daily_lo: 0,
            time_ab_daily: 0,
            time_eq_daily: 0,
            time_fl_daily: 0,
            ib_ref_slave: 0.,
            vb_ref_slave: 0.,
            va_ref_fixed: 0.,
            va_ref_fixed_pct: 0.,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MpptEeprom {
    pub ev_absorp: Voltage,
    pub ev_float: Voltage,
    pub et_absorp: Raw,
    pub et_absorp_ext: Raw,
    pub ev_absorp_ext: Voltage,
    pub ev_float_cancel: Voltage,
    pub et_float_exit_cum: Raw,
    pub ev_eq: Voltage,
    pub et_eqcalendar: Raw,
    pub et_eq_above: Raw,
    pub et_eq_reg: Raw,
    pub et_batt_service: Raw,
    pub ev_tempcomp: Tempcomp,
    pub ev_hvd: Voltage,
    pub ev_hvr: Voltage,
    pub evb_ref_lim: Voltage,
    pub etb_max: Raw,
    pub etb_min: Raw,
    pub ev_soc_g_gy: Voltage,
    pub ev_soc_gy_y: Voltage,
    pub ev_soc_y_yr: Voltage,
    pub ev_soc_yr_r: Voltage,
    pub emodbus_id: Raw,
    pub emeterbus_id: Raw,
    pub eib_lim: Current,
    pub eva_ref_fixed_init: Voltage,
    pub eva_ref_fixed_pct_init: VoltagePercentage,
}

impl MpptEeprom {
    pub fn get_fake() -> Self {
        Self {
            ev_absorp: Voltage::from_u16(0x0000),
            ev_float: Voltage::from_u16(0x0000),
            et_absorp: Raw::from_u16(0x0000),
            et_absorp_ext: Raw::from_u16(0x0000),
            ev_absorp_ext: Voltage::from_u16(0x0000),
            ev_float_cancel: Voltage::from_u16(0x0000),
            et_float_exit_cum: Raw::from_u16(0x0000),
            ev_eq: Voltage::from_u16(0x0000),
            et_eqcalendar: Raw::from_u16(0x0000),
            et_eq_above: Raw::from_u16(0x0000),
            et_eq_reg: Raw::from_u16(0x0000),
            et_batt_service: Raw::from_u16(0x0000),
            ev_tempcomp: Tempcomp::from_u16(0x0000),
            ev_hvd: Voltage::from_u16(0x0000),
            ev_hvr: Voltage::from_u16(0x0000),
            evb_ref_lim: Voltage::from_u16(0x0000),
            etb_max: Raw::from_u16(0x0000),
            etb_min: Raw::from_u16(0x0000),
            ev_soc_g_gy: Voltage::from_u16(0x0000),
            ev_soc_gy_y: Voltage::from_u16(0x0000),
            ev_soc_y_yr: Voltage::from_u16(0x0000),
            ev_soc_yr_r: Voltage::from_u16(0x0000),
            emodbus_id: Raw::from_u16(0x0000),
            emeterbus_id: Raw::from_u16(0x0000),
            eib_lim: Current::from_u16(0x0000),
            eva_ref_fixed_init: Voltage::from_u16(0x0000),
            eva_ref_fixed_pct_init: VoltagePercentage::from_u16(0x0000),
        }
    }
}

impl Display for MpptEeprom {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "ev_absorp: {}
ev_float: {}
et_absorp: {}
et_absorp_ext: {}
ev_absorp_ext: {}
ev_float_cancel: {}
et_float_exit_cum: {}
ev_eq: {}
et_eqcalendar: {}
et_eq_above: {}
et_eq_reg: {}
et_batt_service: {}
ev_tempcomp: {}
ev_hvd: {}
ev_hvr: {}
evb_ref_lim: {}
etb_max: {}
etb_min: {}
ev_soc_g_gy: {}
ev_soc_gy_y: {}
ev_soc_y_yr: {}
ev_soc_yr_r: {}
emodbus_id: {}
emeterbus_id: {}
eib_lim: {}
eva_ref_fixed_init: {}
eva_ref_fixed_pct_init: {}",
            self.ev_absorp.to_string_v(),
            self.ev_float.to_string_v(),
            self.et_absorp.to_string_v(),
            self.et_absorp_ext.to_string_v(),
            self.ev_absorp_ext.to_string_v(),
            self.ev_float_cancel.to_string_v(),
            self.et_float_exit_cum.to_string_v(),
            self.ev_eq.to_string_v(),
            self.et_eqcalendar.to_string_v(),
            self.et_eq_above.to_string_v(),
            self.et_eq_reg.to_string_v(),
            self.et_batt_service.to_string_v(),
            self.ev_tempcomp.to_string_v(),
            self.ev_hvd.to_string_v(),
            self.ev_hvr.to_string_v(),
            self.evb_ref_lim.to_string_v(),
            self.etb_max.to_string_v(),
            self.etb_min.to_string_v(),
            self.ev_soc_g_gy.to_string_v(),
            self.ev_soc_gy_y.to_string_v(),
            self.ev_soc_y_yr.to_string_v(),
            self.ev_soc_yr_r.to_string_v(),
            self.emodbus_id.to_string_v(),
            self.emeterbus_id.to_string_v(),
            self.eib_lim.to_string_v(),
            self.eva_ref_fixed_init.to_string_v(),
            self.eva_ref_fixed_pct_init.to_string_v()
        )
    }
}

#[derive(Serialize, Deserialize)]
pub struct MpptData {
    pub ram: MpptRam,
    pub eeprom: MpptEeprom,
}
