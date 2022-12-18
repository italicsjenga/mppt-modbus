use crate::datatypes::*;
use serde::{Deserialize, Serialize};
use std::fmt::{self, Debug, Display};

#[derive(Serialize, Deserialize, Debug)]
pub struct MpptRam {
    // scaling values
    pub v_pu_hi: Raw,
    pub v_pu_lo: Raw,
    pub i_pu_hi: Raw,
    pub i_pu_lo: Raw,
    pub ver_sw: Raw,
    // filtered ADC
    pub adc_vb_f_med: Raw,
    pub adc_vbterm_f: Raw,
    pub adc_vbs_f: Raw,
    pub adc_va_f: Raw,
    pub adc_ib_f_shadow: Raw,
    pub adc_ia_f_shadow: Raw,
    pub adc_p12_f: Raw,
    pub adc_p3_f: Raw,
    pub adc_pmeter_f: Raw,
    pub adc_p18_f: Raw,
    pub adc_v_ref: Raw,
    // temperatures
    pub t_hs: Raw,
    pub t_rts: Raw,
    pub t_batt: Raw,
    // status
    pub adc_vb_f_1m: Raw,
    pub adc_ib_f_1m: Raw,
    pub vb_min: Raw,
    pub vb_max: Raw,
    pub hourmeter_hi: Raw,
    pub hourmeter_lo: Raw,
    pub fault_all: Raw,
    pub alarm_hi: Raw,
    pub alarm_lo: Raw,
    pub dip_all: Raw,
    pub led_state: Raw,
    // charger
    pub charge_state: Raw,
    pub vb_ref: Raw,
    pub ahc_r_hi: Raw,
    pub ahc_r_lo: Raw,
    pub ahc_t_hi: Raw,
    pub ahc_t_lo: Raw,
    pub kwhc_r: Raw,
    pub kwhc_t: Raw,
    // MpptRam
    pub power_out_shadow: Raw,
    pub power_in_shadow: Raw,
    pub sweep_pin_max: Raw,
    pub sweep_vmp: Raw,
    pub sweep_voc: Raw,
    // logger - today's values
    pub vb_min_daily: Raw,
    pub vb_max_daily: Raw,
    pub va_max_daily: Raw,
    pub ahc_daily: Raw,
    pub whc_daily: Raw,
    pub flags_daily: Raw,
    pub pout_max_daily: Raw,
    pub tb_min_daily: Raw,
    pub tb_max_daily: Raw,
    pub fault_daily: Raw,
    pub alarm_daily_hi: Raw,
    pub alarm_daily_lo: Raw,
    pub time_ab_daily: Raw,
    pub time_eq_daily: Raw,
    pub time_fl_daily: Raw,
    // manual control
    pub ib_ref_slave: Raw,
    pub vb_ref_slave: Raw,
    pub va_ref_fixed: Raw,
    pub va_ref_fixed_pct: Raw,
}

impl MpptRam {
    pub fn get_fake() -> Self {
        Self {
            v_pu_hi: Raw::from_u16(0x0000),
            v_pu_lo: Raw::from_u16(0x0000),
            i_pu_hi: Raw::from_u16(0x0000),
            i_pu_lo: Raw::from_u16(0x0000),
            ver_sw: Raw::from_u16(0x0000),
            adc_vb_f_med: Raw::from_u16(0x0000),
            adc_vbterm_f: Raw::from_u16(0x0000),
            adc_vbs_f: Raw::from_u16(0x0000),
            adc_va_f: Raw::from_u16(0x0000),
            adc_ib_f_shadow: Raw::from_u16(0x0000),
            adc_ia_f_shadow: Raw::from_u16(0x0000),
            adc_p12_f: Raw::from_u16(0x0000),
            adc_p3_f: Raw::from_u16(0x0000),
            adc_pmeter_f: Raw::from_u16(0x0000),
            adc_p18_f: Raw::from_u16(0x0000),
            adc_v_ref: Raw::from_u16(0x0000),
            t_hs: Raw::from_u16(0x0000),
            t_rts: Raw::from_u16(0x0000),
            t_batt: Raw::from_u16(0x0000),
            adc_vb_f_1m: Raw::from_u16(0x0000),
            adc_ib_f_1m: Raw::from_u16(0x0000),
            vb_min: Raw::from_u16(0x0000),
            vb_max: Raw::from_u16(0x0000),
            hourmeter_hi: Raw::from_u16(0x0000),
            hourmeter_lo: Raw::from_u16(0x0000),
            fault_all: Raw::from_u16(0x0000),
            alarm_hi: Raw::from_u16(0x0000),
            alarm_lo: Raw::from_u16(0x0000),
            dip_all: Raw::from_u16(0x0000),
            led_state: Raw::from_u16(0x0000),
            charge_state: Raw::from_u16(0x0000),
            vb_ref: Raw::from_u16(0x0000),
            ahc_r_hi: Raw::from_u16(0x0000),
            ahc_r_lo: Raw::from_u16(0x0000),
            ahc_t_hi: Raw::from_u16(0x0000),
            ahc_t_lo: Raw::from_u16(0x0000),
            kwhc_r: Raw::from_u16(0x0000),
            kwhc_t: Raw::from_u16(0x0000),
            power_out_shadow: Raw::from_u16(0x0000),
            power_in_shadow: Raw::from_u16(0x0000),
            sweep_pin_max: Raw::from_u16(0x0000),
            sweep_vmp: Raw::from_u16(0x0000),
            sweep_voc: Raw::from_u16(0x0000),
            vb_min_daily: Raw::from_u16(0x0000),
            vb_max_daily: Raw::from_u16(0x0000),
            va_max_daily: Raw::from_u16(0x0000),
            ahc_daily: Raw::from_u16(0x0000),
            whc_daily: Raw::from_u16(0x0000),
            flags_daily: Raw::from_u16(0x0000),
            pout_max_daily: Raw::from_u16(0x0000),
            tb_min_daily: Raw::from_u16(0x0000),
            tb_max_daily: Raw::from_u16(0x0000),
            fault_daily: Raw::from_u16(0x0000),
            alarm_daily_hi: Raw::from_u16(0x0000),
            alarm_daily_lo: Raw::from_u16(0x0000),
            time_ab_daily: Raw::from_u16(0x0000),
            time_eq_daily: Raw::from_u16(0x0000),
            time_fl_daily: Raw::from_u16(0x0000),
            ib_ref_slave: Raw::from_u16(0x0000),
            vb_ref_slave: Raw::from_u16(0x0000),
            va_ref_fixed: Raw::from_u16(0x0000),
            va_ref_fixed_pct: Raw::from_u16(0x0000),
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
