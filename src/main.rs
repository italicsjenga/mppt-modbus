#![allow(dead_code)]

mod datatypes;
mod mppt_structs;
mod offsets;
use crate::datatypes::*;
use crate::mppt_structs::{MpptData, MpptEeprom, MpptRam};
use crate::offsets::{OffsetsEeprom, OffsetsRam};
use clap::{Parser, Subcommand};
use libmodbus_rs::{Modbus, ModbusClient, ModbusRTU};
use std::{fmt::Debug, path::Path, process::Command, sync::Mutex};

const DEVICE_ID: u8 = 0x01;
const RAM_DATA_SIZE: u16 = 0x005B;
const EEPROM_BEGIN: u16 = 0xE000;
const EEPROM_DATA_SIZE: u16 = 0x0021;

const DEFAULT_SERIAL: &str = if cfg!(target_os = "linux") {
    "/dev/ttyUSB0"
} else if cfg!(target_os = "macos") {
    "/dev/tty.usbserial-DUT1"
} else {
    "unknown"
};

static INFO_SCALE: Mutex<Info> = Mutex::new(Info {
    v_scale: 1.,
    i_scale: 1.,
});

#[derive(Parser)]
#[clap(disable_help_subcommand = true)]
#[clap(author, about, long_about = None)]
struct Args {
    /// Serial port to connect to MPPT
    #[clap(short, long, default_value = DEFAULT_SERIAL)]
    serial_port: String,

    /// list serial ports on this system
    #[clap(long)]
    get_serial_ports: bool,

    /// Use fake data - for testing
    #[clap(long)]
    fake: bool,

    #[clap(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand)]
enum Commands {
    /// Get single EEPROM value
    Get { name: String },

    /// Set single EEPROM value
    Set { name: String, value: f32 },

    /// Get all RAM values
    GetRam,

    /// Print RAM and EEPROM values to JSON
    PrintJSON,
}

#[derive(Debug, Clone)]
struct Info {
    v_scale: f32,
    i_scale: f32,
}

impl Info {
    pub fn from(data: &[u16]) -> Self {
        Self {
            v_scale: data[0] as f32 + (data[1] as f32 / f32::powf(2., 16.)),
            i_scale: data[2] as f32 + (data[3] as f32 / f32::powf(2., 16.)),
        }
    }
    pub fn scale_power(&self, p: &u16) -> f32 {
        *p as f32 * self.v_scale * self.i_scale * f32::powf(2., -17.)
    }
    pub fn scale_voltage(&self, v: &u16) -> f32 {
        *v as f32 * self.v_scale * f32::powf(2., -15.)
    }
    pub fn scale_current(&self, i: &u16) -> f32 {
        *i as f32 * self.i_scale * f32::powf(2., -15.)
    }
    pub fn scale_voltage_f(&self, v: f32) -> f32 {
        v * self.v_scale * f32::powf(2., -15.)
    }
    pub fn scale_current_f(&self, i: f32) -> f32 {
        i * self.i_scale * f32::powf(2., -15.)
    }
}

fn main() {
    let args = Args::parse();
    if args.get_serial_ports {
        let output = Command::new("sh")
            .arg("-c")
            .arg("ls /dev/tty*")
            .output()
            .expect("failed to execute process");
        let s = match std::str::from_utf8(&output.stdout) {
            Ok(v) => v,
            Err(e) => panic!("Failed reading internal command output: {}", e),
        };
        println!("{}", s);
        return;
    }

    if !Path::new(&args.serial_port).exists() && !args.fake {
        println!(
            "Serial port {} does not exist\nTry \"mppt-control --help\" for usage instructions",
            args.serial_port
        );
        return;
    }

    let (_info, ram_data, eeprom_data, modbus) = if !args.fake {
        println!("Connecting to device on {}", args.serial_port);
        let modbus = connect_modbus(&args.serial_port);
        let (a, b, c) = get_data(&modbus);
        (a, b, c, Some(modbus))
    } else {
        let info = Info {
            v_scale: 1.,
            i_scale: 1.,
        };
        let ram_data = MpptRam::get_fake();
        let eeprom_data = MpptEeprom::get_fake();
        (info, ram_data, eeprom_data, None)
    };

    match args.command {
        Some(Commands::Get { name }) => {
            let t = match_datapoint(name.as_str(), &eeprom_data);
            println!("{name}: {}", t.to_string_v());
            return;
        }
        Some(Commands::Set { name, value }) => {
            println!("setting var {}", name);
            let t = match_datapoint(name.as_str(), &eeprom_data);
            println!("Existing value:\n  {:?}", t);
            let val = t.u16_from_f32(value);
            println!(
                "New value:\n  Scaled: {}, Raw: {}\n",
                t.get_scaled_from(val),
                val
            );
            match modbus {
                Some(modbus) => {
                    let offset = match_offset(&name) as u16;
                    println!("Writing {} to offset {}", val, offset);
                    modbus
                        .write_register(EEPROM_BEGIN as u16 + offset, val)
                        .expect("could not set va   lue");
                }
                None => {
                    println!("No modbus device connected");
                    let offset = match_offset(&name) as u16;
                    println!("Would write {} to offset {}", val, offset);
                }
            }
            return;
        }
        Some(Commands::GetRam) => {
            println!("ram: {:#?}", ram_data);
        }
        Some(Commands::PrintJSON) => {
            println!(
                "{}",
                serde_json::to_string(&MpptData {
                    ram: ram_data,
                    eeprom: eeprom_data
                })
                .expect("Could not format data as JSON!")
            );
        }
        None => {
            println!("{eeprom_data}");
        }
    }
}

fn connect_modbus(serial_port: &str) -> Modbus {
    let baud = 9600;
    let parity = 'N';
    let data_bit = 8;
    let stop_bit = 2;
    let mut modbus = Modbus::new_rtu(serial_port, baud, parity, data_bit, stop_bit)
        .expect("Could not create modbus device");
    modbus
        .set_slave(DEVICE_ID)
        .expect("Could not set client device");
    modbus
        .connect()
        .expect("Could not connect to client device");
    return modbus;
}

fn get_data(modbus: &Modbus) -> (Info, MpptRam, MpptEeprom) {
    let mut data_in: [u16; RAM_DATA_SIZE as usize + 1] = [0; RAM_DATA_SIZE as usize + 1];
    modbus
        .read_registers(0x0000, RAM_DATA_SIZE + 1, &mut data_in)
        .expect("couldnt");
    let info = Info::from(&data_in);

    let mut a = INFO_SCALE.lock().expect("Couldn't lock info");
    a.v_scale = info.v_scale;
    a.i_scale = info.i_scale;
    drop(a);

    let ram_data = MpptRam {
        v_pu_hi: Raw::from_u16(data_in[OffsetsRam::V_PU_HI]),
        v_pu_lo: Raw::from_u16(data_in[OffsetsRam::V_PU_LO]),
        i_pu_hi: Raw::from_u16(data_in[OffsetsRam::I_PU_HI]),
        i_pu_lo: Raw::from_u16(data_in[OffsetsRam::I_PU_LO]),
        ver_sw: Raw::from_u16(data_in[OffsetsRam::VER_SW]),
        adc_vb_f_med: Raw::from_u16(data_in[OffsetsRam::ADC_VB_F_MED]),
        adc_vbterm_f: Raw::from_u16(data_in[OffsetsRam::ADC_VBTERM_F]),
        adc_vbs_f: Raw::from_u16(data_in[OffsetsRam::ADC_VBS_F]),
        adc_va_f: Raw::from_u16(data_in[OffsetsRam::ADC_VA_F]),
        adc_ib_f_shadow: Raw::from_u16(data_in[OffsetsRam::ADC_IB_F_SHADOW]),
        adc_ia_f_shadow: Raw::from_u16(data_in[OffsetsRam::ADC_IA_F_SHADOW]),
        adc_p12_f: Raw::from_u16(data_in[OffsetsRam::ADC_P12_F]),
        adc_p3_f: Raw::from_u16(data_in[OffsetsRam::ADC_P3_F]),
        adc_pmeter_f: Raw::from_u16(data_in[OffsetsRam::ADC_PMETER_F]),
        adc_p18_f: Raw::from_u16(data_in[OffsetsRam::ADC_P18_F]),
        adc_v_ref: Raw::from_u16(data_in[OffsetsRam::ADC_V_REF]),
        t_hs: Raw::from_u16(data_in[OffsetsRam::T_HS]),
        t_rts: Raw::from_u16(data_in[OffsetsRam::T_RTS]),
        t_batt: Raw::from_u16(data_in[OffsetsRam::T_BATT]),
        adc_vb_f_1m: Raw::from_u16(data_in[OffsetsRam::ADC_VB_F_1M]),
        adc_ib_f_1m: Raw::from_u16(data_in[OffsetsRam::ADC_IB_F_1M]),
        vb_min: Raw::from_u16(data_in[OffsetsRam::VB_MIN]),
        vb_max: Raw::from_u16(data_in[OffsetsRam::VB_MAX]),
        hourmeter_hi: Raw::from_u16(data_in[OffsetsRam::HOURMETER_HI]),
        hourmeter_lo: Raw::from_u16(data_in[OffsetsRam::HOURMETER_LO]),
        fault_all: Raw::from_u16(data_in[OffsetsRam::FAULT_ALL]),
        alarm_hi: Raw::from_u16(data_in[OffsetsRam::ALARM_HI]),
        alarm_lo: Raw::from_u16(data_in[OffsetsRam::ALARM_LO]),
        dip_all: Raw::from_u16(data_in[OffsetsRam::DIP_ALL]),
        led_state: Raw::from_u16(data_in[OffsetsRam::LED_STATE]),
        charge_state: Raw::from_u16(data_in[OffsetsRam::CHARGE_STATE]),
        vb_ref: Raw::from_u16(data_in[OffsetsRam::VB_REF]),
        ahc_r_hi: Raw::from_u16(data_in[OffsetsRam::AHC_R_HI]),
        ahc_r_lo: Raw::from_u16(data_in[OffsetsRam::AHC_R_LO]),
        ahc_t_hi: Raw::from_u16(data_in[OffsetsRam::AHC_T_HI]),
        ahc_t_lo: Raw::from_u16(data_in[OffsetsRam::AHC_T_LO]),
        kwhc_r: Raw::from_u16(data_in[OffsetsRam::KWHC_R]),
        kwhc_t: Raw::from_u16(data_in[OffsetsRam::KWHC_T]),
        power_out_shadow: Raw::from_u16(data_in[OffsetsRam::POWER_OUT_SHADOW]),
        power_in_shadow: Raw::from_u16(data_in[OffsetsRam::POWER_IN_SHADOW]),
        sweep_pin_max: Raw::from_u16(data_in[OffsetsRam::SWEEP_PIN_MAX]),
        sweep_vmp: Raw::from_u16(data_in[OffsetsRam::SWEEP_VMP]),
        sweep_voc: Raw::from_u16(data_in[OffsetsRam::SWEEP_VOC]),
        vb_min_daily: Raw::from_u16(data_in[OffsetsRam::VB_MIN_DAILY]),
        vb_max_daily: Raw::from_u16(data_in[OffsetsRam::VB_MAX_DAILY]),
        va_max_daily: Raw::from_u16(data_in[OffsetsRam::VA_MAX_DAILY]),
        ahc_daily: Raw::from_u16(data_in[OffsetsRam::AHC_DAILY]),
        whc_daily: Raw::from_u16(data_in[OffsetsRam::WHC_DAILY]),
        flags_daily: Raw::from_u16(data_in[OffsetsRam::FLAGS_DAILY]),
        pout_max_daily: Raw::from_u16(data_in[OffsetsRam::POUT_MAX_DAILY]),
        tb_min_daily: Raw::from_u16(data_in[OffsetsRam::TB_MIN_DAILY]),
        tb_max_daily: Raw::from_u16(data_in[OffsetsRam::TB_MAX_DAILY]),
        fault_daily: Raw::from_u16(data_in[OffsetsRam::FAULT_DAILY]),
        alarm_daily_hi: Raw::from_u16(data_in[OffsetsRam::ALARM_DAILY_HI]),
        alarm_daily_lo: Raw::from_u16(data_in[OffsetsRam::ALARM_DAILY_LO]),
        time_ab_daily: Raw::from_u16(data_in[OffsetsRam::TIME_AB_DAILY]),
        time_eq_daily: Raw::from_u16(data_in[OffsetsRam::TIME_EQ_DAILY]),
        time_fl_daily: Raw::from_u16(data_in[OffsetsRam::TIME_FL_DAILY]),
        ib_ref_slave: Raw::from_u16(data_in[OffsetsRam::IB_REF_SLAVE]),
        vb_ref_slave: Raw::from_u16(data_in[OffsetsRam::VB_REF_SLAVE]),
        va_ref_fixed: Raw::from_u16(data_in[OffsetsRam::VA_REF_FIXED]),
        va_ref_fixed_pct: Raw::from_u16(data_in[OffsetsRam::VA_REF_FIXED_PCT]),
    };

    let mut data_in: [u16; EEPROM_DATA_SIZE as usize + 1] = [0; EEPROM_DATA_SIZE as usize + 1];
    modbus
        .read_registers(EEPROM_BEGIN, EEPROM_DATA_SIZE + 1, &mut data_in)
        .expect("could not get eeprom data");

    let eeprom_data = MpptEeprom {
        ev_absorp: Voltage::from_u16(data_in[OffsetsEeprom::EV_ABSORP]),
        ev_float: Voltage::from_u16(data_in[OffsetsEeprom::EV_FLOAT]),
        et_absorp: Raw::from_u16(data_in[OffsetsEeprom::ET_ABSORP]),
        et_absorp_ext: Raw::from_u16(data_in[OffsetsEeprom::ET_ABSORP_EXT]),
        ev_absorp_ext: Voltage::from_u16(data_in[OffsetsEeprom::EV_ABSORP_EXT]),
        ev_float_cancel: Voltage::from_u16(data_in[OffsetsEeprom::EV_FLOAT_CANCEL]),
        et_float_exit_cum: Raw::from_u16(data_in[OffsetsEeprom::ET_FLOAT_EXIT_CUM]),
        ev_eq: Voltage::from_u16(data_in[OffsetsEeprom::EV_EQ]),
        et_eqcalendar: Raw::from_u16(data_in[OffsetsEeprom::ET_EQCALENDAR]),
        et_eq_above: Raw::from_u16(data_in[OffsetsEeprom::ET_EQ_ABOVE]),
        et_eq_reg: Raw::from_u16(data_in[OffsetsEeprom::ET_EQ_REG]),
        et_batt_service: Raw::from_u16(data_in[OffsetsEeprom::ET_BATT_SERVICE]),
        ev_tempcomp: Tempcomp::from_u16(data_in[OffsetsEeprom::EV_TEMPCOMP]),
        ev_hvd: Voltage::from_u16(data_in[OffsetsEeprom::EV_HVD]),
        ev_hvr: Voltage::from_u16(data_in[OffsetsEeprom::EV_HVR]),
        evb_ref_lim: Voltage::from_u16(data_in[OffsetsEeprom::EVB_REF_LIM]),
        etb_max: Raw::from_u16(data_in[OffsetsEeprom::ETB_MAX]),
        etb_min: Raw::from_u16(data_in[OffsetsEeprom::ETB_MIN]),
        ev_soc_g_gy: Voltage::from_u16(data_in[OffsetsEeprom::EV_SOC_G_GY]),
        ev_soc_gy_y: Voltage::from_u16(data_in[OffsetsEeprom::EV_SOC_GY_Y]),
        ev_soc_y_yr: Voltage::from_u16(data_in[OffsetsEeprom::EV_SOC_Y_YR]),
        ev_soc_yr_r: Voltage::from_u16(data_in[OffsetsEeprom::EV_SOC_YR_R]),
        emodbus_id: Raw::from_u16(data_in[OffsetsEeprom::EMODBUS_ID]),
        emeterbus_id: Raw::from_u16(data_in[OffsetsEeprom::EMETERBUS_ID]),
        eib_lim: Current::from_u16(data_in[OffsetsEeprom::EIB_LIM]),
        eva_ref_fixed_init: Voltage::from_u16(data_in[OffsetsEeprom::EVA_REF_FIXED_INIT]),
        eva_ref_fixed_pct_init: VoltagePercentage::from_u16(
            data_in[OffsetsEeprom::EVA_REF_FIXED_PCT_INIT],
        ),
    };

    return (info, ram_data, eeprom_data);
}

fn match_datapoint(name: &str, data: &MpptEeprom) -> Box<dyn DataPoint> {
    match name.to_lowercase().as_str() {
        "ev_absorp" => Box::new(data.ev_absorp),
        "ev_float" => Box::new(data.ev_float),
        "et_absorp" => Box::new(data.et_absorp),
        "et_absorp_ext" => Box::new(data.et_absorp_ext),
        "ev_absorp_ext" => Box::new(data.ev_absorp_ext),
        "ev_float_cancel" => Box::new(data.ev_float_cancel),
        "et_float_exit_cum" => Box::new(data.et_float_exit_cum),
        "ev_eq" => Box::new(data.ev_eq),
        "et_eqcalendar" => Box::new(data.et_eqcalendar),
        "et_eq_above" => Box::new(data.et_eq_above),
        "et_eq_reg" => Box::new(data.et_eq_reg),
        "et_batt_service" => Box::new(data.et_batt_service),
        "ev_tempcomp" => Box::new(data.ev_tempcomp),
        "ev_hvd" => Box::new(data.ev_hvd),
        "ev_hvr" => Box::new(data.ev_hvr),
        "evb_ref_lim" => Box::new(data.evb_ref_lim),
        "etb_max" => Box::new(data.etb_max),
        "etb_min" => Box::new(data.etb_min),
        "ev_soc_g_gy" => Box::new(data.ev_soc_g_gy),
        "ev_soc_gy_y" => Box::new(data.ev_soc_gy_y),
        "ev_soc_y_yr" => Box::new(data.ev_soc_y_yr),
        "ev_soc_yr_r" => Box::new(data.ev_soc_yr_r),
        "emodbus_id" => Box::new(data.emodbus_id),
        "emeterbus_id" => Box::new(data.emeterbus_id),
        "eib_lim" => Box::new(data.eib_lim),
        "eva_ref_fixed_init" => Box::new(data.eva_ref_fixed_init),
        "eva_ref_fixed_pct_init" => Box::new(data.eva_ref_fixed_pct_init),
        &_ => todo!(),
    }
}

fn match_offset(name: &str) -> usize {
    match name.to_lowercase().as_str() {
        "ev_absorp" => OffsetsEeprom::EV_ABSORP,
        "ev_float" => OffsetsEeprom::EV_FLOAT,
        "et_absorp" => OffsetsEeprom::ET_ABSORP,
        "et_absorp_ext" => OffsetsEeprom::ET_BATT_SERVICE,
        "ev_absorp_ext" => OffsetsEeprom::EV_ABSORP_EXT,
        "ev_float_cancel" => OffsetsEeprom::EV_FLOAT_CANCEL,
        "et_float_exit_cum" => OffsetsEeprom::ET_FLOAT_EXIT_CUM,
        "ev_eq" => OffsetsEeprom::EV_EQ,
        "et_eqcalendar" => OffsetsEeprom::ET_EQCALENDAR,
        "et_eq_above" => OffsetsEeprom::ET_EQ_ABOVE,
        "et_eq_reg" => OffsetsEeprom::ET_EQ_REG,
        "et_batt_service" => OffsetsEeprom::ET_BATT_SERVICE,
        "ev_tempcomp" => OffsetsEeprom::EV_TEMPCOMP,
        "ev_hvd" => OffsetsEeprom::EV_HVD,
        "ev_hvr" => OffsetsEeprom::EV_HVR,
        "evb_ref_lim" => OffsetsEeprom::EVB_REF_LIM,
        "etb_max" => OffsetsEeprom::ETB_MAX,
        "etb_min" => OffsetsEeprom::ETB_MIN,
        "ev_soc_g_gy" => OffsetsEeprom::EV_SOC_G_GY,
        "ev_soc_gy_y" => OffsetsEeprom::EV_SOC_GY_Y,
        "ev_soc_y_yr" => OffsetsEeprom::EV_SOC_Y_YR,
        "ev_soc_yr_r" => OffsetsEeprom::EV_SOC_YR_R,
        "emodbus_id" => OffsetsEeprom::EMODBUS_ID,
        "emeterbus_id" => OffsetsEeprom::EMETERBUS_ID,
        "eib_lim" => OffsetsEeprom::EIB_LIM,
        "eva_ref_fixed_init" => OffsetsEeprom::EVA_REF_FIXED_INIT,
        "eva_ref_fixed_pct_init" => OffsetsEeprom::EVA_REF_FIXED_PCT_INIT,
        &_ => todo!(),
    }
}
