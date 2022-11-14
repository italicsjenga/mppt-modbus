#![allow(incomplete_features)]
#![allow(dead_code)]
#![feature(adt_const_params)]
#![feature(generic_arg_infer)]
#![feature(generic_const_exprs)]

use std::{path::Path, process::Command};

use clap::{AppSettings, Parser, Subcommand};
use libmodbus_rs::{Modbus, ModbusClient, ModbusRTU};

mod offsets;
use crate::offsets::{OffsetsEeprom, OffsetsRam};

const DEVICE_ID: u8 = 0x01;
const RAM_DATA_SIZE: u16 = 0x005B;
const EEPROM_BEGIN: u16 = 0xE000;
const EEPROM_DATA_SIZE: u16 = 0x0021;
// const EEPROM_DATA_SIZE: u16 = 0xE0CD - EEPROM_BEGIN;

#[derive(Debug)]
struct MpptRam {
    // scaling values
    v_pu: f32,
    i_pu: f32,
    ver_sw: u16,
    // filtered ADC
    adc_vb_f_med: f32,
    adc_vbterm_f: f32,
    adc_vbs_f: f32,
    adc_va_f: f32,
    adc_ib_f_shadow: f32,
    adc_ia_f_shadow: f32,
    adc_p12_f: f32,
    adc_p3_f: f32,
    adc_pmeter_f: f32,
    adc_p18_f: f32,
    adc_v_ref: f32,
    // temperatures
    t_hs: u16,
    t_rts: u16,
    t_batt: u16,
    // status
    adc_vb_f_1m: f32,
    adc_ib_f_1m: f32,
    vb_min: f32,
    vb_max: f32,
    hourmeter_hi: u16,
    hourmeter_lo: u16,
    fault_all: u16,
    alarm_hi: u16,
    alarm_lo: u16,
    dip_all: u16,
    led_state: u16,
    // charger
    charge_state: u16,
    vb_ref: f32,
    ahc_r_hi: u16,
    ahc_r_lo: u16,
    ahc_t_hi: u16,
    ahc_t_lo: u16,
    kwhc_r: u16,
    kwhc_t: u16,
    // MpptRam
    power_out_shadow: f32,
    power_in_shadow: f32,
    sweep_pin_max: f32,
    sweep_vmp: f32,
    sweep_voc: f32,
    // logger - today's values
    vb_min_daily: f32,
    vb_max_daily: f32,
    va_max_daily: f32,
    ahc_daily: f32,
    whc_daily: u16,
    flags_daily: u16,
    pout_max_daily: f32,
    tb_min_daily: u16,
    tb_max_daily: u16,
    fault_daily: u16,
    alarm_daily_hi: u16,
    alarm_daily_lo: u16,
    time_ab_daily: u16,
    time_eq_daily: u16,
    time_fl_daily: u16,
    // manual control
    ib_ref_slave: f32,
    vb_ref_slave: f32,
    va_ref_fixed: f32,
    va_ref_fixed_pct: f32,
}

#[derive(Debug)]
struct MpptEeprom {
    ev_absorp: Datapoint<{ Datatype::Voltage }>,
    ev_float: Datapoint<{ Datatype::Voltage }>,
    et_absorp: Datapoint<{ Datatype::Raw }>,
    et_absorp_ext: Datapoint<{ Datatype::Raw }>,
    ev_absorp_ext: Datapoint<{ Datatype::Voltage }>,
    ev_float_cancel: Datapoint<{ Datatype::Voltage }>,
    et_float_exit_cum: Datapoint<{ Datatype::Raw }>,
    ev_eq: Datapoint<{ Datatype::Voltage }>,
    et_eqcalendar: Datapoint<{ Datatype::Raw }>,
    et_eq_above: Datapoint<{ Datatype::Raw }>,
    et_eq_reg: Datapoint<{ Datatype::Raw }>,
    et_batt_service: Datapoint<{ Datatype::Raw }>,
    ev_tempcomp: Datapoint<{ Datatype::Tempcomp }>,
    ev_hvd: Datapoint<{ Datatype::Voltage }>,
    ev_hvr: Datapoint<{ Datatype::Voltage }>,
    evb_ref_lim: Datapoint<{ Datatype::Voltage }>,
    etb_max: Datapoint<{ Datatype::Raw }>,
    etb_min: Datapoint<{ Datatype::Raw }>,
    ev_soc_g_gy: Datapoint<{ Datatype::Voltage }>,
    ev_soc_gy_y: Datapoint<{ Datatype::Voltage }>,
    ev_soc_y_yr: Datapoint<{ Datatype::Voltage }>,
    ev_soc_yr_r: Datapoint<{ Datatype::Voltage }>,
    emodbus_id: Datapoint<{ Datatype::Raw }>,
    emeterbus_id: Datapoint<{ Datatype::Raw }>,
    eib_lim: Datapoint<{ Datatype::Current }>,
    eva_ref_fixed_init: Datapoint<{ Datatype::Voltage }>,
    eva_ref_fixed_pct_init: Datapoint<{ Datatype::VoltagePercentage }>,
}

#[derive(Debug, PartialEq, Eq)]
enum Datatype {
    Voltage,
    VoltagePercentage,
    Tempcomp,
    Current,
    Raw,
}

#[derive(Debug)]
struct Datapoint<const T: Datatype> {
    data: u16,
}

impl<const T: Datatype> Datapoint<T> {
    fn get_type(&self) -> Datatype {
        T
    }

    fn get_scaled(&self, info: &Info) -> f32 {
        match T {
            Datatype::Voltage => self.data as f32 * info.v_scale * f32::powf(2., -15.),
            Datatype::VoltagePercentage => self.data as f32 * 100. * f32::powf(2., -16.),
            Datatype::Tempcomp => self.data as f32 * info.v_scale * f32::powf(2., -16.),
            Datatype::Current => self.data as f32 * info.i_scale * f32::powf(2., -15.),
            Datatype::Raw => self.data as f32,
        }
    }

    fn get_val(dt: Datatype, input: f32, info: &Info) -> u16 {
        match dt {
            Datatype::Voltage => ((input / f32::powf(2., -15.)) / info.v_scale) as u16,
            Datatype::VoltagePercentage => ((input / f32::powf(2., -16.)) / 100.) as u16,
            Datatype::Tempcomp => ((input / f32::powf(2., -16.)) / info.v_scale) as u16,
            Datatype::Current => ((input / f32::powf(2., -15.)) / info.i_scale) as u16,
            Datatype::Raw => input as u16,
        }
    }

    fn from_u16(d: u16) -> Self {
        Self { data: d }
    }
}

#[derive(Parser)]
#[clap(global_setting = AppSettings::DisableHelpSubcommand)]
#[clap(author, about, long_about = None)]
struct Args {
    /// Serial port to connect to MPPT
    #[clap(short, long, default_value = "/dev/ttyUSB0")]
    serial_port: String,

    /// list serial ports on this system
    #[clap(long)]
    get_serial_ports: bool,

    #[clap(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand)]
enum Commands {
    /// Get single EEPROM value
    Get { name: String },

    /// Set single EEPROM value
    Set { name: String },
}

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
    let baud = 9600;
    let parity = 'N';
    let data_bit = 8;
    let stop_bit = 2;

    if !Path::new(&args.serial_port).exists() {
        println!(
            "Serial port {} does not exist\nTry \"mppt-control --help\" for usage instructions",
            args.serial_port
        );
        // return;
    }

    let eeprom_data = MpptEeprom {
        ev_absorp: Datapoint::from_u16(0x0000),
        ev_float: Datapoint::from_u16(0x0000),
        et_absorp: Datapoint::from_u16(0x0000),
        et_absorp_ext: Datapoint::from_u16(0x0000),
        ev_absorp_ext: Datapoint::from_u16(0x0000),
        ev_float_cancel: Datapoint::from_u16(0x0000),
        et_float_exit_cum: Datapoint::from_u16(0x0000),
        ev_eq: Datapoint::from_u16(0x0000),
        et_eqcalendar: Datapoint::from_u16(0x0000),
        et_eq_above: Datapoint::from_u16(0x0000),
        et_eq_reg: Datapoint::from_u16(0x0000),
        et_batt_service: Datapoint::from_u16(0x0000),
        ev_tempcomp: Datapoint::from_u16(0x0000),
        ev_hvd: Datapoint::from_u16(0x0000),
        ev_hvr: Datapoint::from_u16(0x0000),
        evb_ref_lim: Datapoint::from_u16(0x0000),
        etb_max: Datapoint::from_u16(0x0000),
        etb_min: Datapoint::from_u16(0x0000),
        ev_soc_g_gy: Datapoint::from_u16(0x0000),
        ev_soc_gy_y: Datapoint::from_u16(0x0000),
        ev_soc_y_yr: Datapoint::from_u16(0x0000),
        ev_soc_yr_r: Datapoint::from_u16(0x0000),
        emodbus_id: Datapoint::from_u16(0x0000),
        emeterbus_id: Datapoint::from_u16(0x0000),
        eib_lim: Datapoint::from_u16(0x0000),
        eva_ref_fixed_init: Datapoint::from_u16(0x0000),
        eva_ref_fixed_pct_init: Datapoint::from_u16(0x0000),
    };

    match args.command {
        Some(Commands::Get { name }) => {
            // println!(
            //     "{}: {:#?}",
            //     name.to_lowercase(),
            //     match_datapoint(name.as_str(), &eeprom_data).get_scaled(&Info {
            //         v_scale: 1.,
            //         i_scale: 1.
            //     })
            // );
            let t = match_datapoint_type(
                name.as_str(),
                &eeprom_data,
                &Info {
                    v_scale: 1.,
                    i_scale: 1.,
                },
            );
            println!("{}: {} - {:?}", name, t.val, t.dt);
            return;
        }
        Some(Commands::Set { name }) => {
            println!("set var {}", name);
            let t = match_datapoint_type(
                name.as_str(),
                &eeprom_data,
                &Info {
                    v_scale: 1.,
                    i_scale: 1.,
                },
            );
            println!("type: {:?}", t);
            // let new: Datapoint<{ t }> = Datapoint::from_u16(0x0000);
            // let b: Datapoint<_> = Datapoint::<{ t }>::from_u16(0x0000);
            return;
        }
        None => {
            // println!("ram: {:#?}", ram_data);
            println!("eeprom: {:#?}", eeprom_data);
        }
    }
    println!("Connecting to device on {}", args.serial_port);
    let mut modbus = Modbus::new_rtu(&args.serial_port, baud, parity, data_bit, stop_bit)
        .expect("Could not create modbus device");
    modbus
        .set_slave(DEVICE_ID)
        .expect("Could not set client device");
    modbus
        .connect()
        .expect("Could not connect to client device");
    let mut data_in: [u16; RAM_DATA_SIZE as usize + 1] = [0; RAM_DATA_SIZE as usize + 1];
    modbus
        .read_registers(0x0000, RAM_DATA_SIZE + 1, &mut data_in)
        .expect("couldnt");
    let info = Info::from(&data_in);
    let ram_data = MpptRam {
        v_pu: info.scale_voltage_f(
            data_in[OffsetsRam::V_PU_HI] as f32
                + (data_in[OffsetsRam::V_PU_LO] as f32 / f32::powf(2., 16.)),
        ),
        i_pu: info.scale_current_f(
            data_in[OffsetsRam::I_PU_HI] as f32
                + (data_in[OffsetsRam::I_PU_LO] as f32 / f32::powf(2., 16.)),
        ),
        ver_sw: data_in[OffsetsRam::VER_SW],
        adc_vb_f_med: info.scale_voltage(&data_in[OffsetsRam::ADC_VB_F_MED]),
        adc_vbterm_f: info.scale_voltage(&data_in[OffsetsRam::ADC_VBTERM_F]),
        adc_vbs_f: info.scale_voltage(&data_in[OffsetsRam::ADC_VBS_F]),
        adc_va_f: info.scale_voltage(&data_in[OffsetsRam::ADC_VA_F]),
        adc_ib_f_shadow: info.scale_current(&data_in[OffsetsRam::ADC_IB_F_SHADOW]),
        adc_ia_f_shadow: info.scale_current(&data_in[OffsetsRam::ADC_IA_F_SHADOW]),
        adc_p12_f: data_in[OffsetsRam::ADC_P12_F] as f32 * 18.618 * f32::powf(2., -15.),
        adc_p3_f: data_in[OffsetsRam::ADC_P3_F] as f32 * 6.6 * f32::powf(2., -15.),
        adc_pmeter_f: data_in[OffsetsRam::ADC_PMETER_F] as f32 * 18.618 * f32::powf(2., -15.),
        adc_p18_f: data_in[OffsetsRam::ADC_P18_F] as f32 * 3. * f32::powf(2., -15.),
        adc_v_ref: data_in[OffsetsRam::ADC_V_REF] as f32 * 3. * f32::powf(2., -15.),
        t_hs: data_in[OffsetsRam::T_HS],
        t_rts: data_in[OffsetsRam::T_RTS],
        t_batt: data_in[OffsetsRam::T_BATT],
        adc_vb_f_1m: info.scale_voltage(&data_in[OffsetsRam::ADC_VB_F_1M]),
        adc_ib_f_1m: info.scale_current(&data_in[OffsetsRam::ADC_IB_F_1M]),
        vb_min: info.scale_voltage(&data_in[OffsetsRam::VB_MIN]),
        vb_max: info.scale_voltage(&data_in[OffsetsRam::VB_MAX]),
        hourmeter_hi: data_in[OffsetsRam::HOURMETER_HI],
        hourmeter_lo: data_in[OffsetsRam::HOURMETER_LO],
        fault_all: data_in[OffsetsRam::FAULT_ALL],
        alarm_hi: data_in[OffsetsRam::ALARM_HI],
        alarm_lo: data_in[OffsetsRam::ALARM_LO],
        dip_all: data_in[OffsetsRam::DIP_ALL],
        led_state: data_in[OffsetsRam::LED_STATE],
        charge_state: data_in[OffsetsRam::CHARGE_STATE],
        vb_ref: info.scale_voltage(&data_in[OffsetsRam::VB_REF]),
        ahc_r_hi: data_in[OffsetsRam::AHC_R_HI],
        ahc_r_lo: data_in[OffsetsRam::AHC_R_LO],
        ahc_t_hi: data_in[OffsetsRam::AHC_T_HI],
        ahc_t_lo: data_in[OffsetsRam::AHC_T_LO],
        kwhc_r: data_in[OffsetsRam::KWHC_R],
        kwhc_t: data_in[OffsetsRam::KWHC_T],
        power_out_shadow: info.scale_power(&data_in[OffsetsRam::POWER_OUT_SHADOW]),
        power_in_shadow: info.scale_power(&data_in[OffsetsRam::POWER_IN_SHADOW]),
        sweep_pin_max: info.scale_power(&data_in[OffsetsRam::SWEEP_PIN_MAX]),
        sweep_vmp: info.scale_voltage(&data_in[OffsetsRam::SWEEP_VMP]),
        sweep_voc: info.scale_voltage(&data_in[OffsetsRam::SWEEP_VOC]),
        vb_min_daily: info.scale_voltage(&data_in[OffsetsRam::VB_MIN_DAILY]),
        vb_max_daily: info.scale_voltage(&data_in[OffsetsRam::VB_MAX_DAILY]),
        va_max_daily: info.scale_voltage(&data_in[OffsetsRam::VA_MAX_DAILY]),
        ahc_daily: data_in[OffsetsRam::AHC_DAILY] as f32 * 0.1,
        whc_daily: data_in[OffsetsRam::WHC_DAILY],
        flags_daily: data_in[OffsetsRam::FLAGS_DAILY],
        pout_max_daily: info.scale_power(&data_in[OffsetsRam::POUT_MAX_DAILY]),
        tb_min_daily: data_in[OffsetsRam::TB_MIN_DAILY],
        tb_max_daily: data_in[OffsetsRam::TB_MAX_DAILY],
        fault_daily: data_in[OffsetsRam::FAULT_DAILY],
        alarm_daily_hi: data_in[OffsetsRam::ALARM_DAILY_HI],
        alarm_daily_lo: data_in[OffsetsRam::ALARM_DAILY_LO],
        time_ab_daily: data_in[OffsetsRam::TIME_AB_DAILY],
        time_eq_daily: data_in[OffsetsRam::TIME_EQ_DAILY],
        time_fl_daily: data_in[OffsetsRam::TIME_FL_DAILY],
        ib_ref_slave: data_in[OffsetsRam::IB_REF_SLAVE] as f32 * 80. * f32::powf(2., -15.),
        vb_ref_slave: info.scale_voltage(&data_in[OffsetsRam::VB_REF_SLAVE]),
        va_ref_fixed: info.scale_voltage(&data_in[OffsetsRam::VA_REF_FIXED]),
        va_ref_fixed_pct: data_in[OffsetsRam::VA_REF_FIXED_PCT] as f32 * 100. * f32::powf(2., -16.),
    };

    let mut data_in: [u16; EEPROM_DATA_SIZE as usize + 1] = [0; EEPROM_DATA_SIZE as usize + 1];
    modbus
        .read_registers(EEPROM_BEGIN, EEPROM_DATA_SIZE + 1, &mut data_in)
        .expect("could not get eeprom data");

    let eeprom_data = MpptEeprom {
        ev_absorp: Datapoint::from_u16(data_in[OffsetsEeprom::EV_ABSORP]),
        ev_float: Datapoint::from_u16(data_in[OffsetsEeprom::EV_FLOAT]),
        et_absorp: Datapoint::from_u16(data_in[OffsetsEeprom::ET_ABSORP]),
        et_absorp_ext: Datapoint::from_u16(data_in[OffsetsEeprom::ET_ABSORP_EXT]),
        ev_absorp_ext: Datapoint::from_u16(data_in[OffsetsEeprom::EV_ABSORP_EXT]),
        ev_float_cancel: Datapoint::from_u16(data_in[OffsetsEeprom::EV_FLOAT_CANCEL]),
        et_float_exit_cum: Datapoint::from_u16(data_in[OffsetsEeprom::ET_FLOAT_EXIT_CUM]),
        ev_eq: Datapoint::from_u16(data_in[OffsetsEeprom::EV_EQ]),
        et_eqcalendar: Datapoint::from_u16(data_in[OffsetsEeprom::ET_EQCALENDAR]),
        et_eq_above: Datapoint::from_u16(data_in[OffsetsEeprom::ET_EQ_ABOVE]),
        et_eq_reg: Datapoint::from_u16(data_in[OffsetsEeprom::ET_EQ_REG]),
        et_batt_service: Datapoint::from_u16(data_in[OffsetsEeprom::ET_BATT_SERVICE]),
        ev_tempcomp: Datapoint::from_u16(data_in[OffsetsEeprom::EV_TEMPCOMP]),
        ev_hvd: Datapoint::from_u16(data_in[OffsetsEeprom::EV_HVD]),
        ev_hvr: Datapoint::from_u16(data_in[OffsetsEeprom::EV_HVR]),
        evb_ref_lim: Datapoint::from_u16(data_in[OffsetsEeprom::EVB_REF_LIM]),
        etb_max: Datapoint::from_u16(data_in[OffsetsEeprom::ETB_MAX]),
        etb_min: Datapoint::from_u16(data_in[OffsetsEeprom::ETB_MIN]),
        ev_soc_g_gy: Datapoint::from_u16(data_in[OffsetsEeprom::EV_SOC_G_GY]),
        ev_soc_gy_y: Datapoint::from_u16(data_in[OffsetsEeprom::EV_SOC_GY_Y]),
        ev_soc_y_yr: Datapoint::from_u16(data_in[OffsetsEeprom::EV_SOC_Y_YR]),
        ev_soc_yr_r: Datapoint::from_u16(data_in[OffsetsEeprom::EV_SOC_YR_R]),
        emodbus_id: Datapoint::from_u16(data_in[OffsetsEeprom::EMODBUS_ID]),
        emeterbus_id: Datapoint::from_u16(data_in[OffsetsEeprom::EMETERBUS_ID]),
        eib_lim: Datapoint::from_u16(data_in[OffsetsEeprom::EIB_LIM]),
        eva_ref_fixed_init: Datapoint::from_u16(data_in[OffsetsEeprom::EVA_REF_FIXED_INIT]),
        eva_ref_fixed_pct_init: Datapoint::from_u16(data_in[OffsetsEeprom::EVA_REF_FIXED_PCT_INIT]),
    };

    match args.command {
        Some(Commands::Get { name }) => {
            println!("get var {}", name);
            // println!(
            //     "pp: {:#?}",
            //     match_datapoint(name.as_str(), &eeprom_data).get_scaled(&info)
            // );
            return;
        }
        Some(Commands::Set { name }) => {
            println!("set var {}", name);
            return;
        }
        None => {
            println!("ram: {:#?}", ram_data);
            println!("eeprom: {:#?}", eeprom_data);
        }
    }

    let value = 50.;
    let _value_scaled = ((value / info.v_scale) / f32::powf(2., -15.)) as u16;
    // modbus
    //     .write_register(EEPROM_BEGIN as u16 + OffsetsEeprom::EV_soc_g_gy as u16, value_scaled)
    //     .expect("could not set value");
}

// fn match_datapoint(name: &str, data: &MpptEeprom) -> Datapoint<{ t }> {
//     match name.to_lowercase().as_str() {
//         "ev_absorp" => data.ev_absorp,
//         "ev_float" => data.ev_float,
//         "et_absorp" => data.et_absorp,
//         "et_absorp_ext" => data.et_absorp_ext,
//         "ev_absorp_ext" => data.ev_absorp_ext,
//         "ev_float_cancel" => data.ev_float_cancel,
//         "et_float_exit_cum" => data.et_float_exit_cum,
//         "ev_eq" => data.ev_eq,
//         "et_eqcalendar" => data.et_eqcalendar,
//         "et_eq_above" => data.et_eq_above,
//         "et_eq_reg" => data.et_eq_reg,
//         "et_batt_service" => data.et_batt_service,
//         "ev_tempcomp" => data.ev_tempcomp,
//         "ev_hvd" => data.ev_hvd,
//         "ev_hvr" => data.ev_hvr,
//         "evb_ref_lim" => data.evb_ref_lim,
//         "etb_max" => data.etb_max,
//         "etb_min" => data.etb_min,
//         "ev_soc_g_gy" => data.ev_soc_g_gy,
//         "ev_soc_gy_y" => data.ev_soc_gy_y,
//         "ev_soc_y_yr" => data.ev_soc_y_yr,
//         "ev_soc_yr_r" => data.ev_soc_yr_r,
//         "emodbus_id" => data.emodbus_id,
//         "emeterbus_id" => data.emeterbus_id,
//         "eib_lim" => data.eib_lim,
//         "eva_ref_fixed_init" => data.eva_ref_fixed_init,
//         "eva_ref_fixed_pct_init" => data.eva_ref_fixed_pct_init,
//         &_ => todo!(),
//     }
// }

#[derive(Debug)]
struct DatapointDisplay {
    dt: Datatype,
    val: f32,
}

fn match_datapoint_type(name: &str, data: &MpptEeprom, info: &Info) -> DatapointDisplay {
    // let a = data.ev_absorp;
    let t = match name.to_lowercase().as_str() {
        "ev_absorp" => data.ev_absorp.get_type(),
        "ev_float" => data.ev_float.get_type(),
        "et_absorp" => data.et_absorp.get_type(),
        "et_absorp_ext" => data.et_absorp_ext.get_type(),
        "ev_absorp_ext" => data.ev_absorp_ext.get_type(),
        "ev_float_cancel" => data.ev_float_cancel.get_type(),
        "et_float_exit_cum" => data.et_float_exit_cum.get_type(),
        "ev_eq" => data.ev_eq.get_type(),
        "et_eqcalendar" => data.et_eqcalendar.get_type(),
        "et_eq_above" => data.et_eq_above.get_type(),
        "et_eq_reg" => data.et_eq_reg.get_type(),
        "et_batt_service" => data.et_batt_service.get_type(),
        "ev_tempcomp" => data.ev_tempcomp.get_type(),
        "ev_hvd" => data.ev_hvd.get_type(),
        "ev_hvr" => data.ev_hvr.get_type(),
        "evb_ref_lim" => data.evb_ref_lim.get_type(),
        "etb_max" => data.etb_max.get_type(),
        "etb_min" => data.etb_min.get_type(),
        "ev_soc_g_gy" => data.ev_soc_g_gy.get_type(),
        "ev_soc_gy_y" => data.ev_soc_gy_y.get_type(),
        "ev_soc_y_yr" => data.ev_soc_y_yr.get_type(),
        "ev_soc_yr_r" => data.ev_soc_yr_r.get_type(),
        "emodbus_id" => data.emodbus_id.get_type(),
        "emeterbus_id" => data.emeterbus_id.get_type(),
        "eib_lim" => data.eib_lim.get_type(),
        "eva_ref_fixed_init" => data.eva_ref_fixed_init.get_type(),
        "eva_ref_fixed_pct_init" => data.eva_ref_fixed_pct_init.get_type(),
        &_ => todo!(),
    };
    let v = match name.to_lowercase().as_str() {
        "ev_absorp" => data.ev_absorp.get_scaled(info),
        "ev_float" => data.ev_float.get_scaled(info),
        "et_absorp" => data.et_absorp.get_scaled(info),
        "et_absorp_ext" => data.et_absorp_ext.get_scaled(info),
        "ev_absorp_ext" => data.ev_absorp_ext.get_scaled(info),
        "ev_float_cancel" => data.ev_float_cancel.get_scaled(info),
        "et_float_exit_cum" => data.et_float_exit_cum.get_scaled(info),
        "ev_eq" => data.ev_eq.get_scaled(info),
        "et_eqcalendar" => data.et_eqcalendar.get_scaled(info),
        "et_eq_above" => data.et_eq_above.get_scaled(info),
        "et_eq_reg" => data.et_eq_reg.get_scaled(info),
        "et_batt_service" => data.et_batt_service.get_scaled(info),
        "ev_tempcomp" => data.ev_tempcomp.get_scaled(info),
        "ev_hvd" => data.ev_hvd.get_scaled(info),
        "ev_hvr" => data.ev_hvr.get_scaled(info),
        "evb_ref_lim" => data.evb_ref_lim.get_scaled(info),
        "etb_max" => data.etb_max.get_scaled(info),
        "etb_min" => data.etb_min.get_scaled(info),
        "ev_soc_g_gy" => data.ev_soc_g_gy.get_scaled(info),
        "ev_soc_gy_y" => data.ev_soc_gy_y.get_scaled(info),
        "ev_soc_y_yr" => data.ev_soc_y_yr.get_scaled(info),
        "ev_soc_yr_r" => data.ev_soc_yr_r.get_scaled(info),
        "emodbus_id" => data.emodbus_id.get_scaled(info),
        "emeterbus_id" => data.emeterbus_id.get_scaled(info),
        "eib_lim" => data.eib_lim.get_scaled(info),
        "eva_ref_fixed_init" => data.eva_ref_fixed_init.get_scaled(info),
        "eva_ref_fixed_pct_init" => data.eva_ref_fixed_pct_init.get_scaled(info),
        &_ => todo!(),
    };
    DatapointDisplay { dt: t, val: v }
}
