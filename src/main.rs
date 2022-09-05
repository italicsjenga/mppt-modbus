use std::{path::Path, process::Command};

use clap::{Parser, Subcommand};
use libmodbus_rs::{Modbus, ModbusClient, ModbusRTU};
use serde::{Deserialize, Serialize};

mod offsets;
use crate::offsets::{OffsetsEeprom, OffsetsRam};

const DEVICE_ID: u8 = 0x01;
const RAM_DATA_SIZE: u16 = 0x005B;
const EEPROM_BEGIN: u16 = 0xE000;
const EEPROM_DATA_SIZE: u16 = 0x0021;
// const EEPROM_DATA_SIZE: u16 = 0xE0CD - EEPROM_BEGIN;

#[derive(Serialize, Deserialize, Debug)]
struct MpptRam {
    // scaling values
    V_PU: f32,
    I_PU: f32,
    VER_SW: u16,
    // filtered ADC
    ADC_VB_F_MED: f32,
    ADC_VBTERM_F: f32,
    ADC_VBS_F: f32,
    ADC_VA_F: f32,
    ADC_IB_F_SHADOW: f32,
    ADC_IA_F_SHADOW: f32,
    ADC_P12_F: f32,
    ADC_P3_F: f32,
    ADC_PMETER_F: f32,
    ADC_P18_F: f32,
    ADC_V_REF: f32,
    // temperatures
    T_HS: u16,
    T_RTS: u16,
    T_BATT: u16,
    // status
    ADC_VB_F_1M: f32,
    ADC_IB_F_1M: f32,
    VB_MIN: f32,
    VB_MAX: f32,
    HOURMETER_HI: u16,
    HOURMETER_LO: u16,
    FAULT_ALL: u16,
    ALARM_HI: u16,
    ALARM_LO: u16,
    DIP_ALL: u16,
    LED_STATE: u16,
    // charger
    CHARGE_STATE: u16,
    VB_REF: f32,
    AHC_R_HI: u16,
    AHC_R_LO: u16,
    AHC_T_HI: u16,
    AHC_T_LO: u16,
    KWHC_R: u16,
    KWHC_T: u16,
    // MpptRam
    POWER_OUT_SHADOW: f32,
    POWER_IN_SHADOW: f32,
    SWEEP_PIN_MAX: f32,
    SWEEP_VMP: f32,
    SWEEP_VOC: f32,
    // logger - today's values
    VB_MIN_DAILY: f32,
    VB_MAX_DAILY: f32,
    VA_MAX_DAILY: f32,
    AHC_DAILY: f32,
    WHC_DAILY: u16,
    FLAGS_DAILY: u16,
    POUT_MAX_DAILY: f32,
    TB_MIN_DAILY: u16,
    TB_MAX_DAILY: u16,
    FAULT_DAILY: u16,
    ALARM_DAILY_HI: u16,
    ALARM_DAILY_LO: u16,
    TIME_AB_DAILY: u16,
    TIME_EQ_DAILY: u16,
    TIME_FL_DAILY: u16,
    // manual control
    IB_REF_SLAVE: f32,
    VB_REF_SLAVE: f32,
    VA_REF_FIXED: f32,
    VA_REF_FIXED_PCT: f32,
}

#[derive(Serialize, Deserialize, Debug)]
struct MpptEeprom {
    EV_absorp: f32,
    EV_float: f32,
    Et_absorp: u16,
    Et_absorp_ext: u16,
    EV_absorp_ext: f32,
    EV_float_cancel: f32,
    Et_float_exit_cum: u16,
    EV_eq: f32,
    Et_eqcalendar: u16,
    Et_eq_above: u16,
    Et_eq_reg: u16,
    Et_batt_service: u16,
    EV_tempcomp: f32,
    EV_hvd: f32,
    EV_hvr: f32,
    Evb_ref_lim: f32,
    ETb_max: u16,
    ETb_min: u16,
    EV_soc_g_gy: f32,
    EV_soc_gy_y: f32,
    EV_soc_y_yr: f32,
    EV_soc_yr_r: f32,
    Emodbus_id: u16,
    Emeterbus_id: u16,
    EIb_lim: f32,
    EVa_ref_fixed_init: f32,
    EVa_ref_fixed_pct_init: f32,
}

#[derive(Parser)]
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
        return;
    }

    match args.command {
        Some(Commands::Get { name }) => {
            println!("get var {}", name);
            return;
        }
        Some(Commands::Set { name }) => {
            println!("set var {}", name);
            return;
        }
        None => (),
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
        V_PU: info.scale_voltage_f(
            data_in[OffsetsRam::V_PU_HI] as f32
                + (data_in[OffsetsRam::V_PU_LO] as f32 / f32::powf(2., 16.)),
        ),
        I_PU: info.scale_current_f(
            data_in[OffsetsRam::I_PU_HI] as f32
                + (data_in[OffsetsRam::I_PU_LO] as f32 / f32::powf(2., 16.)),
        ),
        VER_SW: data_in[OffsetsRam::VER_SW],
        ADC_VB_F_MED: info.scale_voltage(&data_in[OffsetsRam::ADC_VB_F_MED]),
        ADC_VBTERM_F: info.scale_voltage(&data_in[OffsetsRam::ADC_VBTERM_F]),
        ADC_VBS_F: info.scale_voltage(&data_in[OffsetsRam::ADC_VBS_F]),
        ADC_VA_F: info.scale_voltage(&data_in[OffsetsRam::ADC_VA_F]),
        ADC_IB_F_SHADOW: info.scale_current(&data_in[OffsetsRam::ADC_IB_F_SHADOW]),
        ADC_IA_F_SHADOW: info.scale_current(&data_in[OffsetsRam::ADC_IA_F_SHADOW]),
        ADC_P12_F: data_in[OffsetsRam::ADC_P12_F] as f32 * 18.618 * f32::powf(2., -15.),
        ADC_P3_F: data_in[OffsetsRam::ADC_P3_F] as f32 * 6.6 * f32::powf(2., -15.),
        ADC_PMETER_F: data_in[OffsetsRam::ADC_PMETER_F] as f32 * 18.618 * f32::powf(2., -15.),
        ADC_P18_F: data_in[OffsetsRam::ADC_P18_F] as f32 * 3. * f32::powf(2., -15.),
        ADC_V_REF: data_in[OffsetsRam::ADC_V_REF] as f32 * 3. * f32::powf(2., -15.),
        T_HS: data_in[OffsetsRam::T_HS],
        T_RTS: data_in[OffsetsRam::T_RTS],
        T_BATT: data_in[OffsetsRam::T_BATT],
        ADC_VB_F_1M: info.scale_voltage(&data_in[OffsetsRam::ADC_VB_F_1M]),
        ADC_IB_F_1M: info.scale_current(&data_in[OffsetsRam::ADC_IB_F_1M]),
        VB_MIN: info.scale_voltage(&data_in[OffsetsRam::VB_MIN]),
        VB_MAX: info.scale_voltage(&data_in[OffsetsRam::VB_MAX]),
        HOURMETER_HI: data_in[OffsetsRam::HOURMETER_HI],
        HOURMETER_LO: data_in[OffsetsRam::HOURMETER_LO],
        FAULT_ALL: data_in[OffsetsRam::FAULT_ALL],
        ALARM_HI: data_in[OffsetsRam::ALARM_HI],
        ALARM_LO: data_in[OffsetsRam::ALARM_LO],
        DIP_ALL: data_in[OffsetsRam::DIP_ALL],
        LED_STATE: data_in[OffsetsRam::LED_STATE],
        CHARGE_STATE: data_in[OffsetsRam::CHARGE_STATE],
        VB_REF: info.scale_voltage(&data_in[OffsetsRam::VB_REF]),
        AHC_R_HI: data_in[OffsetsRam::AHC_R_HI],
        AHC_R_LO: data_in[OffsetsRam::AHC_R_LO],
        AHC_T_HI: data_in[OffsetsRam::AHC_T_HI],
        AHC_T_LO: data_in[OffsetsRam::AHC_T_LO],
        KWHC_R: data_in[OffsetsRam::KWHC_R],
        KWHC_T: data_in[OffsetsRam::KWHC_T],
        POWER_OUT_SHADOW: info.scale_power(&data_in[OffsetsRam::AHC_R_HI]),
        POWER_IN_SHADOW: info.scale_power(&data_in[OffsetsRam::POWER_IN_SHADOW]),
        SWEEP_PIN_MAX: info.scale_power(&data_in[OffsetsRam::SWEEP_PIN_MAX]),
        SWEEP_VMP: info.scale_voltage(&data_in[OffsetsRam::SWEEP_VMP]),
        SWEEP_VOC: info.scale_voltage(&data_in[OffsetsRam::SWEEP_VOC]),
        VB_MIN_DAILY: info.scale_voltage(&data_in[OffsetsRam::VB_MIN_DAILY]),
        VB_MAX_DAILY: info.scale_voltage(&data_in[OffsetsRam::VB_MAX_DAILY]),
        VA_MAX_DAILY: info.scale_voltage(&data_in[OffsetsRam::VA_MAX_DAILY]),
        AHC_DAILY: data_in[OffsetsRam::AHC_DAILY] as f32 * 0.1,
        WHC_DAILY: data_in[OffsetsRam::WHC_DAILY],
        FLAGS_DAILY: data_in[OffsetsRam::FLAGS_DAILY],
        POUT_MAX_DAILY: info.scale_power(&data_in[OffsetsRam::POUT_MAX_DAILY]),
        TB_MIN_DAILY: data_in[OffsetsRam::TB_MIN_DAILY],
        TB_MAX_DAILY: data_in[OffsetsRam::TB_MAX_DAILY],
        FAULT_DAILY: data_in[OffsetsRam::FAULT_DAILY],
        ALARM_DAILY_HI: data_in[OffsetsRam::ALARM_DAILY_HI],
        ALARM_DAILY_LO: data_in[OffsetsRam::ALARM_DAILY_LO],
        TIME_AB_DAILY: data_in[OffsetsRam::TIME_AB_DAILY],
        TIME_EQ_DAILY: data_in[OffsetsRam::TIME_EQ_DAILY],
        TIME_FL_DAILY: data_in[OffsetsRam::TIME_FL_DAILY],
        IB_REF_SLAVE: data_in[OffsetsRam::IB_REF_SLAVE] as f32 * 80. * f32::powf(2., -15.),
        VB_REF_SLAVE: info.scale_voltage(&data_in[OffsetsRam::VB_REF_SLAVE]),
        VA_REF_FIXED: info.scale_voltage(&data_in[OffsetsRam::VA_REF_FIXED]),
        VA_REF_FIXED_PCT: data_in[OffsetsRam::VA_REF_FIXED_PCT] as f32 * 100. * f32::powf(2., -16.),
    };
    println!("ram: {:#?}", ram_data);

    let mut data_in: [u16; EEPROM_DATA_SIZE as usize + 1] = [0; EEPROM_DATA_SIZE as usize + 1];
    modbus
        .read_registers(EEPROM_BEGIN, EEPROM_DATA_SIZE + 1, &mut data_in)
        .expect("could not get eeprom data");

    let eeprom_data = MpptEeprom {
        EV_absorp: info.scale_voltage(&data_in[OffsetsEeprom::EV_absorp]),
        EV_float: info.scale_voltage(&data_in[OffsetsEeprom::EV_float]),
        Et_absorp: data_in[OffsetsEeprom::Et_absorp],
        Et_absorp_ext: data_in[OffsetsEeprom::Et_absorp_ext],
        EV_absorp_ext: info.scale_voltage(&data_in[OffsetsEeprom::EV_absorp_ext]),
        EV_float_cancel: info.scale_voltage(&data_in[OffsetsEeprom::EV_float_cancel]),
        Et_float_exit_cum: data_in[OffsetsEeprom::Et_float_exit_cum],
        EV_eq: info.scale_voltage(&data_in[OffsetsEeprom::EV_eq]),
        Et_eqcalendar: data_in[OffsetsEeprom::Et_eqcalendar],
        Et_eq_above: data_in[OffsetsEeprom::Et_eq_above],
        Et_eq_reg: data_in[OffsetsEeprom::Et_eq_reg],
        Et_batt_service: data_in[OffsetsEeprom::Et_batt_service],
        EV_tempcomp: data_in[OffsetsEeprom::EV_tempcomp] as f32
            * info.v_scale
            * f32::powf(2., -16.),
        EV_hvd: info.scale_voltage(&data_in[OffsetsEeprom::EV_hvd]),
        EV_hvr: info.scale_voltage(&data_in[OffsetsEeprom::EV_hvr]),
        Evb_ref_lim: info.scale_voltage(&data_in[OffsetsEeprom::Evb_ref_lim]),
        ETb_max: data_in[OffsetsEeprom::ETb_max],
        ETb_min: data_in[OffsetsEeprom::ETb_min],
        EV_soc_g_gy: info.scale_voltage(&data_in[OffsetsEeprom::EV_soc_g_gy]),
        EV_soc_gy_y: info.scale_voltage(&data_in[OffsetsEeprom::EV_soc_gy_y]),
        EV_soc_y_yr: info.scale_voltage(&data_in[OffsetsEeprom::EV_soc_y_yr]),
        EV_soc_yr_r: info.scale_voltage(&data_in[OffsetsEeprom::EV_soc_yr_r]),
        Emodbus_id: data_in[OffsetsEeprom::Emodbus_id],
        Emeterbus_id: data_in[OffsetsEeprom::Emeterbus_id],
        EIb_lim: info.scale_current(&data_in[OffsetsEeprom::EIb_lim]),
        EVa_ref_fixed_init: info.scale_voltage(&data_in[OffsetsEeprom::EVa_ref_fixed_init]),
        EVa_ref_fixed_pct_init: data_in[OffsetsEeprom::EVa_ref_fixed_pct_init] as f32
            * 100.
            * f32::powf(2., -16.),
    };

    println!("eeprom: {:#?}", eeprom_data);
    let value = 50.;
    let value_scaled = ((value / info.v_scale) / f32::powf(2., -15.)) as u16;
    // modbus
    //     .write_register(EEPROM_BEGIN as u16 + OffsetsEeprom::EV_soc_g_gy as u16, value_scaled)
    //     .expect("could not set value");
}
