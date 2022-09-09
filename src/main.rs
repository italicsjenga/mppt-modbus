use std::{path::Path, process::Command};

use clap::{AppSettings, Parser, Subcommand};
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

#[derive(Serialize, Deserialize, Debug)]
struct MpptEeprom {
    ev_absorp: Voltage,
    ev_float: Voltage,
    et_absorp: Raw,
    et_absorp_ext: Raw,
    ev_absorp_ext: Voltage,
    ev_float_cancel: Voltage,
    et_float_exit_cum: Raw,
    ev_eq: Voltage,
    et_eqcalendar: Raw,
    et_eq_above: Raw,
    et_eq_reg: Raw,
    et_batt_service: Raw,
    ev_tempcomp: Tempcomp,
    ev_hvd: Voltage,
    ev_hvr: Voltage,
    evb_ref_lim: Voltage,
    etb_max: Raw,
    etb_min: Raw,
    ev_soc_g_gy: Voltage,
    ev_soc_gy_y: Voltage,
    ev_soc_y_yr: Voltage,
    ev_soc_yr_r: Voltage,
    emodbus_id: Raw,
    emeterbus_id: Raw,
    eib_lim: Current,
    eva_ref_fixed_init: Voltage,
    eva_ref_fixed_pct_init: VoltagePercentage,
}

trait Scaled {
    fn get_scaled(&self, info: &Info) -> f32;
    fn new(input: f32, info: &Info) -> Self;
}

#[derive(Serialize, Deserialize, Debug)]
struct Tempcomp {
    data: u16,
}

impl Scaled for Tempcomp {
    fn get_scaled(&self, info: &Info) -> f32 {
        self.data as f32 * info.v_scale * f32::powf(2., -16.)
    }

    fn new(input: f32, info: &Info) -> Self {
        Self {
            data: ((input / f32::powf(2., -16.)) / info.v_scale) as u16,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
struct Voltage {
    data: u16,
}

impl Scaled for Voltage {
    fn get_scaled(&self, info: &Info) -> f32 {
        self.data as f32 * info.v_scale * f32::powf(2., -15.)
    }

    fn new(input: f32, info: &Info) -> Self {
        Self {
            data: ((input / f32::powf(2., -15.)) / info.v_scale) as u16,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
struct VoltagePercentage {
    data: u16,
}

impl Scaled for VoltagePercentage {
    fn get_scaled(&self, _: &Info) -> f32 {
        self.data as f32 * 100. * f32::powf(2., -16.)
    }

    fn new(input: f32, _: &Info) -> Self {
        Self {
            data: ((input / f32::powf(2., -16.)) / 100.) as u16,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
struct Current {
    data: u16,
}

impl Scaled for Current {
    fn get_scaled(&self, info: &Info) -> f32 {
        self.data as f32 * info.i_scale * f32::powf(2., -15.)
    }

    fn new(input: f32, info: &Info) -> Self {
        Self {
            data: ((input / f32::powf(2., -15.)) / info.i_scale) as u16,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
struct Raw {
    data: u16,
}

impl Scaled for Raw {
    fn get_scaled(&self, _: &Info) -> f32 {
        self.data as f32
    }

    fn new(input: f32, _: &Info) -> Self {
        Self { data: input as u16 }
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
    println!("ram: {:#?}", ram_data);

    let mut data_in: [u16; EEPROM_DATA_SIZE as usize + 1] = [0; EEPROM_DATA_SIZE as usize + 1];
    modbus
        .read_registers(EEPROM_BEGIN, EEPROM_DATA_SIZE + 1, &mut data_in)
        .expect("could not get eeprom data");

    let eeprom_data = MpptEeprom {
        ev_absorp: Voltage {
            data: data_in[OffsetsEeprom::EV_ABSORP],
        },
        ev_float: Voltage {
            data: data_in[OffsetsEeprom::EV_FLOAT],
        },
        et_absorp: Raw {
            data: data_in[OffsetsEeprom::ET_ABSORP],
        },
        et_absorp_ext: Raw {
            data: data_in[OffsetsEeprom::ET_ABSORP_EXT],
        },
        ev_absorp_ext: Voltage {
            data: data_in[OffsetsEeprom::EV_ABSORP_EXT],
        },
        ev_float_cancel: Voltage {
            data: data_in[OffsetsEeprom::EV_FLOAT_CANCEL],
        },
        et_float_exit_cum: Raw {
            data: data_in[OffsetsEeprom::ET_FLOAT_EXIT_CUM],
        },
        ev_eq: Voltage {
            data: data_in[OffsetsEeprom::EV_EQ],
        },
        et_eqcalendar: Raw {
            data: data_in[OffsetsEeprom::ET_EQCALENDAR],
        },
        et_eq_above: Raw {
            data: data_in[OffsetsEeprom::ET_EQ_ABOVE],
        },
        et_eq_reg: Raw {
            data: data_in[OffsetsEeprom::ET_EQ_REG],
        },
        et_batt_service: Raw {
            data: data_in[OffsetsEeprom::ET_BATT_SERVICE],
        },
        ev_tempcomp: Tempcomp {
            data: data_in[OffsetsEeprom::EV_TEMPCOMP],
        },
        ev_hvd: Voltage {
            data: data_in[OffsetsEeprom::EV_HVD],
        },
        ev_hvr: Voltage {
            data: data_in[OffsetsEeprom::EV_HVR],
        },
        evb_ref_lim: Voltage {
            data: data_in[OffsetsEeprom::EVB_REF_LIM],
        },
        etb_max: Raw {
            data: data_in[OffsetsEeprom::ETB_MAX],
        },
        etb_min: Raw {
            data: data_in[OffsetsEeprom::ETB_MIN],
        },
        ev_soc_g_gy: Voltage {
            data: data_in[OffsetsEeprom::EV_SOC_G_GY],
        },
        ev_soc_gy_y: Voltage {
            data: data_in[OffsetsEeprom::EV_SOC_GY_Y],
        },
        ev_soc_y_yr: Voltage {
            data: data_in[OffsetsEeprom::EV_SOC_Y_YR],
        },
        ev_soc_yr_r: Voltage {
            data: data_in[OffsetsEeprom::EV_SOC_YR_R],
        },
        emodbus_id: Raw {
            data: data_in[OffsetsEeprom::EMODBUS_ID],
        },
        emeterbus_id: Raw {
            data: data_in[OffsetsEeprom::EMETERBUS_ID],
        },
        eib_lim: Current {
            data: data_in[OffsetsEeprom::EIB_LIM],
        },
        eva_ref_fixed_init: Voltage {
            data: data_in[OffsetsEeprom::EVA_REF_FIXED_INIT],
        },
        eva_ref_fixed_pct_init: VoltagePercentage {
            data: data_in[OffsetsEeprom::EVA_REF_FIXED_PCT_INIT],
        },
    };

    println!("eeprom: {:#?}", eeprom_data);
    let value = 50.;
    let _value_scaled = ((value / info.v_scale) / f32::powf(2., -15.)) as u16;
    // modbus
    //     .write_register(EEPROM_BEGIN as u16 + OffsetsEeprom::EV_soc_g_gy as u16, value_scaled)
    //     .expect("could not set value");
}
