use clap::Parser;
use libmodbus_rs::{Modbus, ModbusClient, ModbusRTU};

const DEVICE_ID: u8 = 0x01;

struct Offsets {}

impl Offsets {
    // scaling values
    const V_PU_HI: u16 = 0x0000;
    const V_PU_LO: u16 = 0x0001;
    const I_PU_HI: u16 = 0x0002;
    const I_PU_LO: u16 = 0x0003;
    const VER_SW: u16 = 0x0004;
    // filtered ADC
    const ADC_VB_F_MED: u16 = 0x0018;
    const ADC_VBTERM_F: u16 = 0x0019;
    const ADC_VBS_F: u16 = 0x001A;
    const ADC_VA_F: u16 = 0x001B;
    const ADC_IB_F_SHADOW: u16 = 0x001C;
    const ADC_IA_F_SHADOW: u16 = 0x001D;
    const ADC_P12_F: u16 = 0x001E;
    const ADC_P3_F: u16 = 0x001F;
    const ADC_PMETER_F: u16 = 0x0020;
    const ADC_P18_F: u16 = 0x0021;
    const ADC_V_REF: u16 = 0x0022;
    // temperatures
    const T_HS: u16 = 0x0023;
    const T_RTS: u16 = 0x0024;
    const T_BATT: u16 = 0x0025;
    // status
    const ADC_VB_F_1M: u16 = 0x0026;
    const ADC_IB_F_1M: u16 = 0x0027;
    const VB_MIN: u16 = 0x0028;
    const VB_MAX: u16 = 0x0029;
    const HOURMETER_HI: u16 = 0x002A;
    const HOURMETER_LO: u16 = 0x002B;
    const FAULT_ALL: u16 = 0x002C;
    const ALARM_HI: u16 = 0x002E;
    const ALARM_LO: u16 = 0x002F;
    const DIP_ALL: u16 = 0x0030;
    const LED_STATE: u16 = 0x0031;
    // charger
    const CHARGE_STATE: u16 = 0x0032;
    const VB_REF: u16 = 0x0033;
    const AHC_R_HI: u16 = 0x0034;
    const AHC_R_LO: u16 = 0x0035;
    const AHC_T_HI: u16 = 0x0036;
    const AHC_T_LO: u16 = 0x0037;
    const KWHC_R: u16 = 0x0038;
    const KWHC_T: u16 = 0x0039;
    // MPPT
    const POWER_OUT_SHADOW: u16 = 0x003A;
    const POWER_IN_SHADOW: u16 = 0x003B;
    const SWEEP_PIN_MAX: u16 = 0x003C;
    const SWEEP_VMP: u16 = 0x003D;
    const SWEEP_VOC: u16 = 0x003E;
    // logger - today's values
    const VB_MIN_DAILY: u16 = 0x0040;
    const VB_MAX_DAILY: u16 = 0x0041;
    const VA_MAX_DAILY: u16 = 0x0042;
    const AHC_DAILY: u16 = 0x0043;
    const WHC_DAILY: u16 = 0x0044;
    const FLAGS_DAILY: u16 = 0x0045;
    const POUT_MAX_DAILY: u16 = 0x0046;
    const TB_MIN_DAILY: u16 = 0x0047;
    const TB_MAX_DAILY: u16 = 0x0048;
    const FAULT_DAILY: u16 = 0x0049;
    const ALARM_DAILY_HI: u16 = 0x004B;
    const ALARM_DAILY_LO: u16 = 0x004C;
    const TIME_AB_DAILY: u16 = 0x004D;
    const TIME_EQ_DAILY: u16 = 0x004E;
    const FIME_FL_DAILY: u16 = 0x004F;
    // manual control
    const IB_REF_SLAVE: u16 = 0x0058;
    const VB_REF_SLAVE: u16 = 0x0059;
    const VA_REF_FIXED: u16 = 0x005A;
    const VA_REF_FIXED_PCT: u16 = 0x005B;
}

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    #[clap(short, long, default_value = "/dev/ttyUSB0")]
    serial_port: String,
}

fn main() {
    println!("Hello, world!");
    let args = Args::parse();
    let baud = 9600;
    let parity = 'N';
    let data_bit = 8;
    let stop_bit = 2;
    let mut modbus = Modbus::new_rtu(&args.serial_port, baud, parity, data_bit, stop_bit)
        .expect("could not create modbus device");
    modbus
        .set_slave(DEVICE_ID)
        .expect("could not set client device");
    modbus.connect().expect("could not connect");
    let mut data: [u16; 5] = [0; 5];
    modbus
        .read_registers(0x0000, 5, &mut data)
        .expect("couldnt read registers");
    println!("voltage scaling (whole term): {}", get_floating(data[0]));
    println!(
        "voltage scaling (fractional term): {}",
        get_floating(data[1])
    );
    println!("current scaling (whole term): {}", get_floating(data[2]));
    println!(
        "current scaling (fractional term): {}",
        get_floating(data[3])
    );
    println!("software version: {}", data[4]);
}

fn get_floating(num: u16) -> f32 {
    num as f32 * 100. / 32768.
}
