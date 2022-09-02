use libmodbus_rs::{Modbus, ModbusRTU};

const DEVICE_ID: u8 = 0x01;

fn main() {
    println!("Hello, world!");
    let device = "/dev/ttyUSB0";
    let baud = 9600;
    let parity = 'N';
    let data_bit = 8;
    let stop_bit = 2;
    let modbus = Modbus::new_rtu(device, baud, parity, data_bit, stop_bit)
        .expect("could not create modbus device");
    modbus
        .set_slave(DEVICE_ID)
        .expect("could not set client device");
    modbus.connect().expect("could not connect");
}
