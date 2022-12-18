use crate::INFO_SCALE;
use serde::{Deserialize, Serialize};
use std::fmt::{self, Debug};

pub trait DataPoint {
    fn get_scaled(&self) -> f32;
    fn get_scaled_from(&self, data: u16) -> f32;
    fn get_raw(&self) -> u16;
    fn to_string(&self) -> String;
    fn to_string_v(&self) -> String {
        format!("{}, raw: {}", self.to_string(), self.get_raw())
    }
    fn get_type(&self) -> String;
    fn u16_from_f32(&self, input: f32) -> u16;
    fn from_u16(data: u16) -> Self
    where
        Self: Sized;
}

impl fmt::Debug for dyn DataPoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "{}", self.to_string_v())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct Voltage {
    data: u16,
}

impl DataPoint for Voltage {
    fn get_scaled(&self) -> f32 {
        self.get_scaled_from(self.data)
    }

    fn get_scaled_from(&self, data: u16) -> f32 {
        let info = INFO_SCALE.lock().expect("Can't lock info");
        data as f32 * info.v_scale * f32::powf(2., -15.)
    }

    fn from_u16(data: u16) -> Self {
        Self { data }
    }

    fn get_raw(&self) -> u16 {
        self.data
    }

    fn to_string(&self) -> String {
        format!("{}V", self.get_scaled())
    }

    fn get_type(&self) -> String {
        String::from("Voltage")
    }

    fn u16_from_f32(&self, input: f32) -> u16 {
        let info = INFO_SCALE.lock().expect("Can't lock info");
        ((input / f32::powf(2., -15.)) / info.v_scale) as u16
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct Current {
    data: u16,
}

impl DataPoint for Current {
    fn get_scaled(&self) -> f32 {
        self.get_scaled_from(self.data)
    }

    fn get_scaled_from(&self, data: u16) -> f32 {
        let info = INFO_SCALE.lock().expect("Can't lock info");
        data as f32 * info.i_scale * f32::powf(2., -15.)
    }

    fn from_u16(data: u16) -> Self {
        Self { data }
    }

    fn get_raw(&self) -> u16 {
        self.data
    }

    fn to_string(&self) -> String {
        format!("{}A", self.get_scaled())
    }

    fn get_type(&self) -> String {
        String::from("Current")
    }

    fn u16_from_f32(&self, input: f32) -> u16 {
        let info = INFO_SCALE.lock().expect("Can't lock info");
        ((input / f32::powf(2., -15.)) / info.i_scale) as u16
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct VoltagePercentage {
    data: u16,
}

impl DataPoint for VoltagePercentage {
    fn get_scaled(&self) -> f32 {
        self.get_scaled_from(self.data)
    }

    fn get_scaled_from(&self, data: u16) -> f32 {
        data as f32 * 100. * f32::powf(2., -16.)
    }

    fn from_u16(data: u16) -> Self {
        Self { data }
    }

    fn get_raw(&self) -> u16 {
        self.data
    }

    fn to_string(&self) -> String {
        format!("{}%", self.get_scaled())
    }

    fn get_type(&self) -> String {
        String::from("Voltage Percentage")
    }

    fn u16_from_f32(&self, input: f32) -> u16 {
        ((input / f32::powf(2., -16.)) / 100.) as u16
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct Tempcomp {
    data: u16,
}

impl DataPoint for Tempcomp {
    fn get_scaled(&self) -> f32 {
        self.get_scaled_from(self.data)
    }

    fn get_scaled_from(&self, data: u16) -> f32 {
        let info = INFO_SCALE.lock().expect("Can't lock info");
        data as f32 * info.v_scale * f32::powf(2., -16.)
    }

    fn from_u16(data: u16) -> Self {
        Self { data }
    }

    fn get_raw(&self) -> u16 {
        self.data
    }

    fn to_string(&self) -> String {
        format!("{} - temperature compensation value", self.get_scaled())
    }

    fn get_type(&self) -> String {
        String::from("Temperature Compensation")
    }

    fn u16_from_f32(&self, input: f32) -> u16 {
        let info = INFO_SCALE.lock().expect("Can't lock info");
        ((input / f32::powf(2., -16.)) / info.v_scale) as u16
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct Raw {
    data: u16,
}

impl DataPoint for Raw {
    fn get_scaled(&self) -> f32 {
        self.get_scaled_from(self.data)
    }

    fn get_scaled_from(&self, data: u16) -> f32 {
        data as f32
    }

    fn from_u16(data: u16) -> Self {
        Self { data }
    }

    fn get_raw(&self) -> u16 {
        self.data
    }

    fn to_string(&self) -> String {
        String::from("raw value")
    }

    fn get_type(&self) -> String {
        String::from("Raw Value")
    }

    fn u16_from_f32(&self, input: f32) -> u16 {
        input as u16
    }
}
