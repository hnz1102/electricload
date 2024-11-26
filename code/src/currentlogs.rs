// CurrentLogs
// CurrentLogs is a module to record the current, voltage, power, battery, temperature, rpm, and pwm.
// It is used to record the data for the electric load.
// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Hiroshi Nakajima

use log::*;

pub struct CurrentLog {
    pub voltage: f32,
    pub current: f32,
    pub power: f32,
    pub clock: u128,
    pub battery: f32,
    pub temp: f32,
    pub rpm: u32,
    pub pwm: u32,
}

impl CurrentLog {
    pub fn default() -> Self {
        CurrentLog { 
            voltage: 0.0,
            current: 0.0,
            power: 0.0,
            clock: 0,
            battery: 0.0,
            temp: 0.0,
            rpm: 0,
            pwm: 0,
         }
    }
}


pub struct CurrentRecord {
    rec: Vec<CurrentLog>,
}

#[allow(dead_code)]
impl CurrentRecord {
    pub fn new() -> CurrentRecord {
        CurrentRecord { rec: Vec::new() }
    }

    pub fn record(&mut self, data: CurrentLog)
    {
        self.rec.push(data);
    }

    pub fn dump(&self)
    {
        info!("time,voltage,current,power,battery,temp,rpm,pwm");
        for it in &self.rec {
           info!("{},{},{},{},{},{},{},{}", it.clock, it.voltage, it.current, it.power, it.battery, it.temp, it.rpm, it.pwm);
        } 
    }

    pub fn clear(&mut self)
    {
        self.rec.clear()
    }

    pub fn get_size(&self) -> usize {
        self.rec.len()    
    }

    pub fn get_all_data(&self) -> &Vec<CurrentLog> {
        &self.rec
    }

    pub fn remove_data(&mut self, size : usize){
        let mut num = size;
        if self.rec.len() < size {
            num = self.rec.len();
        }       
        let _ = &self.rec.drain(0..num);
    }

}

