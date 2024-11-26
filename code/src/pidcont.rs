// PID controller implementation
// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Hiroshi Nakajima

#![allow(dead_code, unused_imports)]

use std::time::SystemTime;
use std::time::UNIX_EPOCH;
use log::info;

pub struct PIDController {
    kp: f32,
    ki: f32,
    kd: f32,
    setpoint: f32,
    integral: f32,
    prev_error: f32,
    prev_time: u128,
}

#[allow(dead_code)]
impl PIDController {
    pub fn new(kp: f32, ki: f32, kd: f32, setpoint: f32) -> PIDController {
        PIDController {
            kp: kp,
            ki: ki,
            kd: kd,
            setpoint: setpoint,
            integral: 0.0,
            prev_error: 0.0,
            prev_time: 0,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        let now = SystemTime::now();
        self.prev_time = now.duration_since(UNIX_EPOCH).unwrap().as_nanos();
    }

    pub fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
    }

    pub fn update(&mut self, input: f32) -> f32 {
        let now = SystemTime::now();
        let nano = now.duration_since(UNIX_EPOCH).unwrap().as_nanos();
        let dt = (nano - self.prev_time) as f32 / 1000000000.0;
        let error = self.setpoint - input;
        self.integral += error * dt as f32;
        let derivative = (error - self.prev_error) / dt as f32;
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        self.prev_error = error;
        self.prev_time = nano;
        // info!("PID error: {} dt:{} integral: {} derivative: {} output: {} output_u32: {}", error, dt, self.integral, derivative, output, output_u32);
        output
    }
}