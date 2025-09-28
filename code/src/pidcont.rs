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
        
        // Initial execution guard
        if self.prev_time == 0 {
            self.prev_time = nano;
            self.prev_error = self.setpoint - input;
            return 0.0;
        }
        
        // Calculate dt in milliseconds (not converted to seconds)
        let dt_ms = (nano - self.prev_time) as f32 / 1000000.0; // Convert nanoseconds to milliseconds
        
        // Guard against abnormal dt values (less than 1ms or more than 10000ms)
        if dt_ms <= 1.0 || dt_ms > 10000.0 || !dt_ms.is_finite() {
            info!("Abnormal dt_ms detected: {} nano: {} prev_time: {}", dt_ms, nano, self.prev_time);
            self.prev_time = nano;
            return 0.0;
        }
        
        let error = self.setpoint - input;
        
        // Update and limit integral term (in milliseconds)
        self.integral += error * dt_ms;
        
        // Prevent integral windup (adjusted for milliseconds)
        let max_integral = if self.ki > 0.0 { 100000.0 / self.ki } else { 100000.0 };
        self.integral = self.integral.clamp(-max_integral, max_integral);
        
        // Reset integral if it becomes infinite
        if !self.integral.is_finite() {
            info!("Integral became infinite, resetting to 0");
            self.integral = 0.0;
        }
        
        let derivative = (error - self.prev_error) / dt_ms;
        // info!("PID input: {} error: {} dt_ms: {} integral: {} derivative: {} nano: {}", 
        //      input, error, dt_ms, self.integral, derivative, nano);
        
        // Limit derivative term if it becomes infinite
        let derivative = if derivative.is_finite() { derivative } else { 0.0 };
        
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        
        // Limit output if it becomes infinite
        let output = if output.is_finite() { 
            output.clamp(-1000.0, 1000.0) 
        } else { 
            info!("Output became infinite, setting to 0");
            0.0 
        };
        
        self.prev_error = error;
        self.prev_time = nano;
        
        // if output > 0.9 || !output.is_finite() || !dt_ms.is_finite() || dt_ms < 5.0 {
        //     info!("PID input: {} error: {} dt_ms: {} integral: {} derivative: {} output: {} nano: {}", 
        //           input, error, dt_ms, self.integral, derivative, output, nano);
        // }
        
        output
    }
}