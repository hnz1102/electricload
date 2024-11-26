// This is electric-load main program for ESP32-S3-WROOM-1-N16R8.
// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Hiroshi Nakajima

use std::{thread, time::Duration};
use esp_idf_hal::{gpio::*, prelude::*, spi, i2c};
use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::peripherals::Peripherals;
use embedded_hal::spi::MODE_0;
use log::*;
use std::time::SystemTime;
use esp_idf_hal::adc::config::Config as AdcConfig;
use esp_idf_hal::adc::AdcChannelDriver;
use esp_idf_hal::adc::AdcDriver;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_svc::sntp::{EspSntp, SyncStatus, SntpConf, OperatingMode, SyncMode};
use chrono::{DateTime, Utc};

mod displayctl;
mod currentlogs;
mod wifi;
mod transfer;
mod touchpad;
mod pidcont;
mod pulscount;

use displayctl::{DisplayPanel, LoggingStatus, WifiStatus};
use currentlogs::{CurrentRecord, CurrentLog};
use transfer::{Transfer, ServerInfo};
use touchpad::{TouchPad, Key};
use pidcont::PIDController;
use pulscount::PulsCountDriver;

#[toml_cfg::toml_config]
pub struct Config {
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_psk: &'static str,
    #[default("")]
    influxdb_server: &'static str,
    #[default("0.00001")]
    pid_kp: &'static str,
    #[default("0.05")]
    pid_ki: &'static str,
    #[default("0.000001")]
    pid_kd: &'static str,
    #[default("4500")]
    pwm_offset: &'static str,
    #[default("11.0")]
    max_current_limit: &'static str,
    #[default("110.0")]
    max_power_limit: &'static str,
    #[default("")]
    influxdb_api_key: &'static str,
    #[default("")]
    influxdb_api: &'static str,
    #[default("")]
    influxdb_measurement: &'static str,
    #[default("")]
    influxdb_tag: &'static str,
}

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    // Initialize nvs
    unsafe {
        esp_idf_sys::nvs_flash_init();
    }
    // Peripherals Initialize
    let peripherals = Peripherals::take().unwrap();
    
    // Load Config
    let max_current_limit = CONFIG.max_current_limit.parse::<f32>().unwrap();
    let max_power_limit = CONFIG.max_power_limit.parse::<f32>().unwrap();
    info!("[Limit] Current: {}A  Power: {}W", max_current_limit, max_power_limit);
    let server_info = ServerInfo::new(CONFIG.influxdb_server.to_string(), 
        CONFIG.influxdb_api_key.to_string(),
        CONFIG.influxdb_api.to_string(),
        CONFIG.influxdb_measurement.to_string(),
        CONFIG.influxdb_tag.to_string());

    // Display SPI
    let spi = peripherals.spi2;
    let sclk = peripherals.pins.gpio45;
    let sdo  = peripherals.pins.gpio17;
    let sdi_not_used : Option<Gpio2> = None;
    let cs_not_used : Option<Gpio2> = None;
    let dc = PinDriver::output(peripherals.pins.gpio15)?;
    let rst = PinDriver::output(peripherals.pins.gpio16)?;
    let spi_config = spi::SpiConfig::new().baudrate(1.MHz().into()).data_mode(MODE_0);
    let spi_driver_config = spi::config::DriverConfig::new();

    let spi_driver = spi::SpiDriver::new(
        spi,
        sclk,
        sdo,
        sdi_not_used,
        &spi_driver_config
    ).unwrap();
    
    let spi_device = spi::SpiDeviceDriver::new(spi_driver, cs_not_used, &spi_config)?;
    let mut dp = DisplayPanel::new();
    dp.start(spi_device, dc, rst);

    // Current/Voltage
    let i2c = peripherals.i2c0;
    let scl = peripherals.pins.gpio47;
    let sda = peripherals.pins.gpio21;
    let config = i2c::I2cConfig::new().baudrate(400.kHz().into());
    let mut i2cdrv = i2c::I2cDriver::new(i2c, sda, scl, &config)?;

    // INA228
    // Config
    let mut config_read_buf = [0u8; 2];
    let mut config_write_buf = [0u8; 3];
    // read config
    i2cdrv.write(0x40, &[0x01u8; 1], BLOCK)?;
    i2cdrv.read(0x40, &mut config_read_buf, BLOCK)?;
    config_write_buf[0] = 0x01;
    config_write_buf[1] = config_read_buf[0];
    config_write_buf[2] = (config_read_buf[1] & 0xF8) | 0x02; // 0x02: 16avg, 0x03: 64avg
    i2cdrv.write(0x40, &config_write_buf, BLOCK)?;
    // read back
    i2cdrv.write(0x40, &[0x01u8; 1], BLOCK)?;
    i2cdrv.read(0x40, &mut config_read_buf, BLOCK)?;
    info!("INA228 Config: {:02X}{:02X}", config_read_buf[0], config_read_buf[1]);
    // Calibration
    // SHUNT_CAL
    let current_lsb = 16.384 / 524_288.0;
    let shunt_cal_val = 13107.2 * current_lsb * 1000_000.0 * 0.005;
    let shunt_cal = shunt_cal_val as u32;
    info!("current_lsb={:?} shunt_cal_val={:?} shunt_cal={:?}", current_lsb, shunt_cal_val, shunt_cal);
    let mut shunt_cal_buf = [0u8; 3];
    shunt_cal_buf[0] = 0x02;
    shunt_cal_buf[1] = (shunt_cal >> 8) as u8;
    shunt_cal_buf[2] = (shunt_cal & 0xFF) as u8;
    i2cdrv.write(0x40, &shunt_cal_buf, BLOCK)?;
    // read back
    let mut shunt_cal_read_buf = [0u8; 2];
    i2cdrv.write(0x40, &[0x02u8; 1], BLOCK)?;
    i2cdrv.read(0x40, &mut shunt_cal_read_buf, BLOCK)?;
    if shunt_cal_read_buf[0] != shunt_cal_buf[1] || shunt_cal_read_buf[1] != shunt_cal_buf[2] {
        info!("shunt_cal_write_buf={:?}", shunt_cal_buf);
        info!("shunt_cal_read_buf={:?}", shunt_cal_read_buf);
        info!("Shunt Calibration Error");
    }

    // PWM
    let timer_config_out_current = TimerConfig::default().frequency(1.kHz().into())
        .resolution(esp_idf_hal::ledc::config::Resolution::Bits14);
    let timer_driver_0 = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config_out_current).unwrap();
    let mut pwm_driver = LedcDriver::new(peripherals.ledc.channel0, &timer_driver_0, peripherals.pins.gpio38).unwrap();
    pwm_driver.set_duty(0).expect("Set duty failure");
    let max_duty = pwm_driver.get_max_duty();
    info!("Max duty: {}", max_duty);
    // FAN PWM
    let timer_config_fan = TimerConfig::default().frequency(1.kHz().into())
        .resolution(esp_idf_hal::ledc::config::Resolution::Bits8);
    let timer_driver_1 = LedcTimerDriver::new(peripherals.ledc.timer1, &timer_config_fan).unwrap();
    let mut fan_pwm_driver = LedcDriver::new(peripherals.ledc.channel1, &timer_driver_1, peripherals.pins.gpio40).unwrap();
    fan_pwm_driver.set_duty(0).expect("Set duty failure");
    let max_fan_duty = fan_pwm_driver.get_max_duty();
    info!("Max duty: {}", max_fan_duty);

    // FAN POWER
    let mut fan_on = PinDriver::output(peripherals.pins.gpio42).unwrap();
    fan_on.set_low().unwrap();
    // FAN DETECT
    let fan_det = Box::new(PinDriver::input(peripherals.pins.gpio41).unwrap());
    let mut pulscnt = PulsCountDriver::new();
    pulscnt.start(fan_det);
    
    // Temperature Logs
    let mut clogs = CurrentRecord::new();

    // WiFi
    let wifi_enable : bool;
    let wifi = wifi::wifi_connect(peripherals.modem, CONFIG.wifi_ssid, CONFIG.wifi_psk);
    match wifi {
        Ok(_) => { wifi_enable = true; },
        Err(e) => { info!("{:?}", e); wifi_enable = false }
    }
    
    // NTP Server
    let sntp_conf = SntpConf {
        servers: ["time.aws.com",
                    "time.google.com",
                    "time.cloudflare.com",
                    "ntp.nict.jp"],
        operating_mode: OperatingMode::Poll,
        sync_mode: SyncMode::Immediate,
    };
    let ntp = EspSntp::new(&sntp_conf).unwrap();

    // NTP Sync
    // let now = SystemTime::now();
    // if now.duration_since(UNIX_EPOCH).unwrap().as_millis() < 1700000000 {
    info!("NTP Sync Start..");

    // wait for sync
    while ntp.get_sync_status() != SyncStatus::Completed {
        thread::sleep(Duration::from_millis(10));
    }
    let now = SystemTime::now();
    let dt_now : DateTime<Utc> = now.into();
    let formatted = format!("{}", dt_now.format("%Y-%m-%d %H:%M:%S"));
    info!("NTP Sync Completed: {}", formatted);
        
    let mut txd =  Transfer::new(server_info);
    txd.start()?;

    // TouchPad
    let mut touchpad = TouchPad::new();
    touchpad.start();
    
    // ADC GPIO18 for Temperature
    let mut adc = AdcDriver::new(peripherals.adc2, &AdcConfig::new().calibration(true))?;
    let mut temp_pin : AdcChannelDriver<'_, {esp_idf_sys::adc_atten_t_ADC_ATTEN_DB_11}, Gpio18> = AdcChannelDriver::new(peripherals.pins.gpio18)?;

    // PID Controller
    let pid_kp = CONFIG.pid_kp.parse::<f32>().unwrap();
    let pid_ki = CONFIG.pid_ki.parse::<f32>().unwrap();
    let pid_kd = CONFIG.pid_kd.parse::<f32>().unwrap();
    let pwm_offset = CONFIG.pwm_offset.parse::<u32>().unwrap();
    info!("PID Controller: KP={} KI={} KD={}", pid_kp, pid_ki, pid_kd);
    let mut pid = PIDController::new(pid_kp, pid_ki, pid_kd, 0.0);

    // Start Display
    dp.enable_display(true);

    // loop
    let mut measurement_count : u32 = 0;
    let mut logging_start = false;
    let mut load_start = false;
    let mut set_load_current = 0.0;
    let mut pwm_duty = 0;
    loop {
        thread::sleep(Duration::from_millis(10));

        let mut start_stop_btn = false;
        measurement_count += 1;
        if measurement_count % 5 == 0 {
            let touch_event : Vec<Key> = touchpad.tocuhpad_key_event();
            for touch in &touch_event {
                match touch {
                    Key::None => {},
                    // Key::Left => { interval_select_btn = true; },
                    Key::Center => { start_stop_btn = true; },
                    Key::Clockwise => {
                        set_load_current += 0.01;
                        dp.set_load_current(set_load_current);
                    },
                    Key::CounterClockwise => {
                        if set_load_current > 0.0 {
                            set_load_current -= 0.01;
                        }
                        dp.set_load_current(set_load_current);
                    },
                    _ => {},
                }
            }
        }
        if start_stop_btn == true {
            if load_start == true {
                // to Stop
                logging_start = false;
                load_start = false;
                // clogs.dump();
                // clogs.clear();
            }
            else {
                // to Start
                logging_start = true;
                load_start = true;
                measurement_count = 0;
                info!("Logging and Sending Start..");
                pid.reset();
                clogs.clear();
                dp.enable_display(true);
            }
        }

        if wifi_enable == false{
            dp.set_wifi_status(WifiStatus::Disconnected);
        }
        else {
            dp.set_wifi_status(WifiStatus::Connected);
        }

        if load_start == true {
            fan_on.set_high().unwrap();
            pid.set_setpoint(set_load_current);
            dp.set_current_status(LoggingStatus::Start);
        }
        else {
            dp.set_current_status(LoggingStatus::Stop);
        }

        // Read Current/Voltage
        let mut vbus_buf  = [0u8; 3];
        let mut data = CurrentLog::default();
        // Timestamp
        let now = SystemTime::now();
        // set clock in ns
        data.clock = now.duration_since(SystemTime::UNIX_EPOCH).unwrap().as_nanos();

        i2cdrv.write(0x40, &[0x05u8; 1], BLOCK)?;
        match i2cdrv.read(0x40, &mut vbus_buf, BLOCK){
            Ok(_v) => {
                let vbus = ((((vbus_buf[0] as u32) << 16 | (vbus_buf[1] as u32) << 8 | (vbus_buf[2] as u32)) >> 4) as f32 * 193.3125) / 1000_000.0;
                data.voltage = vbus; // V
                // info!("vbus={:?} {:?}V", vbus_buf, data.voltage);
            },
            Err(e) => {
                info!("{:?}", e);
                dp.set_message(format!("{:?}", e));
            }
        }
        let mut curt_buf  = [0u8; 3];
        i2cdrv.write(0x40, &[0x07u8; 1], BLOCK)?;
        match i2cdrv.read(0x40, &mut curt_buf, BLOCK) {
            Ok(_v) => {
                let current_reg : f32;
                if curt_buf[0] & 0x80 == 0x80 {
                    current_reg = (0x100000 - (((curt_buf[0] as u32) << 16 | (curt_buf[1] as u32) << 8 | (curt_buf[2] as u32)) >> 4)) as f32 * -1.0;
                }
                else {
                    current_reg = (((curt_buf[0] as u32) << 16 | (curt_buf[1] as u32) << 8 | (curt_buf[2] as u32)) >> 4) as f32;
                }
                let current = current_lsb * current_reg;
                data.current = current;   // A
                // info!("curt={:?} {:?}A", curt_buf, data.current);
            },
            Err(e) => {
                info!("{:?}", e);
                dp.set_message(format!("{:?}", e));
            }
        }
        // Power
        let mut power_buf = [0u8; 3];
        i2cdrv.write(0x40, &[0x08u8; 1], BLOCK)?;
        match i2cdrv.read(0x40, &mut power_buf, BLOCK) {
            Ok(_v) => {
                // info!("power={:?}", power_buf);
                let power_reg = ((power_buf[0] as u32) << 16 | (power_buf[1] as u32) << 8 | (power_buf[2] as u32)) as f32;
                let power = 3.2 * current_lsb * power_reg;        
                data.power = power;   // W
            },
            Err(e) => {
                info!("{:?}", e);
                dp.set_message(format!("{:?}", e));
            }
        }
        // Current and Power Limit
        if data.current > max_current_limit {
            info!("Current Limit Over: {:.3}A", data.current);
            dp.set_message(format!("CLO {:.3}A", data.current));
            load_start = false;
        }
        if data.power > max_power_limit {
            info!("Power Limit Over: {:.1}W", data.power);
            dp.set_message(format!("PLO {:.1}W", data.power));
            load_start = false;
        }

        // Temperature
        let temp = adc.read(&mut temp_pin).unwrap() as f32 * 0.05;
        data.temp = temp;
        // info!("Temperature: {:.2}°C", temp);
        dp.set_temperature(temp);
        let fan_duty = fan_pwm_control(temp, max_fan_duty);
        if load_start == false && fan_duty == 0 {
            fan_on.set_low().unwrap();
        }
        fan_pwm_driver.set_duty(fan_duty).expect("Set duty failure");
        // FAN RPM
        let rpm = pulscnt.get_rpm();
        data.rpm = rpm;
        dp.set_voltage(data.voltage, data.current, data.power);
        if data.voltage < 0.05 || data.current - set_load_current > 5.0 {
            // no voltage, over current
            pid.reset();
        }
        else {
            // PID Control  
            let pid_out = pid.update(data.current);
            pwm_duty = (pid_out * (max_duty as f32)) as u32 + pwm_offset;
        }    
        // info!("Current {:.3}A PID OUT {} -> Duty: {} ", average_current, pid_out, pwm_duty);
        // PID Control
        if pwm_duty > max_duty {
            pwm_duty = max_duty;
        }
        if load_start == false {
            pwm_duty = 0;
        }
        pwm_driver.set_duty(pwm_duty).expect("Set duty failure");
        dp.set_pwm_duty(pwm_duty);
        data.pwm = pwm_duty;
        if logging_start {
            clogs.record(data);
        }
        let current_record = clogs.get_size();
        if current_record >= 4095 {
            logging_start = false;  // Auto stop logging if buffer is full.
        }
        dp.set_buffer_watermark((current_record as u32) * 100 / 4095);

        if wifi_enable == true && current_record > 0 {
            let logs = clogs.get_all_data();
            let txcount = txd.set_transfer_data(logs);
            if txcount > 0 {
                clogs.remove_data(txcount);
            }
        }
    }
}

fn fan_pwm_control(current_temp: f32, max_duty: u32) -> u32 {
    let duty;
    if current_temp > 50.0 {
        duty = max_duty;
    }
    else if current_temp > 40.0 {
        duty = max_duty * ((current_temp - 40.0) as u32 + 1) / 10;
    }
    else {
        duty = 0;
    }
    duty
}