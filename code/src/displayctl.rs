// Display control module for SSD1331 OLED display.
// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Hiroshi Nakajima

#![allow(dead_code)]

use log::*;
use std::{thread, time::Duration, sync::Arc, sync::Mutex};
use esp_idf_hal::{gpio::*, spi, delay::FreeRtos};
use ssd1331::{DisplayRotation, Ssd1331};
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, ascii::FONT_5X8, ascii::FONT_6X12, MonoTextStyle},
    image::Image,
    pixelcolor::{Rgb565},
    text::{Text},
    geometry::Point,
    primitives::{
        Circle, Line, Triangle, PrimitiveStyle,
    },
    prelude::*,
};
use tinybmp::Bmp;

pub enum LoggingStatus {
    Start,
    Stop,
}

pub enum WifiStatus {
    Connected,
    Disconnected,
}

type SPI<'d> = esp_idf_hal::spi::SpiDeviceDriver<'static, spi::SpiDriver<'static>>;
type DC<'d> = esp_idf_hal::gpio::PinDriver<'static, Gpio15, esp_idf_hal::gpio::Output>;
type RST<'d> = esp_idf_hal::gpio::PinDriver<'static, Gpio16, esp_idf_hal::gpio::Output>;

struct DisplayText {
    display_enable: bool,
    voltage: f32,
    current: f32,
    power: f32,
    interval: u32,
    message: String,
    message_enable: bool,
    battery: f32,
    status: LoggingStatus,
    wifi: WifiStatus,
    buffer_water_mark: u32,
    load_current: f32,
    temperature: f32,
    pwm_duty: u32,
}

pub struct DisplayPanel {
    txt: Arc<Mutex<DisplayText>>
}

impl DisplayPanel {

    pub fn new() -> DisplayPanel {
        DisplayPanel { txt: Arc::new(Mutex::new(
            DisplayText {display_enable: false,
                         voltage: 0.0,
                         message: "".to_string(),
                         message_enable: false,
                         current: 0.0,
                         power: 0.0,
                         interval: 0,
                         battery: 0.0,
                         status: LoggingStatus::Stop,
                         wifi: WifiStatus::Disconnected,
                         buffer_water_mark: 0,
                         load_current: 0.0,
                         temperature: 0.0,
                         pwm_duty: 0,
                     })) }
    }

    pub fn start(&mut self,
        spi : SPI, dc: DC, mut rst : RST)
    {
        let txt = self.txt.clone();
        let _th = thread::spawn(move || {
            info!("Start Display Thread.");
            let mut delay = FreeRtos;
            let mut display = Ssd1331::new(spi, dc, DisplayRotation::Rotate0);
            let _ = display.reset(&mut rst, &mut delay);
            let _ = display.init();
            display.clear();
            let _style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
            let middle_style_white = MonoTextStyle::new(&FONT_6X12, Rgb565::WHITE);
            let middle_style_red = MonoTextStyle::new(&FONT_6X12, Rgb565::RED);
            let middle_style_yellow = MonoTextStyle::new(&FONT_6X12, Rgb565::YELLOW);
            let middle_style_blue = MonoTextStyle::new(&FONT_6X12, Rgb565::BLUE);
            let _small_style_white = MonoTextStyle::new(&FONT_5X8, Rgb565::WHITE);
            let wifibmp = Bmp::from_slice(include_bytes!("./img/wifirev.bmp")).unwrap();
            let wifi_img: Image<Bmp<Rgb565>> = Image::new(&wifibmp, Point::new(86, 47));
            let fill = PrimitiveStyle::with_fill(Rgb565::YELLOW);
            // Logo BMP
            let logobmp = Bmp::from_slice(include_bytes!("./img/logo.bmp")).unwrap();
            let logo_img: Image<Bmp<Rgb565>> = Image::new(&logobmp, Point::new(0,0));

            // Number BMP
            let n0 = Bmp::from_slice(include_bytes!("./img/n0.bmp")).unwrap();
            let n0_img: Image<Bmp<Rgb565>> = Image::new(&n0, Point::zero());
            let n1 = Bmp::from_slice(include_bytes!("./img/n1.bmp")).unwrap();
            let n1_img: Image<Bmp<Rgb565>> = Image::new(&n1, Point::zero());
            let n2 = Bmp::from_slice(include_bytes!("./img/n2.bmp")).unwrap();
            let n2_img: Image<Bmp<Rgb565>> = Image::new(&n2, Point::zero());
            let n3 = Bmp::from_slice(include_bytes!("./img/n3.bmp")).unwrap();
            let n3_img: Image<Bmp<Rgb565>> = Image::new(&n3, Point::zero());
            let n4 = Bmp::from_slice(include_bytes!("./img/n4.bmp")).unwrap();
            let n4_img: Image<Bmp<Rgb565>> = Image::new(&n4, Point::zero());
            let n5 = Bmp::from_slice(include_bytes!("./img/n5.bmp")).unwrap();
            let n5_img: Image<Bmp<Rgb565>> = Image::new(&n5, Point::zero());
            let n6 = Bmp::from_slice(include_bytes!("./img/n6.bmp")).unwrap();
            let n6_img: Image<Bmp<Rgb565>> = Image::new(&n6, Point::zero());
            let n7 = Bmp::from_slice(include_bytes!("./img/n7.bmp")).unwrap();
            let n7_img: Image<Bmp<Rgb565>> = Image::new(&n7, Point::zero());
            let n8 = Bmp::from_slice(include_bytes!("./img/n8.bmp")).unwrap();
            let n8_img: Image<Bmp<Rgb565>> = Image::new(&n8, Point::zero());
            let n9 = Bmp::from_slice(include_bytes!("./img/n9.bmp")).unwrap();
            let n9_img: Image<Bmp<Rgb565>> = Image::new(&n9, Point::zero());
            // let vv = Bmp::from_slice(include_bytes!("./img/v.bmp")).unwrap();
            // let vv_img: Image<Bmp<Rgb565>> = Image::new(&vv, Point::new(68, 0));
            let amp = Bmp::from_slice(include_bytes!("./img/A.bmp")).unwrap();
            let amp_img: Image<Bmp<Rgb565>> = Image::new(&amp, Point::new(88, 0));
            let dot = Bmp::from_slice(include_bytes!("./img/dot.bmp")).unwrap();
            let dot_img: Image<Bmp<Rgb565>> = Image::new(&dot, Point::zero());
            let minus = Bmp::from_slice(include_bytes!("./img/minus.bmp")).unwrap();
            let minus_img: Image<Bmp<Rgb565>> = Image::new(&minus, Point::zero());
            let mut digit_img = n0_img.translate(Point::new(0,0));

            let mut loopcount = 0;
            loop {
                thread::sleep(Duration::from_millis(100));
                let lck = txt.lock().unwrap();
                display.clear();
                if lck.message_enable {
                    Text::new(&format!("{}", lck.message), Point::new(1, 20), middle_style_red).draw(&mut display).unwrap();
                    display.flush().unwrap();
                    drop(lck);
                    continue;
                }
                if lck.display_enable {
                    let mut disp_val = lck.current;
                    dot_img.draw(&mut display).unwrap();                
                    // vv_img.draw(&mut display).unwrap();
                    amp_img.draw(&mut display).unwrap();
                    let mut digit_10 = 10.0;
                    let mut first_digit = true;
                    let mut pos_x = 0;
                    for digit in 0..=4 {
                        if pos_x >= 88 {
                            continue;
                        }
                        let num = (disp_val / digit_10) as i32;
                        if disp_val < 0.0 && digit == 0 {
                            digit_img = minus_img.translate(Point::new(pos_x, 0));
                            digit_img.draw(&mut display).unwrap();
                            pos_x += 20;
                        }
                        match num {
                            0 => {
                                if !first_digit || digit > 0 {
                                    digit_img = n0_img.translate(Point::new(pos_x, 0));
                                    pos_x += 20;
                                }
                            },
                            1 | -1 => {
                                digit_img = n1_img.translate(Point::new(pos_x, 0));
                                first_digit = false;
                                pos_x += 20;
                            },
                            2 | -2 => {
                                digit_img = n2_img.translate(Point::new(pos_x, 0));
                                first_digit = false;
                                pos_x += 20;
                            },
                            3 | -3 => {
                                digit_img = n3_img.translate(Point::new(pos_x, 0));
                                first_digit = false;
                                pos_x += 20;
                            },
                            4 | -4 => {
                                digit_img = n4_img.translate(Point::new(pos_x, 0));
                                first_digit = false;
                                pos_x += 20;
                            },
                            5 | -5 => {
                                digit_img = n5_img.translate(Point::new(pos_x, 0));
                                first_digit = false;
                                pos_x += 20;
                            },
                            6 | -6 => {
                                digit_img = n6_img.translate(Point::new(pos_x, 0));
                                first_digit = false;
                                pos_x += 20;
                            },
                            7 | -7 => {
                                digit_img = n7_img.translate(Point::new(pos_x, 0));
                                first_digit = false;
                                pos_x += 20;
                            },
                            8 | -8 => {
                                digit_img = n8_img.translate(Point::new(pos_x, 0));
                                first_digit = false;
                                pos_x += 20;
                            },
                            9 | -9 => {
                                digit_img = n9_img.translate(Point::new(pos_x, 0));
                                first_digit = false;
                                pos_x += 20;
                            },
                            _ => {}
                        }
                        digit_img.draw(&mut display).unwrap();
                        if digit == 1 {
                            digit_img = dot_img.translate(Point::new(pos_x, 0));
                            digit_img.draw(&mut display).unwrap();
                            pos_x += 8;
                        }
                        disp_val = disp_val - digit_10 * (num as f32);
                        digit_10 /= 10.0;
                    }
                }
                else {
                    logo_img.draw(&mut display).unwrap();
                    display.flush().unwrap();
                    drop(lck);
                    continue;
                }

                match lck.status {
                    LoggingStatus::Start => {
                        match loopcount {
                            0..=5 => {
                                Circle::new(Point::new(1, 53), 8)
                                    .into_styled(fill)
                                    .draw(&mut display).unwrap();
                            },
                            _ => {},
                        }
                    },
                    LoggingStatus::Stop => {
                    },
                }
                let cur_pos = 50;
                if lck.voltage.abs() >= 10.0 && lck.voltage <= 30.0 {
                    Text::new(&format!("{:.2}V", lck.voltage), Point::new(10, cur_pos), middle_style_yellow).draw(&mut display).unwrap();
                }
                else if lck.voltage.abs() > 30.0 {
                    Text::new(&format!("{:.2}V", lck.voltage), Point::new(10, cur_pos), middle_style_red).draw(&mut display).unwrap();
                }
                else {
                    Text::new(&format!("{:.2}V", lck.voltage), Point::new(10, cur_pos), middle_style_white).draw(&mut display).unwrap();
                }

                // Power
                if lck.power < 1.0 {
                    Text::new(&format!("{:.0}mW", lck.power * 1000.0), Point::new(54, cur_pos), middle_style_white).draw(&mut display).unwrap();
                }
                else if lck.power >= 10.0 && lck.power < 50.0 {
                    Text::new(&format!("{:.1}W", lck.power), Point::new(54, cur_pos), middle_style_yellow).draw(&mut display).unwrap();
                }
                else if lck.power >= 50.0 {
                    Text::new(&format!("{:.1}W", lck.power), Point::new(54, cur_pos), middle_style_red).draw(&mut display).unwrap();
                }
                else {
                    Text::new(&format!("{:.2}W", lck.power), Point::new(54, cur_pos), middle_style_white).draw(&mut display).unwrap();
                }

                // Water mark of buffer
                let bar_len = (lck.buffer_water_mark * 95 / 100) as i32;
                Line::new(Point::new(0,63), Point::new(bar_len, 63)).into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 1)).draw(&mut display).unwrap();
                Triangle::new(Point::new(bar_len-2,61), Point::new(bar_len,63), Point::new(bar_len-2,63)).into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 1)).draw(&mut display).unwrap();

                match lck.wifi {
                    WifiStatus::Disconnected => {
                    },
                    WifiStatus::Connected => {
                        wifi_img.draw(&mut display).unwrap();
                    },
                }

                // Load current
                if lck.load_current < 5.0 {
                    Text::new(&format!("{:.2}A", lck.load_current), Point::new(10, 60), middle_style_blue).draw(&mut display).unwrap();
                }
                else if lck.load_current >= 5.0 && lck.load_current < 10.0 {
                    Text::new(&format!("{:.2}A", lck.load_current), Point::new(10, 60), middle_style_yellow).draw(&mut display).unwrap();
                }
                else if lck.load_current > 10.0 {
                    Text::new(&format!("{:.2}A", lck.load_current), Point::new(10, 60), middle_style_red).draw(&mut display).unwrap();
                }

                match loopcount {
                    0..=5 => {
                        // Temperature
                        Text::new(&format!("{:.0}C", lck.temperature), Point::new(54, 60), middle_style_white).draw(&mut display).unwrap();
                    },
                    _ => {
                        // PWM Duty
                        Text::new(&format!("{}", lck.pwm_duty), Point::new(54, 60), middle_style_white).draw(&mut display).unwrap();
                    },
                }
 
                loopcount += 1;
                if loopcount == 10 {
                    loopcount = 0;
                }
                display.flush().unwrap();
                drop(lck);
            }
        });
    }

    pub fn enable_display(&mut self, enable: bool)
    {
        let mut lck = self.txt.lock().unwrap();
        lck.display_enable = enable;
    }

    pub fn set_voltage(&mut self, vol: f32, cur: f32, power: f32)
    {
        let mut lck = self.txt.lock().unwrap();
        lck.voltage = vol;
        lck.current = cur;
        lck.power = power;
    }

    pub fn set_interval(&mut self, interval : u32)
    {
        let mut lck = self.txt.lock().unwrap();
        lck.interval = interval;
    }

    pub fn set_current_status(&mut self, status: LoggingStatus)
    {
        let mut lck= self.txt.lock().unwrap();
        lck.status = status;
    }

    pub fn set_wifi_status(&mut self, status: WifiStatus)
    {
        let mut lck= self.txt.lock().unwrap();
        lck.wifi = status;
    }

    pub fn set_message(&mut self, msg: String)
    {
        let mut lck = self.txt.lock().unwrap();
        lck.message = msg;
        lck.message_enable = true;
    }

    pub fn set_battery(&mut self, bat: f32){
        let mut lck = self.txt.lock().unwrap();
        lck.battery = bat;
    }

    pub fn set_buffer_watermark(&mut self, wm: u32){
        let mut lck = self.txt.lock().unwrap();
        lck.buffer_water_mark = wm;
    }

    pub fn set_load_current(&mut self, load_current: f32){
        let mut lck = self.txt.lock().unwrap();
        lck.load_current = load_current;
    }

    pub fn set_pwm_duty(&mut self, duty: u32){
        let mut lck = self.txt.lock().unwrap();
        lck.pwm_duty = duty;
    }

    pub fn set_temperature(&mut self, temp: f32){
        let mut lck = self.txt.lock().unwrap();
        lck.temperature = temp;
    }
}
