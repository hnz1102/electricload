// FAN RPM counter
// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Hiroshi Nakajima

use log::*;
use std::{thread, time::Duration, sync::Arc, sync::Mutex, sync::atomic::AtomicBool};
use esp_idf_hal::{gpio::*};
use std::sync::atomic::Ordering;
use std::time::SystemTime;

type PINDRIVER = Box<PinDriver<'static, esp_idf_hal::gpio::Gpio41, esp_idf_hal::gpio::Input>>;

static GPIO_FLAG: AtomicBool = AtomicBool::new(false);

struct PulsCount {
    count: u32,
    rpm: u32,
}

pub struct PulsCountDriver {
    state: Arc<Mutex<PulsCount>>,
}

impl PulsCountDriver {
    pub fn new() -> Self {
        PulsCountDriver {
            state: Arc::new(Mutex::new(PulsCount { count: 0, rpm: 0 })),
        }
    }

    pub fn gpio_interrupt_handler() {
        GPIO_FLAG.store(true, Ordering::Relaxed);
    }

    pub fn start(&mut self, mut gpio_sig: PINDRIVER) {
        let state = self.state.clone();
        thread::spawn(move || {
            info!("Start puls count thread.");
            gpio_sig.set_pull(Pull::Up).unwrap();
            gpio_sig.set_interrupt_type(InterruptType::AnyEdge).unwrap();
            unsafe {
                gpio_sig.subscribe(PulsCountDriver::gpio_interrupt_handler).unwrap();
            }
            gpio_sig.enable_interrupt().unwrap();
            let mut puls_timer = SystemTime::now();
            loop {
                if GPIO_FLAG.load(Ordering::Relaxed) {
                    let mut state = state.lock().unwrap();
                    state.count += 1;
                    GPIO_FLAG.store(false, Ordering::Relaxed);
                    gpio_sig.enable_interrupt().unwrap();
                    let duration = puls_timer.elapsed().unwrap();
                    if duration.as_millis() > 1000 {
                        state.rpm = (30 * state.count * 1000) / duration.as_millis() as u32;
                        info!("Puls: {} {} rpm", state.count, state.rpm);
                        puls_timer = SystemTime::now();
                        state.count = 0;
                    }
                }
                thread::sleep(Duration::from_millis(10));
            }
        });
    }

    pub fn get_rpm(&self) -> u32 {
        let state = self.state.lock().unwrap();
        state.rpm
    }
}