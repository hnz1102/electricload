// This is electric-load main program.
// License: MIT
// Author: 2024, Hiroshi Nakajima

#![allow(dead_code)]

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, SystemTime};
use std::sync::atomic::{AtomicBool, Ordering};
use std::ffi::c_void;
use log::*;

const MAX_TOUCHPADS: usize = 14;
const THRESHOLD_PERCENT_C: f32 = 0.05;
const THRESHOLD_PERCENT_R: f32 = 0.01;

static TOUCH_ACTIVE_FLAG: AtomicBool = AtomicBool::new(false);

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Key {
    None,
    Center,
    Up,
    Down,
    Left,
    Right,
    Clockwise,
    CounterClockwise,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KeyEvent {
    KeyDown,
    KeyUp,
}

struct KeyState {
    state_all: bool,
    state: [bool; MAX_TOUCHPADS],
    event_timer: SystemTime,
    center_button_pressed: bool,
    center_button_notified: bool,
    center_button_timer: SystemTime,
    event: Key,
    key_pos: Vec<(bool, f32, f32)>,
    p_pos: (f32, f32),
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(unused)]
enum TouchPadChannel {
    TouchPad1 = 1,      // C1
    TouchPad2 = 2,      // C2
    TouchPad3 = 3,      // C3
    TouchPad4 = 4,      // C1
    TouchPad5 = 5,      // C5
    TouchPad6 = 6,      // C6
    TouchPad7 = 7,      // C7
    TouchPad8 = 8,      // R1
    TouchPad9 = 9,      // R2
    TouchPad10 = 10,    // R3
    TouchPad11 = 11,    // R4
    TouchPad12 = 12,    // R5
    TouchPad13 = 13,    // R6
    TouchPad14 = 14,    // R7
}

const USE_TOUCH_PAD_CHANNEL : [TouchPadChannel; MAX_TOUCHPADS] = [
    TouchPadChannel::TouchPad4,     // C1 0
    TouchPadChannel::TouchPad5,     // C2 1
    TouchPadChannel::TouchPad6,     // C3 2
    TouchPadChannel::TouchPad7,     // C4 3
    TouchPadChannel::TouchPad8,     // C5 4
    TouchPadChannel::TouchPad3,     // C6 5
    TouchPadChannel::TouchPad9,     // C7 6
    TouchPadChannel::TouchPad1,     // R1 7
    TouchPadChannel::TouchPad2,     // R2 8
    TouchPadChannel::TouchPad14,    // R3 9
    TouchPadChannel::TouchPad13,    // R4 10
    TouchPadChannel::TouchPad12,    // R5 11
    TouchPadChannel::TouchPad11,    // R6 12
    TouchPadChannel::TouchPad10,    // R7 13
];

struct TouchState {
    smooth_value: [u32; MAX_TOUCHPADS],
}

pub struct TouchPad {
    touch_state: Arc<Mutex<TouchState>>,
    key_state: Arc<Mutex<KeyState>>,
}

unsafe extern "C" fn touch_key_interrupt_handler(_arg: *mut c_void) {
    let intr = esp_idf_sys::touch_pad_read_intr_status_mask();
    if (intr & (esp_idf_sys::touch_pad_intr_mask_t_TOUCH_PAD_INTR_MASK_ACTIVE as u32 |
                esp_idf_sys::touch_pad_intr_mask_t_TOUCH_PAD_INTR_MASK_INACTIVE as u32)
    ) != 0 {
        TOUCH_ACTIVE_FLAG.store(true, Ordering::Relaxed);
    }
}

#[allow(dead_code)]
impl TouchPad {
    pub fn new() -> TouchPad {
        TouchPad { 
            touch_state: Arc::new(Mutex::new(
                TouchState {
                            smooth_value: [0; MAX_TOUCHPADS],
                }
            )),
            key_state: Arc::new(Mutex::new(
                KeyState {
                    state_all: false,
                    state: [false; MAX_TOUCHPADS],
                    event_timer: SystemTime::now(),
                    center_button_pressed: false,
                    center_button_notified: false,
                    center_button_timer: SystemTime::now(),
                    event: Key::None,
                    key_pos: Vec::new(),
                    p_pos: (0.0, 0.0),
                }
            )),
        }
    }

    pub fn start(&mut self)
    {
        let touch_state = self.touch_state.clone();
        let key_state = self.key_state.clone();
        let _th = thread::spawn(move || {
            info!("Start TouchPad Read Thread.");
            unsafe {
                esp_idf_sys::touch_pad_init();
                for i in USE_TOUCH_PAD_CHANNEL.iter() {
                    match i {
                        TouchPadChannel::TouchPad1 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM1);
                        },
                        TouchPadChannel::TouchPad2 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM2);
                        },
                        TouchPadChannel::TouchPad3 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM3);
                        },
                        TouchPadChannel::TouchPad4 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM4);
                        },
                        TouchPadChannel::TouchPad5 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM5);
                        },
                        TouchPadChannel::TouchPad6 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM6);
                        },
                        TouchPadChannel::TouchPad7 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM7);
                        },
                        TouchPadChannel::TouchPad8 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM8);
                        },
                        TouchPadChannel::TouchPad9 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM9);
                        },
                        TouchPadChannel::TouchPad10 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM10);
                        },
                        TouchPadChannel::TouchPad11 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM11);
                        },
                        TouchPadChannel::TouchPad12 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM12);
                        },
                        TouchPadChannel::TouchPad13 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM13);
                        },
                        TouchPadChannel::TouchPad14 => {
                            esp_idf_sys::touch_pad_config(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM14);
                        },
                    }
                }
                esp_idf_sys::touch_pad_isr_register(Some(touch_key_interrupt_handler), std::ptr::null_mut(),
                    esp_idf_sys::touch_pad_intr_mask_t_TOUCH_PAD_INTR_MASK_ACTIVE |
                    esp_idf_sys::touch_pad_intr_mask_t_TOUCH_PAD_INTR_MASK_INACTIVE);
                esp_idf_sys::touch_pad_intr_enable(
                    esp_idf_sys::touch_pad_intr_mask_t_TOUCH_PAD_INTR_MASK_ACTIVE |
                    esp_idf_sys::touch_pad_intr_mask_t_TOUCH_PAD_INTR_MASK_INACTIVE);
                esp_idf_sys::touch_pad_set_fsm_mode(esp_idf_sys::touch_fsm_mode_t_TOUCH_FSM_MODE_TIMER);
                esp_idf_sys::touch_pad_fsm_start();
                thread::sleep(Duration::from_millis(100));

                let mut lck = touch_state.lock().unwrap();
                for i in USE_TOUCH_PAD_CHANNEL.iter() {
                    match i {
                        TouchPadChannel::TouchPad1 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM1, &mut lck.smooth_value[0]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM1, (lck.smooth_value[0] as f32 * THRESHOLD_PERCENT_R) as u32);
                            info!("TouchPad1 threshold: {}", (lck.smooth_value[0] as f32 * THRESHOLD_PERCENT_R) as u32);
                        },
                        TouchPadChannel::TouchPad2 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM2, &mut lck.smooth_value[1]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM2, (lck.smooth_value[1] as f32 * THRESHOLD_PERCENT_C) as u32);
                            info!("TouchPad2 threshold: {}", (lck.smooth_value[1] as f32 * THRESHOLD_PERCENT_C) as u32);
                        },
                        TouchPadChannel::TouchPad3 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM3, &mut lck.smooth_value[2]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM3, (lck.smooth_value[2] as f32 * THRESHOLD_PERCENT_C) as u32);
                            info!("TouchPad3 threshold: {}", (lck.smooth_value[2] as f32 * THRESHOLD_PERCENT_C) as u32);
                        },
                        TouchPadChannel::TouchPad4 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM4, &mut lck.smooth_value[3]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM4, (lck.smooth_value[3] as f32 * THRESHOLD_PERCENT_C) as u32);
                            info!("TouchPad4 threshold: {}", (lck.smooth_value[3] as f32 * THRESHOLD_PERCENT_C) as u32);
                        },
                        TouchPadChannel::TouchPad5 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM5, &mut lck.smooth_value[4]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM5, (lck.smooth_value[4] as f32 * THRESHOLD_PERCENT_C) as u32);
                            info!("TouchPad5 threshold: {}", (lck.smooth_value[4] as f32 * THRESHOLD_PERCENT_C) as u32);
                        },
                        TouchPadChannel::TouchPad6 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM6, &mut lck.smooth_value[5]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM6, (lck.smooth_value[5] as f32 * THRESHOLD_PERCENT_C) as u32);
                            info!("TouchPad6 threshold: {}", (lck.smooth_value[5] as f32 * THRESHOLD_PERCENT_C) as u32);
                        },
                        TouchPadChannel::TouchPad7 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM7, &mut lck.smooth_value[6]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM7, (lck.smooth_value[6] as f32 * THRESHOLD_PERCENT_C) as u32);
                            info!("TouchPad7 threshold: {}", (lck.smooth_value[6] as f32 * THRESHOLD_PERCENT_C) as u32);
                        },
                        TouchPadChannel::TouchPad8 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM8, &mut lck.smooth_value[7]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM8, (lck.smooth_value[7] as f32 * THRESHOLD_PERCENT_R) as u32);
                            info!("TouchPad8 threshold: {}", (lck.smooth_value[7] as f32 * THRESHOLD_PERCENT_R) as u32);
                        },
                        TouchPadChannel::TouchPad9 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM9, &mut lck.smooth_value[8]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM9, (lck.smooth_value[8] as f32 * THRESHOLD_PERCENT_R) as u32);
                            info!("TouchPad9 threshold: {}", (lck.smooth_value[8] as f32 * THRESHOLD_PERCENT_R) as u32);
                        },
                        TouchPadChannel::TouchPad10 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM10, &mut lck.smooth_value[9]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM10, (lck.smooth_value[9] as f32 * THRESHOLD_PERCENT_R) as u32);
                            info!("TouchPad10 threshold: {}", (lck.smooth_value[9] as f32 * THRESHOLD_PERCENT_R) as u32);
                        },
                        TouchPadChannel::TouchPad11 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM11, &mut lck.smooth_value[10]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM11, (lck.smooth_value[10] as f32 * THRESHOLD_PERCENT_R) as u32);
                            info!("TouchPad11 threshold: {}", (lck.smooth_value[10] as f32 * THRESHOLD_PERCENT_R) as u32);
                        },
                        TouchPadChannel::TouchPad12 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM12, &mut lck.smooth_value[11]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM12, (lck.smooth_value[11] as f32 * THRESHOLD_PERCENT_R) as u32);
                            info!("TouchPad12 threshold: {}", (lck.smooth_value[11] as f32 * THRESHOLD_PERCENT_R) as u32);
                        },
                        TouchPadChannel::TouchPad13 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM13, &mut lck.smooth_value[12]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM13, (lck.smooth_value[12] as f32 * THRESHOLD_PERCENT_R) as u32);
                            info!("TouchPad13 threshold: {}", (lck.smooth_value[12] as f32 * THRESHOLD_PERCENT_R) as u32);
                        },
                        TouchPadChannel::TouchPad14 => {
                            esp_idf_sys::touch_pad_filter_read_smooth(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM14, &mut lck.smooth_value[13]);
                            esp_idf_sys::touch_pad_set_thresh(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM14, (lck.smooth_value[13] as f32 * THRESHOLD_PERCENT_R) as u32);
                            info!("TouchPad14 threshold: {}", (lck.smooth_value[13] as f32 * THRESHOLD_PERCENT_R) as u32);
                        },
                    }
                }
            }
            unsafe {
                let mut times : u16 = 0;
                esp_idf_sys::touch_pad_get_charge_discharge_times(&mut times);
                info!("TouchPad charge discharge times: {} -> 1000", times);
                // esp_idf_sys::touch_pad_set_charge_discharge_times(1000);
            }

            loop {
                thread::sleep(Duration::from_millis(20));
                // info!("TouchPad Read Thread.");
                // raw data from touch pad
                // unsafe {
                //     let mut value1 = 0;
                //     let mut value2 = 0;
                //     let mut value3 = 0;
                //     let mut value4 = 0;
                //     let mut value5 = 0;
                //     let mut value6 = 0;
                //     let mut value7 = 0;
                //     let mut value8 = 0;
                //     let mut value9 = 0;
                //     esp_idf_sys::touch_pad_read_raw_data(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM1, &mut value1);
                //     esp_idf_sys::touch_pad_read_raw_data(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM2, &mut value2);
                //     esp_idf_sys::touch_pad_read_raw_data(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM3, &mut value3);
                //     esp_idf_sys::touch_pad_read_raw_data(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM4, &mut value4);
                //     esp_idf_sys::touch_pad_read_raw_data(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM5, &mut value5);
                //     esp_idf_sys::touch_pad_read_raw_data(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM6, &mut value6);
                //     esp_idf_sys::touch_pad_read_raw_data(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM7, &mut value7);
                //     esp_idf_sys::touch_pad_read_raw_data(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM8, &mut value8);
                //     esp_idf_sys::touch_pad_read_raw_data(esp_idf_sys::touch_pad_t_TOUCH_PAD_NUM9, &mut value9);
                //     info!("TouchPad raw data: {}, {}, {}, {}, {}, {}, {}, {}, {}", value1, value2, value3, value4, value5, value6, value7, value8, value9);
                // }
                if TOUCH_ACTIVE_FLAG.load(Ordering::Relaxed) {
                    unsafe {
                        let touch_status = esp_idf_sys::touch_pad_get_status();
                        //  info!("TouchPad status: {:08b}", touch_status);
                        for i in 0..=MAX_TOUCHPADS {
                            if touch_status & (1 << i) != 0 {
                                // info!("TouchPad{} touched.", i);
                                // i is 1 to 14
                                match i {
                                    1..=14 => {
                                        // convert interrupt number to touch pad c or r
                                        let state_idx = convert_index(i);
                                        let mut keylck = key_state.lock().unwrap();
                                        if ! keylck.state[state_idx] {
                                            keylck.state[state_idx] = true;
                                            // info!("TouchPad{} touched. idx: {}", i, state_idx);
                                        }
                                    },
                                    _ => {},
                                }
                            }
                            else {
                                match i {
                                    1..=14 => {
                                        // convert interrupt number to touch pad c or r
                                        let state_idx = convert_index(i);
                                        let mut keylck = key_state.lock().unwrap();    
                                        if keylck.state[state_idx] {
                                            keylck.state[state_idx] = false;
                                            // info!("TouchPad{} released. idx: {}", i, state_idx);
                                        }
                                    },
                                    _ => {},
                                }
                            }
                        }
                    }
                    TOUCH_ACTIVE_FLAG.store(false, Ordering::Relaxed);
                }
                let (state, xp, yp) = {
                    let mut x: f32 = 0.0;
                    let mut y: f32 = 0.0;
                    let mut nop : f32 = 0.0;
                    let keylck = key_state.lock().unwrap();
                    for r in 1..=7 {
                        for c in 1..=7 {
                            if keylck.state[r+6] && keylck.state[c-1] {
                                x += c as f32 - 4.0;
                                y += r as f32 - 4.0;
                                nop += 1.0;
                            }
                        }
                    }
                    if nop > 0.0 {
                        x = x / nop;
                        y = y / nop;
                        (true, x, y)
                    }
                    else {
                        (false, 0.0, 0.0)
                    }
                };
                let mut keylck = key_state.lock().unwrap();
                if state {
                    keylck.key_pos.push((state, xp, yp));
                }
            }
        });
    }

    pub fn stop_touchpad(&mut self)
    {
        unsafe {
            esp_idf_sys::touch_pad_intr_disable(
                esp_idf_sys::touch_pad_intr_mask_t_TOUCH_PAD_INTR_MASK_ACTIVE |
                esp_idf_sys::touch_pad_intr_mask_t_TOUCH_PAD_INTR_MASK_INACTIVE);
            esp_idf_sys::touch_pad_fsm_stop();
            esp_idf_sys::touch_pad_isr_register(None, std::ptr::null_mut(),
                esp_idf_sys::touch_pad_intr_mask_t_TOUCH_PAD_INTR_MASK_ACTIVE |
                esp_idf_sys::touch_pad_intr_mask_t_TOUCH_PAD_INTR_MASK_INACTIVE);
        }
    }

    pub fn tocuhpad_key_event(&mut self) -> Vec<Key>
    {
        let mut lck = self.key_state.lock().unwrap();
        let mut vector : Vec<Key> = Vec::new();
        if lck.key_pos.len() == 0 {
            return vector;
        }

        let duration = lck.event_timer.elapsed().unwrap().as_millis();
        if duration > 500 {
            lck.p_pos = (0.0, 0.0);
        }
        let (mut sx, mut sy) = (0.0, 0.0);
        // info!("Key Pos: {:?} duration:{} ms", lck.key_pos, duration);
        for (_s, x, y) in lck.key_pos.clone() {
            if x >=-0.5 && x <= 0.5 && y >= -0.5 && y <= 0.5 && duration < 400 {
                if !lck.center_button_pressed {
                    lck.center_button_pressed = true;
                    lck.center_button_timer = SystemTime::now();
                }
                else {
                    let pressed_time = lck.center_button_timer.elapsed().unwrap().as_millis();
                    if pressed_time > 1000 && !lck.center_button_notified {
                        lck.center_button_notified = true;
                        // info!("Center Button Long Pressed.");
                        vector.push(Key::Center);
                    }
                }
                lck.key_pos.clear();
                lck.event_timer = SystemTime::now();
                return vector;
            }
            else {
                // info!("Center Button Released. duration : {}", duration);
                lck.center_button_pressed = false;
                lck.center_button_notified = false;
            }
            sx += x;
            sy += y;
        }
        sx = sx / lck.key_pos.len() as f32;
        sy = sy / lck.key_pos.len() as f32;
        // info!("sx: {}, sy: {}", sx, sy);
        let px = lck.p_pos.0;
        let py = lck.p_pos.1;
        let angle_d = 1.0 - (px * sx + py * sy) / (px.hypot(py) * sx.hypot(sy));
        let s = px * sy - py * sx;
        // if s != 0.0 {  
        //     info!("({},{})->({},{}), angle_d: {} s:{}", px, py, sx, sy, angle_d, s);
        // }
        if s != 0.0 && angle_d != 0.0 {
            if s > 0.0 {
                let count = match angle_d {
                    0.03..0.5 => 1,
                    0.5..2.0 => 10,
                    _ => 0,
                };
                for _ in 0..count {
                    vector.push(Key::CounterClockwise);
                }
            }
            else if s < 0.0 {
                let count = match angle_d {
                    0.03..0.5 => 1,
                    0.5..2.0 => 10,
                    _ => 0,
                };
                for _ in 0..count {
                    vector.push(Key::Clockwise);
                }
            }    
        }
        lck.p_pos = (sx, sy);
        lck.key_pos.clear();
        lck.event_timer = SystemTime::now();
        vector
    }
 }

fn convert_index(idx: usize) -> usize
{
    let state_idx = match idx {
        1 => 7,
        2 => 8,
        3 => 5,
        4 => 0,
        5 => 1,
        6 => 2,
        7 => 3,
        8 => 4,
        9 => 6,
        10 => 13,
        11 => 12,
        12 => 11,
        13 => 10,
        14 => 9,
        _ => 0,
    };
    state_idx
}