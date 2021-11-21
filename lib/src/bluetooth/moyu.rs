use crate::bluetooth::BluetoothCubeDevice;
use crate::common::{Cube, Face, Move, TimedMove};
use crate::cube3x3x3::Cube3x3x3;
use anyhow::{anyhow, Result};
use btleplug::api::{Characteristic, Peripheral};
use std::ops::Deref;
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use uuid::Uuid;

extern crate hexdump;
// use byteorder::{ByteOrder, LittleEndian};
// use pretty_hex::*;
use std::fmt;
// use std::ops::{Mul, Sub};
use std::ops::Sub;
// use std::time::Duration;

use crate::gyro::QGyroState;

const DEBUG_GYRO: bool = false;

struct MoYuCube<P: Peripheral + 'static> {
    device: P,
    state: Arc<Mutex<Cube3x3x3>>,
    synced: Arc<Mutex<bool>>,
}

// #[derive(PartialEq, PartialOrd, Debug, Copy, Clone)]
// struct Quat {
//     w: f32,
//     x: f32,
//     y: f32,
//     z: f32,
// }

// impl Quat {
//     pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
//         // Self { w: w, x: x, z: -z, y: y }
//         Self { w, x, y, z }
//     }
//     pub fn from_array(q: &[f32; 4]) -> Self {
//         Self {
//             w: q[0],
//             x: q[1],
//             y: q[2],
//             z: q[3],
//         }
//     }
//     pub fn as_array(&self) -> [f32; 4] {
//         [self.w, self.x, self.y, self.z]
//     }
//     fn from_bytes(bytes: &[u8]) -> Quat {
//         let mut r: [f32; 4] = [0.0, 0.0, 0.0, 0.0];
//         for i in 0..4 {
//             r[i] = LittleEndian::read_f32(&bytes[i * 4..i * 4 + 4]);
//         }
//         Quat::new(r[0], r[1], -r[3], r[2])
//     }

//     fn quat_invert(&self) -> Quat {
//         let (w, x, y, z) = (self.w, self.x, self.y, self.z);
//         let f = 1.0 / (w * w + x * x + y * y + z * z);
//         return Quat::new(w * f, -x * f, -y * f, -z * f);
//     }

//     fn quat_normalize(&self) -> Quat {
//         let (w, x, y, z) = (self.w, self.x, self.y, self.z);
//         let f = (1.0 / (w * w + x * x + y * y + z * z)).sqrt();
//         return Quat::new(w * f, x * f, y * f, z * f);
//     }

//     fn quat_matrix(&self) -> [Quat; 4] {
//         let (w, x, y, z) = (self.w, self.x, self.y, self.z);
//         return [
//             Quat::new(
//                 w * w + x * x - y * y - z * z,
//                 2.0 * x * y - 2.0 * w * z,
//                 2.0 * x * z + 2.0 * w * y,
//                 0.0,
//             ),
//             Quat::new(
//                 2.0 * x * y + 2.0 * w * z,
//                 w * w - x * x + y * y - z * z,
//                 2.0 * y * z - 2.0 * w * x,
//                 0.0,
//             ),
//             Quat::new(
//                 2.0 * x * z - 2.0 * w * y,
//                 2.0 * y * z + 2.0 * w * x,
//                 w * w - x * x - y * y + z * z,
//                 0.0,
//             ),
//             Quat::new(0.0, 0.0, 0.0, 1.0),
//         ];
//     }
// }

// impl Sub for Quat {
//     type Output = Quat;
//     fn sub(self, rhs: Quat) -> Quat {
//         Quat {
//             w: self.w - rhs.w,
//             x: self.x - rhs.x,
//             y: self.y - rhs.y,
//             z: self.z - rhs.z,
//         }
//     }
// }

// impl Mul for Quat {
//     type Output = Quat;
//     fn mul(self, rhs: Quat) -> Quat {
//         let (a, b, c, d) = (self.w, self.x, self.y, self.z);
//         let (w, x, y, z) = (rhs.w, rhs.x, rhs.y, rhs.z);
//         Quat {
//             w: a * w - b * x - c * y - d * z,
//             x: a * x + b * w + c * d - d * y,
//             y: a * y - b * z + c * w + d * x,
//             z: a * z + b * y - c * x + d * w,
//         }
//     }
// }

// impl fmt::Display for Quat {
//     fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
//         const F: f32 = 100000.0;
//         let (w, x, y, z) = (self.w * F, self.x * F, self.y * F, self.z * F);
//         write!(
//             f,
//             "[w:{:+10.2}  x:{:+10.2}  y:{:+10.2}  z:{:+10.2}]",
//             w, x, y, z,
//             // self.w * f, self.x * f, self.y * f, self.z * f,
//         )
//     }
// }

// #[derive(PartialEq, PartialOrd, Debug, Copy, Clone)]
// struct QGyroState {
//     t: u32,
//     q: Quat,
//     d: bool,
// }

// impl QGyroState {
//     pub fn new(t: u32, q: Quat) -> Self {
//         Self { t, q, d: false }
//     }
//     pub fn from_bytes(bytes: &[u8]) -> Self {
//         let t = BigEndian::read_u32(&bytes[0..4]);
//         let q = Quat::from_bytes(&bytes[4..20]);
//         Self::new(t, q)
//     }
// }

// impl Sub for QGyroState {
//     type Output = QGyroState;
//     fn sub(self, rhs: QGyroState) -> QGyroState {
//         QGyroState {
//             t: self.t.wrapping_sub(rhs.t),
//             q: self.q - rhs.q,
//             d: true,
//         }
//     }
// }

// impl fmt::Display for QGyroState {
//     fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
//         write!(
//             f,
//             "GyroState {} {}{:#?}",
//             self.q,
//             if self.d { "+" } else { " " },
//             Duration::from_micros(self.t as u64 / 10),
//         )
//     }
// }

#[derive(PartialEq, PartialOrd, Debug, Copy, Clone)]
struct GyroState {
    n: [i8; 4],
    number: f32,
    n1: i16,
    n2: i16,
}

#[derive(PartialEq, PartialOrd, Debug, Copy, Clone)]
struct GyroStates {
    n: [GyroState; 4],
}

impl Sub for GyroState {
    type Output = GyroState;
    fn sub(self, rhs: GyroState) -> GyroState {
        GyroState {
            n: [
                self.n[0].wrapping_sub(rhs.n[0]),
                self.n[1].wrapping_sub(rhs.n[1]),
                self.n[2].wrapping_sub(rhs.n[2]),
                self.n[3].wrapping_sub(rhs.n[3]),
            ],
            // number: self.number.wrapping_sub(rhs.number),
            number: self.number - rhs.number,
            n1: self.n1.wrapping_sub(rhs.n1),
            n2: self.n2.wrapping_sub(rhs.n2),
        }
    }
}

impl Sub for GyroStates {
    type Output = GyroStates;
    fn sub(self, rhs: GyroStates) -> GyroStates {
        GyroStates {
            n: [
                self.n[0].sub(rhs.n[0]),
                self.n[1].sub(rhs.n[1]),
                self.n[2].sub(rhs.n[2]),
                self.n[3].sub(rhs.n[3]),
            ],
        }
    }
}

impl fmt::Display for GyroStates {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{:}\n{:}\n{:}\n{:}",
            self.n[0], self.n[1], self.n[2], self.n[3],
        )
    }
}

impl fmt::Display for GyroState {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "GyroState {:4} {:4} {:4} {:4} {:04x} {:6} {:+0.8}",
            self.n[0], self.n[1], self.n[2], self.n[3], self.n1, self.n2, self.number
        )
    }
}

impl<P: Peripheral + 'static> MoYuCube<P> {
    const FACES: [Face; 6] = [
        Face::Bottom,
        Face::Left,
        Face::Back,
        Face::Right,
        Face::Front,
        Face::Top,
    ];

    pub fn new(
        device: P,
        turn: Characteristic,
        gyro: Characteristic,
        read: Characteristic,
        move_listener: Box<dyn Fn(&[TimedMove], &Cube3x3x3) + Send + 'static>,
        gyro_listener: Box<dyn Fn(&[QGyroState]) + Send + 'static>,
    ) -> Result<Self> {
        let state = Arc::new(Mutex::new(Cube3x3x3::new()));
        let synced = Arc::new(Mutex::new(true));
        let synced2 = Arc::new(Mutex::new(true));

        let state_copy = state.clone();
        let synced_copy = synced.clone();
        let synced_copy2 = synced2.clone();
        let mut last_move_time = None;
        let turn_uuid = turn.uuid.clone();
        let gyro_uuid = gyro.uuid.clone();
        let mut face_rotations: [i8; 6] = [0, 0, 0, 0, 0, 0];

        let mut last_gyro_time = None;
        // let mut last_gyro_timeu: u64 = 0;
        // let mut last_gyro: Option<GyroStates> = None;
        let mut last_qgyro: Option<QGyroState> = None;

        device.on_notification(Box::new(move |value| {
            if value.uuid == gyro_uuid {
                if value.value.len() < 1 {
                    *synced_copy2.lock().unwrap() = false;
                    return;
                }

                let qgyro_state = QGyroState::from_bytes(&value.value);
                // let delta_t: u64 = (qgyro_state.t as u64).wrapping_sub(last_gyro_timeu);
                // let delta_t = if qgyro_state.t as u64 > last_gyro_timeu {
                //     qgyro_state.t as u64 - last_gyro_timeu as u64
                // } else {
                //     last_gyro_timeu = qgyro_state.t as u64;
                //     0
                // };

                if let Some(gyro) = last_qgyro {
                    let delta = qgyro_state - gyro;
                    let mut changed = false;
                    let n = delta.q.as_array();
                    for i in 0..4 {
                        if n[i].abs() > 0.01 {
                            changed = true;
                            break;
                        }
                    }
                    if changed {
                        last_qgyro = Some(qgyro_state);
                        if DEBUG_GYRO {
                            println!("Current: {}", qgyro_state);
                            println!("Delta:   {}", delta);
                        }

                        let turn = &value.value[0..4];
                        let timestamp = (((turn[1] as u32) << 24)
                            | ((turn[0] as u32) << 16)
                            | ((turn[3] as u32) << 8)
                            | (turn[2] as u32)) as f64
                            / 65536.0;

                        // There was a move, get time since last move
                        let prev_gyro_time = if let Some(time) = last_gyro_time {
                            time
                        } else {
                            timestamp
                        };
                        let time_passed = timestamp - prev_gyro_time;
                        let time_passed_ms = (time_passed * 1000.0) as u32;
                        last_gyro_time = Some(prev_gyro_time + time_passed_ms as f64 / 1000.0);

                        // Report the new move
                        // state_copy.lock().unwrap().do_move(mv);
                        gyro_listener(&[qgyro_state]);
                    }
                } else {
                    last_qgyro = Some(qgyro_state);
                }

                // if (qgyro_state.t as u64 / 1000000 - last_gyro_timeu / 1000000) > 10 {
                // if (delta_t as f64 / 100000.0) > 3.0 {
                // if (Duration::from_micros(delta_t as u64 / 10).subsec_millis()) > 2 {
                //     last_gyro_timeu = qgyro_state.t as u64;
                //     println!("{:?} {}", Duration::from_micros(delta_t as u64 / 10), qgyro_state);
                // }

                // if false {
                //     let n1 = LittleEndian::read_u16(&value.value[0..2]);
                //     let n2 = LittleEndian::read_u16(&value.value[2..4]);
                //     let tt: f32 = n1 as f32 + (n2 as f32 / 0x10000 as f32);
                //     let delta_t = tt - last_gyro_time;
                //     if delta_t < 0.25 {
                //         return;
                //     }
                //     last_gyro_time = tt;
                //     let count: usize = 20;

                //     // if let Some(gyro) = &last_gyro {
                //     //     println!("Last gyro: {:?}", gyro);
                //     // }

                //     // let mut n: [i8; 4] = [0, 0, 0, 0];
                //     // let mut number: f32 = 0.0;
                //     // let mut n1: u16 = 0;
                //     // let mut n2: i16 = 0;
                //     let gs = GyroState {
                //         n: [0, 0, 0, 0],
                //         number: 0.0,
                //         n1: 0,
                //         n2: 0,
                //     };
                //     let mut curr_gyro: GyroStates = GyroStates {
                //         n: [gs, gs, gs, gs],
                //     };
                //     for i in 1..5 {
                //         for j in 0..4 {
                //             curr_gyro.n[i - 1].n[j] = value.value[(i * 4) + j] as i8;
                //         }
                //         curr_gyro.n[i - 1].number =
                //             LittleEndian::read_f32(&value.value[(i * 4)..(i * 4) + 4]);
                //         curr_gyro.n[i - 1].n1 =
                //             LittleEndian::read_i16(&value.value[(i * 4)..(i * 4) + 2]);
                //         curr_gyro.n[i - 1].n2 =
                //             LittleEndian::read_i16(&value.value[(i * 4) + 2..(i * 4) + 4]);
                //     }

                //     // println!("\nGyro:\n{}", curr_gyro);

                //     // if let Some(gyro) = last_gyro {
                //     //     println!("\nDelta:\n{}", curr_gyro - gyro);
                //     // }

                //     // last_gyro = Some(curr_gyro);

                //     // let mut count = value.value[0];
                //     // let mut count = LittleEndian::read_u16(&value.value[0..2]);
                //     // if value.value.len() < 1 + count as usize * 6 {
                //     //     *synced_copy2.lock().unwrap() = false;
                //     //     return;
                //     // }
                //     // println!("Count {}", count);
                //     // if count > 20 {
                //     //     count = 20;
                //     // }
                //     // hexdump::hexdump(&value.value[0..count as usize]);
                //     let cfg = HexConfig {
                //         title: false,
                //         width: count,
                //         group: 0,
                //         ..HexConfig::default()
                //     };

                //     let v = &value.value[0..count as usize];

                //     if false {
                //         for i in 0..5 {
                //             let mut n: [i8; 4] = [0, 0, 0, 0];
                //             for j in 0..4 {
                //                 n[j] = value.value[(i * 4) + j] as i8;
                //             }
                //             // let number = LittleEndian::read_f32(&value.value[(i * 4)..(i * 4) + 4]);
                //             // let n1 = LittleEndian::read_i16(&value.value[(i * 4)..(i * 4) + 2]);
                //             // let n2 = LittleEndian::read_i16(&value.value[(i * 4) + 2..(i * 4) + 4]);
                //             // println!("Axis ({}) {:03} {:03} {:03} {:03} {:08x?}", i, n[0], n[1], n[2], n[3], number);
                //             // println!("Axis ({}) {:03} {:03} {:03} {:03} {:04x?} {:04x?}", i, n[0], n[1], n[2], n[3], n1, n2);
                //             if i == 0 {
                //                 // let t = LittleEndian::read_u32(&value.value[(i * 4)..(i * 4) + 4]);
                //                 // let n1 = LittleEndian::read_u16(&value.value[(i * 4)..(i * 4) + 2]);
                //                 // let n2 = LittleEndian::read_u16(&value.value[(i * 4) + 2..(i * 4) + 4]);
                //                 // let tt: f32 = n1 as f32 + (n2 as f32 / 0x10000 as f32);
                //                 // let d = Duration::from_nanos(t.into());
                //                 println!(
                //                     // "Axis ({}) {:4} {:4} {:4} {:4} {:6} {:6} {:04x?} {:04x?} {:?} {} {:0.5}",
                //                     // "Axis ({}) {:4} {:4} {:4} {:4} {:6} {:6} {:04x?} {:04x?} {:0.5}",
                //                     "\nTime: {:0.5}",
                //                     // i, n[0], n[1], n[2], n[3], n1, n2, n1, n2,
                //                     tt
                //                 );
                //             } else {
                //                 let number =
                //                     LittleEndian::read_f32(&value.value[(i * 4)..(i * 4) + 4]);
                //                 let n1 = LittleEndian::read_u16(&value.value[(i * 4)..(i * 4) + 2]);
                //                 let n2 =
                //                     LittleEndian::read_i16(&value.value[(i * 4) + 2..(i * 4) + 4]);
                //                 println!(
                //                     "Axis ({}) {:4} {:4} {:4} {:4} {:04x} {:6} {:+0.8}",
                //                     i, n[0], n[1], n[2], n[3], n1, n2, number
                //                 );
                //             }
                //         }
                //     }
                //     // for i in 0..5 {
                //     //     let number = LittleEndian::read_u32(&value.value[(i*4)..(i*4)+4]);
                //     //     println!("Number ({}) {}", i, number);
                //     // }

                //     if let Some(gyro) = last_gyro {
                //         let delta = curr_gyro - gyro;
                //         let mut changed = false;
                //         for i in 0..4 {
                //             if delta.n[i].n[2].abs() > 127
                //                 || delta.n[i].n[3] > 0
                //                 || delta.n[i].n[3] < 0
                //             {
                //                 changed = true;
                //                 break;
                //             }
                //         }
                //         if changed {
                //             println!("\nTime: {:0.5}", tt);
                //             println!("\nCurrent:\n{}", curr_gyro);
                //             println!("\nDelta:\n{}", curr_gyro - gyro);
                //             assert_eq!(config_hex(&v, cfg), format!("{:?}", v.hex_conf(cfg)));

                //             println!("{:?}", v.hex_conf(cfg));
                //             last_gyro = Some(curr_gyro);
                //         }
                //     } else {
                //         last_gyro = Some(curr_gyro);
                //     }
                // }
                return;
            } else if value.uuid == turn_uuid {
                // Get count of turn reports and check lengths
                if value.value.len() < 1 {
                    *synced_copy.lock().unwrap() = false;
                    return;
                }
                let count = value.value[0];
                if value.value.len() < 1 + count as usize * 6 {
                    *synced_copy.lock().unwrap() = false;
                    return;
                }

                // Parse each turn report
                for i in 0..count {
                    let offset = 1 + i as usize * 6;
                    let turn = &value.value[offset..offset + 6];
                    let timestamp = (((turn[1] as u32) << 24)
                        | ((turn[0] as u32) << 16)
                        | ((turn[3] as u32) << 8)
                        | (turn[2] as u32)) as f64
                        / 65536.0;
                    let face = turn[4];
                    let direction = turn[5] as i8 / 36;

                    // Decode face rotation into moves
                    let old_rotation = face_rotations[face as usize];
                    let new_rotation = old_rotation + direction;
                    face_rotations[face as usize] = (new_rotation + 9) % 9;
                    let mv = if old_rotation >= 5 && new_rotation <= 4 {
                        Some(Move::from_face_and_rotation(Self::FACES[face as usize], -1).unwrap())
                    } else if old_rotation <= 4 && new_rotation >= 5 {
                        Some(Move::from_face_and_rotation(Self::FACES[face as usize], 1).unwrap())
                    } else {
                        None
                    };

                    if let Some(mv) = mv {
                        // There was a move, get time since last move
                        let prev_move_time = if let Some(time) = last_move_time {
                            time
                        } else {
                            timestamp
                        };
                        let time_passed = timestamp - prev_move_time;
                        let time_passed_ms = (time_passed * 1000.0) as u32;
                        last_move_time = Some(prev_move_time + time_passed_ms as f64 / 1000.0);

                        // Report the new move
                        state_copy.lock().unwrap().do_move(mv);
                        move_listener(
                            &[TimedMove::new(mv, time_passed_ms)],
                            state_copy.lock().unwrap().deref(),
                        );
                    }
                }
            }
        }));
        device.subscribe(&turn)?;
        device.subscribe(&gyro)?;
        device.subscribe(&read)?;

        // We can't request state because the Bluetooth library is incompatible with
        // making writes to this device.

        Ok(Self {
            device,
            state,
            synced,
        })
    }
}

impl<P: Peripheral> BluetoothCubeDevice for MoYuCube<P> {
    fn cube_state(&self) -> Cube3x3x3 {
        self.state.lock().unwrap().clone()
    }

    fn battery_percentage(&self) -> Option<u32> {
        None
    }

    fn battery_charging(&self) -> Option<bool> {
        None
    }

    fn reset_cube_state(&self) {
        *self.state.lock().unwrap() = Cube3x3x3::new();
    }

    fn synced(&self) -> bool {
        *self.synced.lock().unwrap()
    }

    fn disconnect(&self) {
        let _ = self.device.disconnect();
    }
}

pub(crate) fn moyu_connect<P: Peripheral + 'static>(
    device: P,
    move_listener: Box<dyn Fn(&[TimedMove], &Cube3x3x3) + Send + 'static>,
    gyro_listener: Box<dyn Fn(&[QGyroState]) + Send + 'static>,
) -> Result<Box<dyn BluetoothCubeDevice>> {
    let characteristics = device.discover_characteristics()?;

    let mut turn = None;
    let mut gyro = None;
    let mut read = None;
    for characteristic in characteristics {
        if characteristic.uuid == Uuid::from_str("00001003-0000-1000-8000-00805f9b34fb").unwrap() {
            turn = Some(characteristic);
        } else if characteristic.uuid
            == Uuid::from_str("00001004-0000-1000-8000-00805f9b34fb").unwrap()
        {
            gyro = Some(characteristic);
        } else if characteristic.uuid
            == Uuid::from_str("00001002-0000-1000-8000-00805f9b34fb").unwrap()
        {
            read = Some(characteristic);
        }
    }
    if turn.is_some() && gyro.is_some() && read.is_some() {
        Ok(Box::new(MoYuCube::new(
            device,
            turn.unwrap(),
            gyro.unwrap(),
            read.unwrap(),
            move_listener,
            gyro_listener,
        )?))
    } else {
        Err(anyhow!("Unrecognized MoYu cube version"))
    }
}
