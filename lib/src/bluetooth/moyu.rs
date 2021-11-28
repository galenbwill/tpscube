use crate::bluetooth::BluetoothCubeDevice;
use crate::common::{Cube, Face, Move, TimedMove};
use crate::cube3x3x3::Cube3x3x3;
use anyhow::{anyhow, Result};
use btleplug::api::{Characteristic, Peripheral, WriteType};
use std::ops::Deref;
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use uuid::Uuid;

extern crate hexdump;
// use byteorder::{ByteOrder, LittleEndian};
// use pretty_hex::*;
// use std::fmt;
// use std::ops::{Mul, Sub};
// use std::ops::Sub;
// use std::time::Duration;

use crate::gyro::QGyroState;

const DEBUG_GYRO: bool = false;

struct MoYuCube<P: Peripheral + 'static> {
    device: P,
    state: Arc<Mutex<Cube3x3x3>>,
    synced: Arc<Mutex<bool>>,
    write: Characteristic,
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
        write: Characteristic,
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
        let mut last_qgyro: Option<QGyroState> = None;

        device.on_notification(Box::new(move |value| {
            if value.uuid == gyro_uuid {
                if value.value.len() < 1 {
                    *synced_copy2.lock().unwrap() = false;
                    return;
                }

                let qgyro_state = QGyroState::from_bytes(&value.value);

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
                        let _last_gyro_time = prev_gyro_time + time_passed_ms as f64 / 1000.0;
                        last_gyro_time = Some(_last_gyro_time);

                        gyro_listener(&[qgyro_state]);
                    }
                } else {
                    last_qgyro = Some(qgyro_state);
                }

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
            write,
        })
    }
}

impl<P: Peripheral> BluetoothCubeDevice for MoYuCube<P> {
    fn cube_state(&self) -> Cube3x3x3 {
        self.state.lock().unwrap().clone()
    }

    fn supports_gyro(&self) -> bool {
        true
    }

    fn battery_percentage(&self) -> Option<u32> {
        None
    }

    fn battery_charging(&self) -> Option<bool> {
        None
    }

    fn reset_cube_state(&self) {
        if false {
            let messages = [
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x00, 0x10,
                    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x01, 0x10,
                    0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x02, 0x10,
                    0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x03, 0x10,
                    0x6B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x04, 0x10,
                    0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x05, 0x10,
                    0xA7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x06, 0x10,
                    0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x07, 0x10,
                    0xF4, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x08, 0x10,
                    0x14, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x09, 0x10,
                    0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0A, 0x20,
                    0x5A, 0x00, 0x00, 0x00, 0x00, 0x10, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22,
                    0x22, 0x32, 0x33, 0x33, 0x33,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0A, 0x21,
                    0x33, 0x44, 0x44, 0x44, 0x44, 0x54, 0x55, 0x55, 0x55, 0x55, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0B, 0x10,
                    0x6A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0C, 0x10,
                    0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0D, 0x10,
                    0xB4, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0E, 0x50,
                    0xD8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x80, 0x3F, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0E, 0x51,
                    0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0E, 0x52,
                    0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0E, 0x53,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00,
                    0x80, 0x3F, 0x00, 0x00, 0x80,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0E, 0x54,
                    0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x0F, 0x10,
                    0xF4, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x10, 0x50,
                    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x80, 0x3F, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x10, 0x51,
                    0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x5C, 0x92, 0x61, 0xBD, 0x69, 0x00,
                    0xE6, 0xBB, 0x1A, 0xE8, 0x39,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x10, 0x52,
                    0xBC, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x3F,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x10, 0x53,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
                [
                    0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x10, 0x54,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00,
                ],
            ];
            // let message: [u8; 31] = [
            //     0x4B, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x04, 0x00, 0x52, 0x1D, 0x00, 0x00, 0x10, 0x03,
            //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            //     0x00, 0x00, 0x00,
            // ];
            for i in 0..messages.len() {
                let message = messages[i];
                let result = self
                    .device
                    .write(&self.write, &message, WriteType::WithoutResponse);
                if result.is_err() {
                    println!("Error writing message[{}]", i);
                }
            }
        }
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
    let mut write = None;
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
        } else if characteristic.uuid
            == Uuid::from_str("00001001-0000-1000-8000-00805f9b34fb").unwrap()
        {
            write = Some(characteristic);
        }
    }
    if turn.is_some() && gyro.is_some() && read.is_some() {
        Ok(Box::new(MoYuCube::new(
            device,
            turn.unwrap(),
            gyro.unwrap(),
            read.unwrap(),
            write.unwrap(),
            move_listener,
            gyro_listener,
        )?))
    } else {
        Err(anyhow!("Unrecognized MoYu cube version"))
    }
}
