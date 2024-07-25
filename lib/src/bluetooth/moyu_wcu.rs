use crate::bluetooth::{BluetoothCubeDevice, BluetoothCubeEvent};
use crate::common::{
    Color, Corner, CornerPiece, Cube, CubeFace, InitialCubeState, Move, TimedMove,
};
use crate::cube3x3x3::{Cube3x3x3, Cube3x3x3Faces, Edge3x3x3, EdgePiece3x3x3};
use aes::{
    cipher::generic_array::GenericArray,
    cipher::{BlockDecrypt, BlockEncrypt},
    Aes128, Block, NewBlockCipher,
};
use anyhow::{anyhow, Result};
use btleplug::api::{Characteristic, Peripheral, WriteType};
use num_enum::TryFromPrimitive;
// use std::borrow::Borrow;
use std::collections::HashSet;
use std::convert::{TryFrom, TryInto};
use std::iter::FromIterator;
use std::num::ParseIntError;
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use uuid::Uuid;

struct WCUCubeVersion1Characteristics {
    version: Characteristic,
    hardware: Characteristic,
    cube_state: Characteristic,
    last_moves: Characteristic,
    timing: Characteristic,
    battery: Characteristic,
}

struct WCUCubeVersion1<P: Peripheral + 'static> {
    device: P,
    state: Mutex<Cube3x3x3>,
    battery_percentage: Mutex<u32>,
    battery_charging: Mutex<bool>,
    synced: Mutex<bool>,
    last_move_count: Mutex<u8>,
    characteristics: WCUCubeVersion1Characteristics,
    cipher: WCUCubeVersion1Cipher,
    move_listener: Box<dyn Fn(BluetoothCubeEvent) + Send + 'static>,
}

struct WCUCubeVersion1Cipher {
    device_key: [u8; 16],
}

struct WCUCubeVersion2<P: Peripheral + 'static> {
    device: P,
    state: Arc<Mutex<Cube3x3x3>>,
    battery_percentage: Arc<Mutex<Option<u32>>>,
    battery_charging: Arc<Mutex<Option<bool>>>,
    synced: Arc<Mutex<bool>>,
    write: Characteristic,
    cipher: WCUCubeVersion2Cipher,
}

#[derive(Clone)]
struct WCUCubeVersion2Cipher {
    device_key: [u8; 16],
    device_iv: [u8; 16],
}

struct WCUSmartTimer<P: Peripheral + 'static> {
    device: P,
}

const DEBUG_CORNERS: bool = false;
const DEBUG_EDGES: bool = false;

impl<P: Peripheral> WCUCubeVersion1<P> {
    const LAST_MOVE_COUNT_OFFSET: usize = 12;
    const LAST_MOVE_LIST_OFFSET: usize = 13;

    pub fn new(
        device: P,
        characteristics: WCUCubeVersion1Characteristics,
        move_listener: Box<dyn Fn(BluetoothCubeEvent) + Send + 'static>,
        minor_version: u8,
    ) -> Result<Self> {
        // Read device identifier, this is used to derive the key
        // println!("reading device id...");
        let device_id = device.read(&characteristics.hardware)?;
        if device_id.len() < 6 {
            return Err(anyhow!("Device identifier invalid"));
        }
        // println!("device_id: {:?}", device_id);

        // Derive the key
        const WCU_V1_KEYS: [[u8; 16]; 2] = [
            [
                0xc6, 0xca, 0x15, 0xdf, 0x4f, 0x6e, 0x13, 0xb6, 0x77, 0x0d, 0xe6, 0x59, 0x3a, 0xaf,
                0xba, 0xa2,
            ],
            [
                0x43, 0xe2, 0x5b, 0xd6, 0x7d, 0xdc, 0x78, 0xd8, 0x07, 0x60, 0xa3, 0xda, 0x82, 0x3c,
                0x01, 0xf1,
            ],
        ];
        let mut key = WCU_V1_KEYS[minor_version as usize].clone();
        for i in 0..6 {
            key[i] = key[i].wrapping_add(device_id[5 - i]);
        }
        let cipher = WCUCubeVersion1Cipher { device_key: key };

        // Get initial cube state
        // println!("reading initial cube state...");
        let state = device.read(&characteristics.cube_state)?;
        // println!(
        //     "characteristics.cube_state {:?}",
        //     characteristics.cube_state
        // );
        if state.len() < 18 {
            return Err(anyhow!("Cube state is invalid"));
        }
        let state = cipher.decrypt(&state)?;
        let state = Self::decode_cube_state(&state)?;
        let state = Mutex::new(state);

        // Get the initial move count
        let moves = device.read(&characteristics.last_moves)?;
        if moves.len() < 19 {
            return Err(anyhow!("Invalid last move data"));
        }
        let moves = cipher.decrypt(&moves)?;
        let last_move_count = Mutex::new(moves[Self::LAST_MOVE_COUNT_OFFSET]);

        // Get battery state
        let battery = device.read(&characteristics.battery)?;
        if battery.len() < 8 {
            return Err(anyhow!("Battery state is invalid"));
        }
        let battery = cipher.decrypt(&battery)?;
        let battery_percentage = battery[7];
        let battery_charging = battery[6] != 0;

        Ok(WCUCubeVersion1 {
            device,
            state,
            battery_percentage: Mutex::new(battery_percentage as u32),
            battery_charging: Mutex::new(battery_charging),
            synced: Mutex::new(true),
            last_move_count,
            characteristics,
            cipher,
            move_listener,
        })
    }

    fn decode_cube_state(data: &[u8]) -> Result<Cube3x3x3> {
        const FACES: [CubeFace; 6] = [
            CubeFace::Top,
            CubeFace::Right,
            CubeFace::Front,
            CubeFace::Bottom,
            CubeFace::Left,
            CubeFace::Back,
        ];
        const COLORS: [Color; 6] = [
            Color::White,
            Color::Red,
            Color::Green,
            Color::Yellow,
            Color::Orange,
            Color::Blue,
        ];
        let mut state: [Color; 6 * 9] = [Color::White; 6 * 9];

        for face in 0..6 {
            // Find face index in our representation using mapping
            let target_face_idx = FACES[face] as u8 as usize;
            state[target_face_idx * 9 + 1 * 3 + 1] = COLORS[face];

            // Decode face's data from buffer
            let face_data = ((data[(face * 3) ^ 1] as u32) << 16)
                | ((data[((face * 3) + 1) ^ 1] as u32) << 8)
                | data[((face * 3) + 2) ^ 1] as u32;

            // Place colors into the cube state array
            let mut offset = target_face_idx * 9;
            for i in 0..8 {
                if i == 4 {
                    // Skip center, not represented in data
                    offset += 1;
                }

                let color_idx = (face_data >> (3 * (7 - i))) & 7;
                if color_idx >= 6 {
                    return Err(anyhow!("Invalid cube state"));
                }

                state[offset] = COLORS[color_idx as usize];
                offset += 1;
            }
        }

        // Create cube state and convert to normal format
        Ok(Cube3x3x3Faces::from_colors(state).as_pieces())
    }

    fn move_poll(&self) -> Result<()> {
        if !*self.synced.lock().unwrap() {
            // Not synced, do not try to poll moves
            return Err(anyhow!("Not synced"));
        }

        // Read move data and move timing data
        let move_data = self.device.read(&self.characteristics.last_moves)?;
        if move_data.len() < 19 {
            return Err(anyhow!("Invalid last move data"));
        }
        let move_data = self.cipher.decrypt(&move_data)?;

        let timing = self.device.read(&self.characteristics.timing)?;
        if timing.len() < 19 {
            return Err(anyhow!("Invalid timing data"));
        }
        let timing = self.cipher.decrypt(&timing)?;

        // Check number of moves since last message.
        let current_move_count = move_data[Self::LAST_MOVE_COUNT_OFFSET];
        let timestamp_move_count = timing[0];
        let mut last_move_count = self.last_move_count.lock().unwrap();
        let move_count = current_move_count.wrapping_sub(*last_move_count) as usize;
        if move_count > 6 {
            // There are too many moves since the last message. Our cube
            // state is out of sync. Let the client know and reset the
            // last move count such that we don't parse any more move
            // messages, since they aren't valid anymore.
            return Err(anyhow!("Move buffer exceeded"));
        }

        // Gather the moves
        let mut moves = Vec::with_capacity(move_count);
        for j in 0..move_count {
            let i = (6 - move_count) + j;

            // Decode move data
            let move_num = move_data[Self::LAST_MOVE_LIST_OFFSET + i] as usize;

            let timestamp_idx = (*last_move_count)
                .wrapping_add(j as u8)
                .wrapping_sub(timestamp_move_count.wrapping_sub(9));
            if timestamp_idx >= 9 {
                return Err(anyhow!("Timestamp not present for move"));
            }
            let move_time = timing[timestamp_idx as usize * 2 + 1] as u32
                | ((timing[timestamp_idx as usize * 2 + 2] as u32) << 8);

            const MOVES: &[Move] = &[
                Move::U,
                Move::U2,
                Move::Up,
                Move::R,
                Move::R2,
                Move::Rp,
                Move::F,
                Move::F2,
                Move::Fp,
                Move::D,
                Move::D2,
                Move::Dp,
                Move::L,
                Move::L2,
                Move::Lp,
                Move::B,
                Move::B2,
                Move::Bp,
            ];
            if move_num >= MOVES.len() {
                return Err(anyhow!("Invalid move"));
            }
            let mv = MOVES[move_num];
            moves.push(TimedMove::new(mv, move_time));

            // Apply move to the cube state.
            self.state.lock().unwrap().do_move(mv);
        }

        *last_move_count = current_move_count;

        if moves.len() != 0 {
            // Let clients know there is a new move
            let move_listener = self.move_listener.as_ref();
            move_listener(BluetoothCubeEvent::Move(
                moves,
                self.state.lock().unwrap().clone(),
            ));
        }

        Ok(())
    }
}

impl<P: Peripheral> BluetoothCubeDevice for WCUCubeVersion1<P> {
    fn cube_state(&self) -> Cube3x3x3 {
        self.state.lock().unwrap().clone()
    }

    fn battery_percentage(&self) -> Option<u32> {
        Some(*self.battery_percentage.lock().unwrap())
    }

    fn battery_charging(&self) -> Option<bool> {
        Some(*self.battery_charging.lock().unwrap())
    }

    fn reset_cube_state(&self) {
        // These bytes represent the cube state in the solved state.
        let message: [u8; 18] = [
            0x00, 0x00, 0x24, 0x00, 0x49, 0x92, 0x24, 0x49, 0x6d, 0x92, 0xdb, 0xb6, 0x49, 0x92,
            0xb6, 0x24, 0x6d, 0xdb,
        ];
        let _ = self.device.write(
            &self.characteristics.cube_state,
            &message,
            WriteType::WithResponse,
        );

        *self.state.lock().unwrap() = Cube3x3x3::new();
    }

    fn synced(&self) -> bool {
        *self.synced.lock().unwrap()
    }

    fn update(&self) {
        if let Err(_) = self.move_poll() {
            *self.synced.lock().unwrap() = false;
        }
    }

    fn disconnect(&self) {
        let _ = self.device.disconnect();
    }

    // Older WCU cubes have *very* uncalibrated clocks
    fn estimated_clock_ratio(&self) -> f64 {
        0.95
    }

    fn clock_ratio_range(&self) -> (f64, f64) {
        (0.9, 1.02)
    }
}

impl WCUCubeVersion1Cipher {
    fn decrypt(&self, value: &[u8]) -> Result<Vec<u8>> {
        if value.len() <= 16 {
            return Err(anyhow!("Packet size less than expected length"));
        }

        // Packets are larger than block size. First decrypt the last 16 bytes
        // of the packet in place.
        let mut value = value.to_vec();
        let aes = Aes128::new(GenericArray::from_slice(&self.device_key));
        let offset = value.len() - 16;
        let end_cipher = &value[offset..];
        let mut end_plain = Block::clone_from_slice(end_cipher);
        aes.decrypt_block(&mut end_plain);
        for i in 0..16 {
            value[offset + i] = end_plain[i];
        }

        // Decrypt the first 16 bytes of the packet in place. This will overlap
        // with the decrypted block above.
        let start_cipher = &value[0..16];
        let mut start_plain = Block::clone_from_slice(start_cipher);
        aes.decrypt_block(&mut start_plain);
        for i in 0..16 {
            value[i] = start_plain[i];
        }

        Ok(value)
    }
}

impl<P: Peripheral> WCUCubeVersion2<P> {
    const CUBE_INFO_MESSAGE: u8 = 161; // 2;
    const CUBE_STATE_MESSAGE: u8 = 163; // 4;
    const BATTERY_STATE_MESSAGE: u8 = 164; // 9;
    const CUBE_MOVES_MESSAGE: u8 = 165; // 2;
    const RESET_CUBE_STATE_MESSAGE: u8 = 10;
    const CUBE_GYRO_MESSAGE: u8 = 171; // 2;

    const CUBE_STATE_TIMEOUT_MS: usize = 2000;

    pub fn new(
        device: P,
        read: Characteristic,
        write: Characteristic,
        move_listener: Box<dyn Fn(BluetoothCubeEvent) + Send + 'static>,
    ) -> Result<Self> {
        // Derive keys. These are based on a 6 byte device identifier found in the
        // manufacturer data.
        let mac = if let Some(device_name) = device.properties().local_name {
            let prefix = "CF301600".to_owned();
            let cube_id = device_name[9..13].to_owned();
            let result = prefix + &cube_id;
            result

        } else {
            return Err(anyhow!("Manufacturer data missing device identifier"));
        };
        let mac_data: Result<Vec<u8>, ParseIntError> = (0..mac.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&mac[i..i + 2], 16))
            .collect();
        let device_key = mac_data.ok().unwrap();

        const WCU_V2_KEY: [u8; 16] = [
            0x15, 0x77, 0x3a, 0x5c, 0x67, 0xe, 0x2d, 0x1f, 0x17, 0x67, 0x2a, 0x13, 0x9b, 0x67,
            0x52, 0x57,
        ];
        const WCU_V2_IV: [u8; 16] = [
            0x11, 0x23, 0x26, 0x25, 0x86, 0x2a, 0x2c, 0x3b, 0x55, 0x6, 0x7f, 0x31, 0x7e, 0x67,
            0x21, 0x57,
        ];
        let mut key = WCU_V2_KEY.clone();
        let mut iv = WCU_V2_IV.clone();
        // for (idx, byte) in device_key.iter().enumerate() {
        for idx in 0..device_key.len() {
            key[idx] = ((key[idx] as u16 + device_key[5 - idx] as u16) % 255) as u8;
            iv[idx] = ((iv[idx] as u16 + device_key[5 - idx] as u16) % 255) as u8;
        }
        let cipher: WCUCubeVersion2Cipher = WCUCubeVersion2Cipher {
            device_key: key,
            device_iv: iv,
        };

        let state = Arc::new(Mutex::new(Cube3x3x3::new()));
        let state_set = Arc::new(Mutex::new(false));
        let battery_percentage = Arc::new(Mutex::new(None));
        let battery_charging = Arc::new(Mutex::new(None));
        let last_move_count = Mutex::new(None);
        let synced = Arc::new(Mutex::new(true));

        let cipher_copy = cipher.clone();
        let state_copy = state.clone();
        let state_set_copy = state_set.clone();
        let battery_percentage_copy = battery_percentage.clone();
        let _battery_charging_copy = battery_charging.clone();
        let synced_copy = synced.clone();

        device.on_notification(Box::new(move |value| {
            // println!("on_notification: {:?}", &value.value);
            if let Ok(value) = cipher_copy.decrypt(&value.value) {
                // let message_type = Self::extract_bits(&value, 0, 4) as u8;
                let message_type = value[0];
                // println!("message_type: {}", message_type);
                match message_type {
                    Self::CUBE_GYRO_MESSAGE => {
                        // println!("rx: CUBE_GYRO_MESSAGE");
                    }
                    Self::CUBE_INFO_MESSAGE => {
                        // println!("rx: CUBE_INFO_MESSAGE");
                        // println!("decrypted: {:?}", value);
                    }
                    Self::CUBE_MOVES_MESSAGE => {
                        // println!("rx: CUBE_MOVES_MESSAGE");
                        // println!("decrypted: {:?}", value);
                        let current_move_count = Self::extract_bits(&value, 88, 8) as u8;

                        // println!("current_move_count: {}", current_move_count);
                        // If we haven't received a cube state message yet, we can't know what
                        // the curent cube state is. Ignore moves until the cube state message
                        // is received. If there has been a cube state message, we will have
                        // a last move count and we can continue.
                        let mut last_move_count_option = last_move_count.lock().unwrap();
                        if let Some(last_move_count) = *last_move_count_option {
                            // Check number of moves since last message.
                            let move_count =
                                current_move_count.wrapping_sub(last_move_count) as usize;
                            // println!("last_move_count: {}", last_move_count);
                            // println!("move_count: {}", move_count);
                            if move_count > 7 {
                                // There are too many moves since the last message. Our cube
                                // state is out of sync. Let the client know and reset the
                                // last move count such that we don't parse any more move
                                // messages, since they aren't valid anymore.
                                *synced_copy.lock().unwrap() = false;
                                *last_move_count_option = None;
                                return;
                            }

                            // Gather the moves
                            let mut moves = Vec::with_capacity(move_count);
                            for j in 0..move_count {
                                // Build move list in reverse order. In the packet the moves
                                // are from the latest move to the oldest move, but the callback
                                // should take the moves in the order they happened.
                                let i = (move_count - 1) - j;

                                // Decode move data
                                let move_num = Self::extract_bits(&value, 96 + i * 5, 5) as usize;
                                let move_time = Self::extract_bits(&value, 8 + i * 16, 16);
                                const MOVES: &[Move] = &[
                                    Move::F,
                                    Move::Fp,
                                    Move::B,
                                    Move::Bp,
                                    Move::U,
                                    Move::Up,
                                    Move::D,
                                    Move::Dp,
                                    Move::L,
                                    Move::Lp,
                                    Move::R,
                                    Move::Rp,
                                ];
                                if move_num >= MOVES.len() {
                                    // Bad move data. Cube is now desynced.
                                    *synced_copy.lock().unwrap() = false;
                                    *last_move_count_option = None;
                                    return;
                                }
                                let mv = MOVES[move_num];
                                moves.push(TimedMove::new(mv, move_time));

                                // Apply move to the cube state.
                                state_copy.lock().unwrap().do_move(mv);
                            }

                            *last_move_count_option = Some(current_move_count);

                            // println!("moves: {:?}", moves.clone());

                            if moves.len() != 0 {
                                // Let clients know there is a new move
                                move_listener(BluetoothCubeEvent::Move(
                                    moves,
                                    state_copy.lock().unwrap().clone(),
                                ));
                            }
                        } else {
                        }
                    }
                    Self::CUBE_STATE_MESSAGE => {
                        // println!("rx: CUBE_STATE_MESSAGE");
                        // println!("decrypted: {:?}", value);
                        *last_move_count.lock().unwrap() =
                            Some(Self::extract_bits(&value, 152, 8) as u8);
                        let latest_facelet = &value[1..20];
                        // let latest_facelet: [u8; 20] = {
                        //     let values =
                        //     // Some(Self::extract_bits(&value, 8, 152));
                        //     &value[1..20];
                        //     let mut result: Vec<u8> = Vec::new();
                        //     result
                        // };
                        let faces = [2, 5, 0, 3, 4, 1];
                        let mut cur_state: Vec<u8> = Vec::new();
                        let mut state_faces = [[[' '; 3]; 3]; 6];
                        for i in 0..6 {
                            // let face = Self::extract_bits(&latestFacelet, faces[i] * 24, 24) as u32;
                            let face =
                                &latest_facelet[((faces[i] * 24) / 8)..((24 + faces[i] * 24) / 8)];
                            for j in 0..8 {
                                let f = Self::extract_bits(&face, j * 3, 3) as usize;
                                cur_state.push("FBUDLR".as_bytes()[f]);
                                // state.push("FBUDLR"[])
                                if j == 3 {
                                    cur_state.push("FBUDLR".as_bytes()[faces[i]]);
                                }
                            }
                        }
                        let cur_state_str = String::from_utf8(cur_state).expect("URFDLB");
                        for i in 0..6 {
                            for j in 0..3 {
                                for k in 0..3 {
                                    state_faces[i][j][k] =
                                        cur_state_str.as_bytes()[i * 9 + j * 3 + k] as char;
                                }
                            }
                        }
                        // println!("cur_state: {:?}", cur_state_str);
                        // println!("cur_state_faces: {:?}", state_faces);

                        // Set up corner and edge state
                        let mut corners = [0; 8];
                        let mut corner_twist = [0; 8];
                        let mut corners_left: HashSet<u32> =
                            HashSet::from_iter((&[0, 1, 2, 3, 4, 5, 6, 7]).iter().cloned());
                        let mut edges = [0; 12];
                        let mut edge_parity = [0; 12];
                        let edges_initial = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];
                        let mut edges_left: HashSet<u32> = HashSet::from_iter(
                            (&[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]).iter().cloned(),
                        );
                        let mut total_corner_twist = 0;
                        let mut total_edge_parity = 0;

                        let edge_indices: [usize; 12 * 3 * 2] = [
                            0, 1, 2,  1, 0, 1, // UR = 0,
                            0, 2, 1,  2, 0, 1, // UF = 1,
                            0, 1, 0,  4, 0, 1, // UL = 2,
                            0, 0, 1,  5, 0, 1, // UB = 3,
                            3, 1, 2,  1, 2, 1, // DR = 4,
                            3, 0, 1,  2, 2, 1, // DF = 5,
                            3, 1, 0,  4, 2, 1, // DL = 6,
                            3, 2, 1,  5, 2, 1, // DB = 7,
                            2, 1, 2,  1, 1, 0, // FR = 8,
                            2, 1, 0,  4, 1, 2, // FL = 9,
                            5, 1, 2,  4, 1, 0, // BL = 10,
                            5, 1, 0,  1, 1, 2, // BR = 11,
                        ];

                        let face_indices = [
                            0, 2, 2,  1, 0, 0,  2, 0, 2, // URF
                            0, 2, 0,  2, 0, 0,  4, 0, 2, // UFL
                            0, 0, 0,  4, 0, 0,  5, 0, 2, // ULB
                            0, 0, 2,  5, 0, 0,  1, 0, 2, // UBR
                            3, 0, 2,  2, 2, 2,  1, 2, 0, // DFR
                            3, 0, 0,  4, 2, 2,  2, 2, 0, // DLF
                            3, 2, 0,  5, 2, 2,  4, 2, 0, // DBL
                            3, 2, 2,  1, 2, 2,  5, 2, 0, // DRB
                        ];
                        if DEBUG_CORNERS {
                            for y in 0..3 {
                                for f in 0..6 {
                                    print!("{:?}    ", state_faces[f][y])
                                }
                                println!();
                            }
                        }
                        let fi = face_indices;
                        let sf = state_faces;
                        // let names = "URFDLB".as_bytes();
                        // let mut vcorners: Vec<Corner> = Vec::new();
                        // Decode corners.
                        for i in 0..8 {
                            // corners[i] = Self::extract_bits(&value, 12 + i * 3, 3);
                            // corner_twist[i] = Self::extract_bits(&value, 33 + i * 2, 2);
                            // total_corner_twist += corner_twist[i];
                            // if !corners_left.remove(&corners[i]) || corner_twist[i] >= 3 {
                            //     return;
                            // }
                            let j = i * 9;
                            let mut corner_str = String::from("");
                            for k in 0..3 {
                                let b = j + 3 * k;
                                let sff = sf[fi[b]];
                                let foo = sff[fi[b + 1]][fi[b + 2]];
                                // s.extend(names[foo] as char);
                                if corner_str.len() > 0 && foo == 'U' || foo == 'D' {
                                    corner_twist[i] = k;
                                    let mut ss = String::from("");
                                    ss.extend([foo]);
                                    ss += &corner_str;
                                    corner_str = ss;
                                }
                                else {
                                    corner_str.extend([foo]);
                                }
                            }
                            if i < 7 {
                                total_corner_twist += corner_twist[i];
                            }
                            // corner_twist[i] = 1;
                            corner_str = String::from(match corner_str.as_str() {
                                "UFR" => "URF",
                                "ULF" => "UFL",
                                "UBL" => "ULB",
                                "URB" => "UBR",
                                "DRF" => "DFR",
                                "DFL" => "DLF",
                                "DLB" => "DBL",
                                "DBR" => "DRB",
                                _ => &corner_str
                            });
                            if DEBUG_CORNERS {
                                println!("corner[{}]: {} twist: {} ", i, &corner_str, corner_twist[i]);
                            }
                            // if i == 2 {
                            //     println!();
                            // }

                            // if corner[0] != 'U' && corner[0] != 'D' {
                            //     if corner.as_bytes()[2] as char == 'U' || corner[2] == 'D' {
                            //         corner = corner[2] + corner[0] + corner[1];
                            //     }
                            // }

                            let corner = Corner::from_str(&corner_str).expect("Invalid corner");
                            // vcorners.push(corner);
                            corners[i] = corner as u32;
                            if !corners_left.remove(&corners[i]) || corner_twist[i] >= 3 {
                                return;
                            }
                        }
                        // println!("Corners: {:?}", vcorners.clone());

                        // Decode edges.
                        for i in 0..12 {

                            // edges[i] = edges_initial[i];
                            // edges[i] = Self::extract_bits(&value, 47 + i * 4, 4);
                            let mut s: String = String::from("");
                            let mut face = i * 6;
                            let mut first = state_faces[edge_indices[face]][edge_indices[face+1]][edge_indices[face+2]];
                            face += 3;
                            let mut second = state_faces[edge_indices[face]][edge_indices[face+1]][edge_indices[face+2]];
                            let mut parity: u32 = 0;
                            if first == 'U' || first == 'D' {

                            }
                            else if second == 'U' || second == 'D' {
                                let t = second;
                                second = first;
                                first = t;
                                parity = 1;
                            }
                            else if first == 'F' || first == 'B' {
                                
                            }
                            else if second == 'F' || second == 'B' {
                                assert!(first == 'L' || first == 'R', "Invalid edge");
                                let t = second;
                                second = first;
                                first = t;
                                parity = 1;
                            }
                            else {
                                return;
                            }
                            s.extend([first, second]);
                            if DEBUG_EDGES {
                                println!("edge[{}]: {} parity: {}", i, s, parity);
                            }
                            let edge = Edge3x3x3::from_str(&s).expect("Invalid edge");
                            edges[i] = edge as u32;
                            edge_parity[i] = parity;
                            if i < 11 {
                                total_edge_parity += edge_parity[i];
                            }
                            if !edges_left.remove(&edges[i]) || edge_parity[i] >= 2 {
                                return;
                            }
                        }

                        if (corner_twist[7] != (3 - total_corner_twist % 3) % 3)
                            || (edge_parity[11] != total_edge_parity & 1)
                        {
                            return;
                        }

                        // Create cube state. Our representation of the cube state matches
                        // the one used in the packet. We have already verified the data
                        // is valid so we can unwrap the conversions with panic.
                        let mut corner_pieces = Vec::with_capacity(8);
                        let mut edge_pieces = Vec::with_capacity(12);
                        for i in 0..8 {
                            corner_pieces.push(CornerPiece {
                                piece: Corner::try_from(corners[i] as u8).unwrap(),
                                orientation: corner_twist[i] as u8,
                            });
                        }
                        for i in 0..12 {
                            edge_pieces.push(EdgePiece3x3x3 {
                                piece: Edge3x3x3::try_from(edges[i] as u8).unwrap(),
                                orientation: edge_parity[i] as u8,
                            });
                        }

                        let cube = Cube3x3x3::from_corners_and_edges(
                            corner_pieces.try_into().unwrap(),
                            edge_pieces.try_into().unwrap(),
                        );

                        // println!("stste set: {:?}", cube.clone());
                        *state_copy.lock().unwrap() = cube;
                        *state_set_copy.lock().unwrap() = true;
                    }
                    Self::BATTERY_STATE_MESSAGE => {
                        // println!("rx: BATTERY_STATE_MESSAGE");
                        // println!("decrypted: {:?}", value);
                        *battery_percentage_copy.lock().unwrap() =
                            Some(Self::extract_bits(&value, 8, 8));
                    }
                    _ => (),
                }
            }
        }));
        device.subscribe(&read)?;

        // Request battery state immediately
        let mut message: [u8; 20] = [0; 20];
        message[0] = Self::CUBE_INFO_MESSAGE;
        let message = cipher.encrypt(&message)?;
        device.write(&write, &message, WriteType::WithResponse)?;

        // Request initial cube state
        let mut loop_count = 0;
        loop {
            let mut message: [u8; 20] = [0; 20];
            message[0] = Self::CUBE_STATE_MESSAGE;
            let message = cipher.encrypt(&message)?;
            device.write(&write, &message, WriteType::WithResponse)?;

            std::thread::sleep(Duration::from_millis(200));

            if *state_set.lock().unwrap() {
                break;
            }

            loop_count += 1;
            if loop_count > Self::CUBE_STATE_TIMEOUT_MS / 200 {
                return Err(anyhow!("Did not receive initial cube state"));
            }
        }

        // Request battery state immediately
        let mut message: [u8; 20] = [0; 20];
        message[0] = Self::BATTERY_STATE_MESSAGE;
        let message = cipher.encrypt(&message)?;
        device.write(&write, &message, WriteType::WithResponse)?;

        Ok(Self {
            device,
            state,
            battery_percentage,
            battery_charging,
            synced,
            cipher,
            write,
        })
    }

    fn extract_bits(data: &[u8], start: usize, count: usize) -> u32 {
        let mut result = 0;
        for i in 0..count {
            let bit = start + i;
            result <<= 1;
            if data[bit / 8] & (1 << (7 - (bit % 8))) != 0 {
                result |= 1;
            }
        }
        result
    }
}

impl WCUCubeVersion2Cipher {
    fn decrypt(&self, value: &[u8]) -> Result<Vec<u8>> {
        // println!("decrypt: {:?}", value);
        if value.len() <= 16 {
            return Err(anyhow!("Packet size less than expected length"));
        }

        // Packets are larger than block size. First decrypt the last 16 bytes
        // of the packet in place.
        let mut value = value.to_vec();
        let aes = Aes128::new(GenericArray::from_slice(&self.device_key));
        let offset = value.len() - 16;
        let end_cipher = &value[offset..];
        let mut end_plain = Block::clone_from_slice(end_cipher);
        aes.decrypt_block(&mut end_plain);
        for i in 0..16 {
            end_plain[i] ^= !!self.device_iv[i];
            value[offset + i] = end_plain[i];
        }

        // Decrypt the first 16 bytes of the packet in place. This will overlap
        // with the decrypted block above.
        let start_cipher = &value[0..16];
        let mut start_plain = Block::clone_from_slice(start_cipher);
        aes.decrypt_block(&mut start_plain);
        for i in 0..16 {
            start_plain[i] ^= !!self.device_iv[i];
            value[i] = start_plain[i];
        }

        Ok(value)
    }

    fn encrypt(&self, value: &[u8]) -> Result<Vec<u8>> {
        if value.len() <= 16 {
            return Err(anyhow!("Packet size less than expected length"));
        }

        // Packets are larger than block size. First encrypt the first 16 bytes
        // of the packet in place.
        let mut value = value.to_vec();
        for i in 0..16 {
            value[i] ^= !!self.device_iv[i];
        }
        let mut cipher = Block::clone_from_slice(&value[0..16]);
        let aes = Aes128::new(GenericArray::from_slice(&self.device_key));
        aes.encrypt_block(&mut cipher);
        for i in 0..16 {
            value[i] = cipher[i];
        }

        // Decrypt the last 16 bytes of the packet in place. This will overlap
        // with the decrypted block above.
        let offset = value.len() - 16;
        for i in 0..16 {
            value[offset + i] ^= !!self.device_iv[i];
        }
        let mut cipher = Block::clone_from_slice(&value[offset..]);
        aes.encrypt_block(&mut cipher);
        for i in 0..16 {
            value[offset + i] = cipher[i];
        }

        Ok(value)
    }
}

impl<P: Peripheral> BluetoothCubeDevice for WCUCubeVersion2<P> {
    fn cube_state(&self) -> Cube3x3x3 {
        self.state.lock().unwrap().clone()
    }

    fn battery_percentage(&self) -> Option<u32> {
        *self.battery_percentage.lock().unwrap()
    }

    fn battery_charging(&self) -> Option<bool> {
        *self.battery_charging.lock().unwrap()
    }

    fn reset_cube_state(&self) {
        // These bytes represent the cube state in the solved state.
        let message: [u8; 20] = [
            Self::RESET_CUBE_STATE_MESSAGE,
            0x05,
            0x39,
            0x77,
            0x00,
            0x00,
            0x01,
            0x23,
            0x45,
            0x67,
            0x89,
            0xab,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ];
        let message = self.cipher.encrypt(&message).unwrap();
        let _ = self
            .device
            .write(&self.write, &message, WriteType::WithResponse);

        *self.state.lock().unwrap() = Cube3x3x3::new();
    }

    fn synced(&self) -> bool {
        *self.synced.lock().unwrap()
    }

    fn disconnect(&self) {
        let _ = self.device.disconnect();
    }
}

impl<P: Peripheral> WCUSmartTimer<P> {
    pub fn new(
        device: P,
        updates: Characteristic,
        move_listener: Box<dyn Fn(BluetoothCubeEvent) + Send + 'static>,
    ) -> Result<Self> {
        device.on_notification(Box::new(move |value| {
            if value.value.len() >= 4 {
                match value.value[3] {
                    1 => move_listener(BluetoothCubeEvent::TimerReady),
                    2 => move_listener(BluetoothCubeEvent::TimerStartCancel),
                    3 => move_listener(BluetoothCubeEvent::TimerStarted),
                    4 => {
                        if value.value.len() >= 8 {
                            let min = value.value[4] as u32;
                            let sec = value.value[5] as u32;
                            let msec = ((value.value[7] as u32) << 8) | (value.value[6] as u32);
                            move_listener(BluetoothCubeEvent::TimerFinished(
                                min * 60000 + sec * 1000 + msec,
                            ));
                        }
                    }
                    6 => move_listener(BluetoothCubeEvent::HandsOnTimer),
                    _ => (),
                }
            }
        }));
        device.subscribe(&updates)?;

        Ok(WCUSmartTimer { device })
    }
}

impl<P: Peripheral> BluetoothCubeDevice for WCUSmartTimer<P> {
    fn timer_only(&self) -> bool {
        true
    }

    fn cube_state(&self) -> Cube3x3x3 {
        Cube3x3x3::new()
    }

    fn battery_percentage(&self) -> Option<u32> {
        None
    }

    fn battery_charging(&self) -> Option<bool> {
        None
    }

    fn reset_cube_state(&self) {}
    fn update(&self) {}

    fn synced(&self) -> bool {
        true
    }

    fn disconnect(&self) {
        let _ = self.device.disconnect();
    }
}

pub(crate) fn moyu_wcu_cube_connect<P: Peripheral + 'static>(
    device: P,
    move_listener: Box<dyn Fn(BluetoothCubeEvent) + Send + 'static>,
) -> Result<Box<dyn BluetoothCubeDevice>> {
    let characteristics = device.discover_characteristics()?;

    // Find characteristics for communicating with the cube. There are two different
    // versions of the WCU cubes with different characteristics.
    let mut v1_version = None;
    let mut v1_hardware = None;
    let mut v1_cube_state = None;
    let mut v1_last_moves = None;
    let mut v1_timing = None;
    let mut v1_battery = None;
    let mut v2_write = None;
    let mut v2_read = None;
    for characteristic in characteristics {
        // println!("Characteristic: {:?}", characteristic);
        if characteristic.uuid == Uuid::from_str("00002a28-0000-1000-8000-00805f9b34fb").unwrap() {
            v1_version = Some(characteristic);
        } else if characteristic.uuid
            == Uuid::from_str("00002a23-0000-1000-8000-00805f9b34fb").unwrap()
        {
            v1_hardware = Some(characteristic);
        } else if characteristic.uuid
            == Uuid::from_str("0000fff2-0000-1000-8000-00805f9b34fb").unwrap()
        {
            v1_cube_state = Some(characteristic);
        } else if characteristic.uuid
            == Uuid::from_str("0000fff5-0000-1000-8000-00805f9b34fb").unwrap()
        {
            v1_last_moves = Some(characteristic);
        } else if characteristic.uuid
            == Uuid::from_str("0000fff6-0000-1000-8000-00805f9b34fb").unwrap()
        {
            v1_timing = Some(characteristic);
        } else if characteristic.uuid
            == Uuid::from_str("0000fff7-0000-1000-8000-00805f9b34fb").unwrap()
        {
            v1_battery = Some(characteristic);
        // } else if characteristic.uuid
        //     == Uuid::from_str("28be4a4a-cd67-11e9-a32f-2a2ae2dbcce4").unwrap()
        // {
        //     v2_write = Some(characteristic);
        // } else if characteristic.uuid
        //     == Uuid::from_str("28be4cb6-cd67-11e9-a32f-2a2ae2dbcce4").unwrap()
        // {
        //     v2_read = Some(characteristic);
        } else if characteristic.uuid
            == Uuid::from_str("0783b03e-7735-b5a0-1760-a305d2795cb2").unwrap()
        {
            v2_write = Some(characteristic);
        } else if characteristic.uuid
            == Uuid::from_str("0783b03e-7735-b5a0-1760-a305d2795cb1").unwrap()
        {
            v2_read = Some(characteristic);
            // } else if characteristic.uuid
            //     == Uuid::from_str("0783B03E-7735-B5A0-1760-A305D2795CB2").unwrap()
            // {
            //     v2_write = Some(characteristic);
            // } else if characteristic.uuid
            //     == Uuid::from_str("0783B03E-7735-B5A0-1760-A305D2795CB1").unwrap()
            // {
            //     v2_read = Some(characteristic);
        }
    }

    // Create cube object based on available characteristics
    if v1_version.is_some()
        && v1_hardware.is_some()
        && v1_cube_state.is_some()
        && v1_last_moves.is_some()
        && v1_timing.is_some()
        && v1_battery.is_some()
    {
        let characteristics = WCUCubeVersion1Characteristics {
            version: v1_version.unwrap(),
            hardware: v1_hardware.unwrap(),
            cube_state: v1_cube_state.unwrap(),
            last_moves: v1_last_moves.unwrap(),
            timing: v1_timing.unwrap(),
            battery: v1_battery.unwrap(),
        };

        // Detect cube version
        let version = device.read(&characteristics.version)?;
        if version.len() < 3 {
            return Err(anyhow!("Device version invalid"));
        }
        let major = version[0];
        let minor = version[1];
        if major == 1 && minor <= 1 {
            Ok(Box::new(WCUCubeVersion1::new(
                device,
                characteristics,
                move_listener,
                minor,
            )?))
        } else if major == 2 && minor == 0 {
            Ok(Box::new(WCUSmartTimer::new(
                device,
                characteristics.last_moves,
                move_listener,
            )?))
        } else {
            Err(anyhow!(
                "WCU cube version {}.{} not supported",
                major,
                minor
            ))
        }
    } else if v2_read.is_some() && v2_write.is_some() {
        // println!("v2_read {:?}\nv2_write {:?}", v2_read, v2_write);
        Ok(Box::new(WCUCubeVersion2::new(
            device,
            v2_read.unwrap(),
            v2_write.unwrap(),
            move_listener,
        )?))
    } else {
        Err(anyhow!("Unrecognized WCU cube version"))
    }
}
