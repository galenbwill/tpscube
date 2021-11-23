use byteorder::{BigEndian, ByteOrder, LittleEndian};
use std::fmt;
use std::ops::{Mul, Sub};
use std::time::{Duration, Instant};

#[derive(PartialEq, PartialOrd, Debug, Copy, Clone)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }
    pub fn from_array(q: &[f32; 4]) -> Self {
        Self {
            w: q[0],
            x: q[1],
            y: q[2],
            z: q[3],
        }
    }
    pub fn as_array(&self) -> [f32; 4] {
        [self.w, self.x, self.y, self.z]
    }
    pub fn from_bytes(bytes: &[u8]) -> Quat {
        let mut r: [f32; 4] = [0.0, 0.0, 0.0, 0.0];
        for i in 0..4 {
            r[i] = LittleEndian::read_f32(&bytes[i * 4..i * 4 + 4]);
        }
        // Quat::new(r[0], r[1], -r[3], r[2]) // original -- y axis reversed
        Quat::new(r[0], -r[1], -r[3], r[2]) // y axis tracks
    }

    pub fn quat_invert(&self) -> Quat {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let f = 1.0 / (w * w + x * x + y * y + z * z);
        return Quat::new(w * f, -x * f, -y * f, -z * f);
    }

    pub fn quat_normalize(&self) -> Quat {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let f = (1.0 / (w * w + x * x + y * y + z * z)).sqrt();
        return Quat::new(w * f, x * f, y * f, z * f);
    }

    pub fn quat_matrix(&self) -> [Quat; 4] {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        return [
            Quat::new(
                w * w + x * x - y * y - z * z,
                2.0 * x * y - 2.0 * w * z,
                2.0 * x * z + 2.0 * w * y,
                0.0,
            ),
            Quat::new(
                2.0 * x * y + 2.0 * w * z,
                w * w - x * x + y * y - z * z,
                2.0 * y * z - 2.0 * w * x,
                0.0,
            ),
            Quat::new(
                2.0 * x * z - 2.0 * w * y,
                2.0 * y * z + 2.0 * w * x,
                w * w - x * x - y * y + z * z,
                0.0,
            ),
            Quat::new(0.0, 0.0, 0.0, 1.0),
        ];
    }
}

impl Sub for Quat {
    type Output = Quat;
    fn sub(self, rhs: Quat) -> Quat {
        Quat {
            w: self.w - rhs.w,
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Mul for Quat {
    type Output = Quat;
    fn mul(self, rhs: Quat) -> Quat {
        let (a, b, c, d) = (self.w, self.x, self.y, self.z);
        let (w, x, y, z) = (rhs.w, rhs.x, rhs.y, rhs.z);
        Quat {
            w: a * w - b * x - c * y - d * z,
            x: a * x + b * w + c * d - d * y,
            y: a * y - b * z + c * w + d * x,
            z: a * z + b * y - c * x + d * w,
        }
    }
}

impl fmt::Display for Quat {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        const F: f32 = 100000.0;
        let (w, x, y, z) = (self.w * F, self.x * F, self.y * F, self.z * F);
        write!(
            f,
            "[w:{:+10.2}  x:{:+10.2}  y:{:+10.2}  z:{:+10.2}]",
            w,
            x,
            y,
            z,
            // self.w * f, self.x * f, self.y * f, self.z * f,
        )
    }
}

#[derive(PartialEq, PartialOrd, Debug, Copy, Clone)]
struct TimeStamp {
    i: Option<Instant>,
    d: Option<Duration>,
}

impl TimeStamp {
    fn from_instant(i: Instant) -> TimeStamp {
        TimeStamp {
            i: Some(i),
            d: None,
        }
    }
}

impl Sub for TimeStamp {
    type Output = Result<TimeStamp, &'static str>;
    fn sub(self, rhs: TimeStamp) -> Result<TimeStamp, &'static str> {
        match self.i {
            Some(self_i) => match rhs.i {
                None => Err("rhs ts not an instant"),
                Some(rhs_i) => Ok(TimeStamp {
                    i: None,
                    d: Some(self_i - rhs_i),
                }),
            },
            None => match self.d {
                Some(self_d) => match rhs.d {
                    None => Err("rhs ts not a duration"),
                    Some(rhs_d) => Ok(TimeStamp {
                        i: None,
                        d: Some(self_d - rhs_d),
                    }),
                },
                None => Err("self ts is None"),
            },
        }
    }
}

#[derive(PartialEq, PartialOrd, Debug, Copy, Clone)]
pub struct QGyroState {
    pub t: u32,
    pub q: Quat,
    d: bool,
    ts: TimeStamp,
}

impl QGyroState {
    pub fn new(t: u32, q: Quat) -> Self {
        Self {
            t,
            q,
            d: false,
            ts: TimeStamp::from_instant(Instant::now()),
        }
    }
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let t = BigEndian::read_u32(&bytes[0..4]);
        let q = Quat::from_bytes(&bytes[4..20]);
        Self::new(t, q)
    }
}

impl Sub for QGyroState {
    type Output = QGyroState;
    fn sub(self, rhs: QGyroState) -> QGyroState {
        QGyroState {
            // t: self.t.wrapping_sub(rhs.t),
            t: if self.t > rhs.t {
                self.t - rhs.t
            } else {
                rhs.t - self.t
            },
            q: self.q - rhs.q,
            d: true,
            ts: (self.ts - rhs.ts).unwrap(),
        }
    }
}

impl fmt::Display for QGyroState {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "GyroState {} {}{:#?} ({:#?})",
            self.q,
            if self.d { "+" } else { " " },
            Duration::from_micros(self.t as u64 / 10),
            if self.d {
                if let Some(d) = self.ts.d {
                    d
                } else if let Some(i) = self.ts.i {
                    i.elapsed()
                } else {
                    Duration::from_secs(0)
                }
            } else if let Some(i) = self.ts.i {
                i.elapsed()
            } else {
                Duration::from_secs(0)
            }
        )
    }
}
