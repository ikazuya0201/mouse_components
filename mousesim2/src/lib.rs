#![no_std]

#[allow(unused_imports)]
use micromath::F32Ext;
use mousecore2::{
    control::MotorOutput,
    estimate::State,
    solve::WallState,
    wall::{Pose, PoseConverter, Walls},
};
use typed_builder::TypedBuilder;
use uom::si::{
    angular_velocity::radian_per_second,
    electric_potential::volt,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularVelocity, ElectricPotential, Length, Time,
        Velocity,
    },
    length::meter,
    ratio::ratio,
    velocity::meter_per_second,
};

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct EncoderOutput {
    pub left: Length,
    pub right: Length,
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Normal<T> {
    pub mean: T,
    pub stddev: T,
}

#[derive(TypedBuilder)]
pub struct Simulator<const W: u8> {
    #[builder(setter(transform = |s: &str| s.parse().unwrap()))]
    walls: Walls<W>,
    current: State,
    last: State,
    #[builder(default, setter(skip))]
    right_voltage: ElectricPotential,
    #[builder(default, setter(skip))]
    left_voltage: ElectricPotential,
    max_voltage: ElectricPotential,
    period: Time,
    trans_k: f32,
    trans_t1: Time,
    rot_k: f32,
    rot_t1: Time,
    wheel_interval: Length,
    #[builder(default, setter(skip))]
    pose_converter: PoseConverter<W>,
}

impl<const W: u8> Simulator<W> {
    pub fn walls(&self) -> &Walls<W> {
        &self.walls
    }

    pub fn step(&mut self) {
        let mut next = self.current.clone();

        let current_trans_vel = Self::trans_vel(&self.current);
        let next_trans_vel = self.next_trans_vel();

        let next_rot_vel = self.next_rot_vel();

        next.theta.x += Angle::from(next_rot_vel * self.period);
        next.theta.v = next_rot_vel;
        next.theta.a =
            AngularAcceleration::from((next_rot_vel - self.current.theta.v) / self.period);

        let cur_sin = self.current.theta.x.value.sin();
        let cur_cos = self.current.theta.x.value.cos();
        let next_sin = next.theta.x.value.sin();
        let next_cos = next.theta.x.value.cos();

        next.x.x += (current_trans_vel * cur_cos + next_trans_vel * next_cos) * self.period / 2.0;
        next.y.x += (current_trans_vel * cur_sin + next_trans_vel * next_sin) * self.period / 2.0;

        next.x.v = next_trans_vel * next_cos;
        next.y.v = next_trans_vel * next_sin;

        next.x.a = (next.x.v - self.current.x.v) / self.period;
        next.y.a = (next.y.v - self.current.y.v) / self.period;

        self.last = self.current.clone();
        self.current = next;
    }

    fn trans_vel(state: &State) -> Velocity {
        state.x.v * state.theta.x.value.cos() + state.y.v * state.theta.x.value.sin()
    }

    fn next_trans_vel(&self) -> Velocity {
        let alpha = (self.trans_t1 / (self.trans_t1 + self.period)).get::<ratio>();
        let current = Self::trans_vel(&self.current);
        let trans_voltage = (self.right_voltage + self.left_voltage) / 2.0;
        current * alpha
            + (1.0 - alpha)
                * Velocity::new::<meter_per_second>(self.trans_k * trans_voltage.get::<volt>())
    }

    fn next_rot_vel(&self) -> AngularVelocity {
        let alpha = (self.rot_t1 / (self.rot_t1 + self.period)).get::<ratio>();
        let current = self.current.theta.v;
        let rot_voltage = (self.right_voltage - self.left_voltage) / 2.0;
        current * alpha
            + (1.0 - alpha)
                * AngularVelocity::new::<radian_per_second>(self.rot_k * rot_voltage.get::<volt>())
    }

    pub fn distance(&self) -> EncoderOutput {
        let average_trans = self.average(&|state| {
            state.x.v * state.theta.x.value.cos() + state.y.v * state.theta.x.value.sin()
        });
        let average_rot = self.average(&|state| state.theta.v);
        EncoderOutput {
            left: (average_trans - average_rot * self.wheel_interval / 2.0) * self.period,
            right: (average_trans + average_rot * self.wheel_interval / 2.0) * self.period,
        }
    }

    fn average<T, F>(&self, f: &F) -> T
    where
        F: Fn(&State) -> T,
        T: core::ops::Add<T, Output = T> + core::ops::Div<f32, Output = T>,
    {
        let current = f(&self.current);
        let prev = f(&self.last);
        (current + prev) / 2.0
    }

    pub fn angular_velocity(&self) -> AngularVelocity {
        self.current.theta.v
    }

    pub fn translational_acceleration(&self) -> Acceleration {
        let theta = self.current.theta.x;
        self.current.x.a * theta.value.cos() + self.current.y.a * theta.value.sin()
    }

    pub fn apply(&mut self, voltage: &MotorOutput) {
        let limit = |val| {
            if val > self.max_voltage {
                self.max_voltage
            } else if val < -self.max_voltage {
                -self.max_voltage
            } else {
                val
            }
        };
        let left = limit(voltage.left);
        let right = limit(voltage.right);
        self.left_voltage = left;
        self.right_voltage = right;
    }

    pub fn distance_from_wall(&self, pose: &Pose) -> Option<Normal<Length>> {
        let wall_info = self.pose_converter.convert(pose)?;
        let distance = if matches!(
            self.walls.wall_state(&wall_info.coord),
            WallState::Checked { exists: true }
        ) {
            wall_info.existing_distance
        } else {
            wall_info.not_existing_distance
        };
        if distance.is_nan() {
            return None;
        }
        Some(Normal {
            mean: distance,
            stddev: Length::new::<meter>(0.002),
        })
    }
}
