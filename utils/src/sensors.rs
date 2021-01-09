use alloc::rc::Rc;
use alloc::vec::Vec;
use core::cell::RefCell;
use core::convert::Infallible;
use core::ops::Mul;

use components::{
    data_types::{Pose, State},
    errors::ConversionError,
    impls::{PoseConverter, WallManager},
    prelude::*,
    sensors::{
        DistanceSensor as IDistanceSensor, Encoder as IEncoder, Motor as IMotor, IMU as IIMU,
    },
    utils::{mutex::Mutex, probability::Probability, sample::Sample},
};
use generic_array::ArrayLength;
use typenum::{consts::*, PowerOfTwo, Unsigned};
use uom::si::f32::{
    Acceleration, Angle, AngularAcceleration, AngularVelocity, ElectricPotential, Length, Time,
    Velocity,
};
use uom::si::{
    angular_velocity::radian_per_second, electric_potential::volt, length::meter, ratio::ratio,
    velocity::meter_per_second,
};

use crate::math::MathFake;

pub struct AgentSimulator<N>
where
    N: Mul<N>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    inner: Rc<RefCell<AgentSimulatorInner>>,
    wall_storage: Rc<WallManager<N>>,
    distance_sensors_poses: Vec<Pose>,
}

impl<N> AgentSimulator<N>
where
    N: Mul<N>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    pub fn new(
        state: State,
        period: Time,
        trans_model_gain: f32,
        trans_model_time_constant: Time,
        rot_model_gain: f32,
        rot_model_time_constant: Time,
        wall_storage: WallManager<N>,
        distance_sensors_poses: Vec<Pose>,
    ) -> Self {
        Self {
            inner: Rc::new(RefCell::new(AgentSimulatorInner {
                current: state.clone(),
                prev: state,
                right_voltage: Default::default(),
                left_voltage: Default::default(),
                period,
                trans_model_gain,
                trans_model_time_constant,
                rot_model_gain,
                rot_model_time_constant,
            })),
            wall_storage: Rc::new(wall_storage),
            distance_sensors_poses,
        }
    }

    pub fn split(
        self,
        wheel_interval: Length,
    ) -> (
        Stepper,
        Observer,
        Encoder,
        Encoder,
        IMU,
        Motor,
        Motor,
        Vec<DistanceSensor<N>>,
    ) {
        let distance_sensors = self
            .distance_sensors_poses
            .iter()
            .map(|pose| DistanceSensor {
                inner: Rc::clone(&self.inner),
                pose: pose.clone(),
                wall_storage: Rc::clone(&self.wall_storage),
                pose_converter: Default::default(),
            })
            .collect();
        (
            Stepper {
                inner: Rc::clone(&self.inner),
            },
            Observer {
                inner: Rc::clone(&self.inner),
            },
            Encoder {
                inner: Rc::clone(&self.inner),
                wheel_interval,
                direction: Direction::Right,
            },
            Encoder {
                inner: Rc::clone(&self.inner),
                wheel_interval,
                direction: Direction::Left,
            },
            IMU {
                inner: Rc::clone(&self.inner),
            },
            Motor {
                inner: Rc::clone(&self.inner),
                direction: Direction::Right,
            },
            Motor {
                inner: Rc::clone(&self.inner),
                direction: Direction::Left,
            },
            distance_sensors,
        )
    }
}

pub struct Stepper {
    inner: Rc<RefCell<AgentSimulatorInner>>,
}

impl Stepper {
    pub fn step(&self) {
        self.inner.borrow_mut().step()
    }
}

pub struct Observer {
    inner: Rc<RefCell<AgentSimulatorInner>>,
}

impl Observer {
    pub fn state(&self) -> State {
        self.inner.borrow().current.clone()
    }

    pub fn voltage(&self) -> (ElectricPotential, ElectricPotential) {
        let inner = self.inner.borrow();
        (inner.right_voltage, inner.left_voltage)
    }
}

struct AgentSimulatorInner {
    current: State,
    prev: State,
    right_voltage: ElectricPotential,
    left_voltage: ElectricPotential,
    period: Time,
    trans_model_gain: f32,
    trans_model_time_constant: Time,
    rot_model_gain: f32,
    rot_model_time_constant: Time,
}

impl AgentSimulatorInner {
    fn step(&mut self) {
        let mut next = self.current.clone();

        let current_trans_vel = Self::trans_vel(&self.current);
        let next_trans_vel = self.next_trans_vel();

        let next_rot_vel = self.next_rot_vel();

        next.theta.x += Angle::from(next_rot_vel * self.period);
        next.theta.v = next_rot_vel;
        next.theta.a =
            AngularAcceleration::from((next_rot_vel - self.current.theta.v) / self.period);

        let (cur_sin, cur_cos) = MathFake::sincos(self.current.theta.x);
        let (next_sin, next_cos) = MathFake::sincos(next.theta.x);

        next.x.x += (current_trans_vel * cur_cos + next_trans_vel * next_cos) * self.period / 2.0;
        next.y.x += (current_trans_vel * cur_sin + next_trans_vel * next_sin) * self.period / 2.0;

        next.x.v = next_trans_vel * next_cos;
        next.y.v = next_trans_vel * next_sin;

        next.x.a = (next.x.v - self.current.x.v) / self.period;
        next.y.a = (next.y.v - self.current.y.v) / self.period;

        self.prev = self.current.clone();
        self.current = next;
    }

    fn trans_vel(state: &State) -> Velocity {
        state.x.v * MathFake::cos(state.theta.x) + state.y.v * MathFake::sin(state.theta.x)
    }

    fn next_trans_vel(&self) -> Velocity {
        let alpha = (self.trans_model_time_constant
            / (self.trans_model_time_constant + self.period))
            .get::<ratio>();
        let current = Self::trans_vel(&self.current);
        let trans_voltage = (self.right_voltage + self.left_voltage) / 2.0;
        current * alpha
            + (1.0 - alpha)
                * Velocity::new::<meter_per_second>(
                    self.trans_model_gain * trans_voltage.get::<volt>(),
                )
    }

    fn next_rot_vel(&self) -> AngularVelocity {
        let alpha = (self.rot_model_time_constant / (self.rot_model_time_constant + self.period))
            .get::<ratio>();
        let current = self.current.theta.v;
        let rot_voltage = (self.right_voltage - self.left_voltage) / 2.0;
        current * alpha
            + (1.0 - alpha)
                * AngularVelocity::new::<radian_per_second>(
                    self.rot_model_gain * rot_voltage.get::<volt>(),
                )
    }
}

#[derive(Clone, Copy, Debug)]
struct FinishError;

#[derive(Clone, Copy, Debug)]
enum Direction {
    Right,
    Left,
}

pub struct Encoder {
    inner: Rc<RefCell<AgentSimulatorInner>>,
    wheel_interval: Length,
    direction: Direction,
}

impl Encoder {
    fn translational_velocity(state: &State) -> Velocity {
        state.x.v * state.theta.x.value.cos() + state.y.v * state.theta.x.value.sin()
    }

    fn rotational_velocity(state: &State) -> AngularVelocity {
        state.theta.v
    }

    fn average<T, F>(&self, f: &F) -> T
    where
        F: Fn(&State) -> T,
        T: core::ops::Add<T, Output = T> + core::ops::Div<f32, Output = T>,
    {
        let current = f(&self.inner.borrow().current);
        let prev = f(&self.inner.borrow().prev);
        (current + prev) / 2.0
    }
}

impl IEncoder for Encoder {
    type Error = Infallible;

    fn get_relative_distance(&mut self) -> nb::Result<Length, Self::Error> {
        let average_trans = self.average(&Self::translational_velocity);
        let average_rot = self.average(&Self::rotational_velocity);
        let period = self.inner.borrow().period;
        match self.direction {
            Direction::Right => Ok((average_trans
                + Velocity::from(average_rot * self.wheel_interval / 2.0))
                * period),
            Direction::Left => Ok((average_trans
                - Velocity::from(average_rot * self.wheel_interval / 2.0))
                * period),
        }
    }
}

pub struct IMU {
    inner: Rc<RefCell<AgentSimulatorInner>>,
}

impl IIMU for IMU {
    type Error = Infallible;

    fn get_angular_velocity(&mut self) -> nb::Result<AngularVelocity, Self::Error> {
        Ok(self.inner.borrow().current.theta.v)
    }

    fn get_translational_acceleration(&mut self) -> nb::Result<Acceleration, Self::Error> {
        let state = &self.inner.borrow().current;
        let theta = state.theta.x.value;
        Ok(state.x.a * theta.cos() + state.y.a * theta.sin())
    }
}

pub struct Motor {
    inner: Rc<RefCell<AgentSimulatorInner>>,
    direction: Direction,
}

impl IMotor for Motor {
    fn apply(&mut self, electric_potential: ElectricPotential) {
        match self.direction {
            Direction::Right => self.inner.borrow_mut().right_voltage = electric_potential,
            Direction::Left => self.inner.borrow_mut().left_voltage = electric_potential,
        }
    }
}

pub struct DistanceSensor<N>
where
    N: Mul<N>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    inner: Rc<RefCell<AgentSimulatorInner>>,
    pose: Pose,
    wall_storage: Rc<WallManager<N>>,
    pose_converter: PoseConverter<N, MathFake>,
}

impl<N> IDistanceSensor for DistanceSensor<N>
where
    N: Mul<N> + PowerOfTwo + Unsigned,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    type Error = ConversionError;

    fn pose(&self) -> Pose {
        self.pose
    }

    fn get_distance(&mut self) -> nb::Result<Sample<Length>, Self::Error> {
        let state = &self.inner.borrow().current;
        let pose = Pose {
            x: state.x.x + self.pose.x,
            y: state.y.x + self.pose.y,
            theta: state.theta.x + self.pose.theta,
        };
        let wall_info = self.pose_converter.convert(&pose)?;
        let distance = if self.wall_storage.exists(&wall_info.wall) {
            wall_info.existing_distance
        } else {
            wall_info.not_existing_distance
        };
        Ok(Sample {
            mean: distance,
            standard_deviation: Length::new::<meter>(0.001),
        })
    }
}
