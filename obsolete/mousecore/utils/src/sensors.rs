use alloc::rc::Rc;
use alloc::vec::Vec;
use core::cell::RefCell;
use core::convert::Infallible;

#[cfg(not(feature = "std"))]
use micromath::F32Ext;
use mousecore::{
    prelude::*,
    sensors::{
        DistanceSensor as IDistanceSensor, Encoder as IEncoder, Imu as IImu, Motor as IMotor,
    },
    types::data::{Pose, RobotState},
    utils::random::Random,
    wall_detector::{ConversionError, PoseConverter, PoseConverterBuilder},
    wall_manager::WallManager,
};
use uom::si::f32::{
    Acceleration, Angle, AngularAcceleration, AngularVelocity, ElectricPotential, Length, Time,
    Velocity,
};
use uom::si::{
    angular_velocity::radian_per_second, electric_potential::volt, length::meter, ratio::ratio,
    velocity::meter_per_second,
};

pub struct AgentSimulator<const N: usize> {
    inner: Rc<RefCell<AgentSimulatorInner>>,
    wall_storage: Rc<WallManager<N>>,
    distance_sensors_poses: Vec<Pose>,
}

impl<const N: usize> AgentSimulator<N> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        state: RobotState,
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
        max_voltage: ElectricPotential,
        square_width: Length,
        wall_width: Length,
        ignore_radius_from_pillar: Length,
        ignore_length_from_wall: Length,
    ) -> (
        Stepper,
        Observer,
        Encoder,
        Encoder,
        Imu,
        Motor,
        Motor,
        Vec<DistanceSensor<N>>,
    ) {
        let distance_sensors = self
            .distance_sensors_poses
            .iter()
            .map(|pose| DistanceSensor {
                inner: Rc::clone(&self.inner),
                pose: *pose,
                wall_storage: Rc::clone(&self.wall_storage),
                pose_converter: PoseConverterBuilder::new()
                    .square_width(square_width)
                    .wall_width(wall_width)
                    .ignore_radius_from_pillar(ignore_radius_from_pillar)
                    .ignore_length_from_wall(ignore_length_from_wall)
                    .build()
                    .unwrap(),
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
            Imu {
                inner: Rc::clone(&self.inner),
            },
            Motor {
                inner: Rc::clone(&self.inner),
                direction: Direction::Right,
                max_voltage,
            },
            Motor {
                inner: Rc::clone(&self.inner),
                direction: Direction::Left,
                max_voltage,
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
    pub fn state(&self) -> RobotState {
        self.inner.borrow().current.clone()
    }

    pub fn voltage(&self) -> (ElectricPotential, ElectricPotential) {
        let inner = self.inner.borrow();
        (inner.right_voltage, inner.left_voltage)
    }
}

struct AgentSimulatorInner {
    current: RobotState,
    prev: RobotState,
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

        let cur_sin = self.current.theta.x.value.sin();
        let cur_cos = self.current.theta.x.value.cos();
        let next_sin = next.theta.x.value.sin();
        let next_cos = next.theta.x.value.cos();

        next.x.x.mean +=
            (current_trans_vel * cur_cos + next_trans_vel * next_cos) * self.period / 2.0;
        next.y.x.mean +=
            (current_trans_vel * cur_sin + next_trans_vel * next_sin) * self.period / 2.0;

        next.x.v = next_trans_vel * next_cos;
        next.y.v = next_trans_vel * next_sin;

        next.x.a = (next.x.v - self.current.x.v) / self.period;
        next.y.a = (next.y.v - self.current.y.v) / self.period;

        self.prev = self.current.clone();
        self.current = next;
    }

    fn trans_vel(state: &RobotState) -> Velocity {
        state.x.v * state.theta.x.value.cos() + state.y.v * state.theta.x.value.sin()
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
    fn translational_velocity(state: &RobotState) -> Velocity {
        state.x.v * state.theta.x.value.cos() + state.y.v * state.theta.x.value.sin()
    }

    fn rotational_velocity(state: &RobotState) -> AngularVelocity {
        state.theta.v
    }

    fn average<T, F>(&self, f: &F) -> T
    where
        F: Fn(&RobotState) -> T,
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
            Direction::Right => {
                Ok((average_trans + average_rot * self.wheel_interval / 2.0) * period)
            }
            Direction::Left => {
                Ok((average_trans - average_rot * self.wheel_interval / 2.0) * period)
            }
        }
    }
}

pub struct Imu {
    inner: Rc<RefCell<AgentSimulatorInner>>,
}

impl IImu for Imu {
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
    max_voltage: ElectricPotential,
}

impl IMotor for Motor {
    fn apply(&mut self, electric_potential: ElectricPotential) {
        let electric_potential = if electric_potential > self.max_voltage {
            self.max_voltage
        } else if electric_potential < -self.max_voltage {
            -self.max_voltage
        } else {
            electric_potential
        };
        match self.direction {
            Direction::Right => self.inner.borrow_mut().right_voltage = electric_potential,
            Direction::Left => self.inner.borrow_mut().left_voltage = electric_potential,
        }
    }
}

pub struct DistanceSensor<const N: usize> {
    inner: Rc<RefCell<AgentSimulatorInner>>,
    pose: Pose,
    wall_storage: Rc<WallManager<N>>,
    pose_converter: PoseConverter<N>,
}

#[derive(Clone, PartialEq, Debug)]
pub enum DistanceSensorError {
    PoseConverter(ConversionError),
    DistanceIsNaN,
}

impl<const N: usize> IDistanceSensor for DistanceSensor<N> {
    type Error = DistanceSensorError;

    fn pose(&self) -> &Pose {
        &self.pose
    }

    fn get_distance(&mut self) -> nb::Result<Random<Length>, Self::Error> {
        let state = &self.inner.borrow().current;
        let sin_th = state.theta.x.value.sin();
        let cos_th = state.theta.x.value.cos();
        let pose = Pose {
            x: state.x.x.mean + self.pose.x * cos_th - self.pose.y * sin_th,
            y: state.y.x.mean + self.pose.x * sin_th + self.pose.y * cos_th,
            theta: state.theta.x + self.pose.theta,
        };
        let wall_info = self
            .pose_converter
            .convert(&pose)
            .map_err(DistanceSensorError::PoseConverter)?;
        let distance = if self.wall_storage.exists(&wall_info.wall) {
            wall_info.existing_distance
        } else {
            wall_info.not_existing_distance
        };
        if distance.is_nan() {
            return Err(nb::Error::Other(DistanceSensorError::DistanceIsNaN));
        }
        Ok(Random {
            mean: distance,
            standard_deviation: Length::new::<meter>(0.002),
        })
    }
}
