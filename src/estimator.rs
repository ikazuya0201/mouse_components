mod encoder;
mod imu;

use nb::block;
use quantities::{Angle, AngularSpeed, Distance, Frequency, Speed, Time};

use crate::agent::StateEstimator;
use crate::tracker::{State, SubState};
pub use encoder::Encoder;
pub use imu::IMU;

pub struct Estimator<LE, RE, I> {
    initial_x: Distance,
    initial_y: Distance,
    initial_theta: Angle,
    x: Distance,
    y: Distance,
    theta: Angle,
    period: Time,
    alpha: f32,
    wheel_interval: Distance,
    trans_speed: Speed,
    angular_speed: AngularSpeed,
    imu_angular_speed: AngularSpeed,
    filtered_imu_angular_speed: AngularSpeed,
    filtered_encoder_angular_speed: AngularSpeed,
    left_encoder: LE,
    right_encoder: RE,
    imu: I,
}

impl<LE, RE, I> StateEstimator<State> for Estimator<LE, RE, I>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
    <LE as Encoder>::Error: core::fmt::Debug,
    <RE as Encoder>::Error: core::fmt::Debug,
    <I as IMU>::Error: core::fmt::Debug,
{
    fn init(&mut self) {
        self.x = self.initial_x;
        self.y = self.initial_y;
        self.theta = self.initial_theta;
        self.trans_speed = Default::default();
        self.angular_speed = Default::default();
        self.imu_angular_speed = Default::default();
        self.filtered_imu_angular_speed = Default::default();
        self.filtered_encoder_angular_speed = Default::default();
    }

    fn estimate(&mut self) -> State {
        //velocity estimation
        let left_distance = block!(self.left_encoder.get_relative_distance()).unwrap();
        let right_distance = block!(self.right_encoder.get_relative_distance()).unwrap();

        let average_left_speed = left_distance / self.period;
        let average_right_speed = right_distance / self.period;

        let average_trans_speed = (average_left_speed + average_right_speed) / 2.0;

        let trans_acceleration = block!(self.imu.get_acceleration_y()).unwrap();
        let imu_angular_speed = block!(self.imu.get_angular_speed_z()).unwrap();

        //complementary filter
        let trans_speed = self.alpha * (self.trans_speed + trans_acceleration * self.period)
            + (1.0 - self.alpha) * average_trans_speed;
        //------

        let filtered_imu_angular_speed = self.alpha * self.filtered_imu_angular_speed
            + self.alpha * (imu_angular_speed - self.imu_angular_speed);

        let encoder_angular_speed =
            (right_distance - left_distance) / self.period / self.wheel_interval;

        let filtered_encoder_angular_speed = self.alpha * self.filtered_encoder_angular_speed
            + (1.0 - self.alpha) * encoder_angular_speed;

        let angular_speed = filtered_encoder_angular_speed + filtered_imu_angular_speed
            - self.filtered_imu_angular_speed;

        self.filtered_imu_angular_speed = filtered_imu_angular_speed;
        self.filtered_encoder_angular_speed = filtered_encoder_angular_speed;
        self.imu_angular_speed = imu_angular_speed;

        //pose estimation
        let trans_distance = trans_speed * self.period;
        let middle_theta =
            self.theta + (self.angular_speed + angular_speed / 2.0) * self.period / 2.0;
        self.x += trans_distance * middle_theta.cos();
        self.y += trans_distance * middle_theta.sin();

        self.theta += (self.angular_speed + angular_speed) * self.period / 2.0;
        //------

        let cos_th = self.theta.cos();
        let sin_th = self.theta.sin();
        let vx = trans_speed * cos_th;
        let vy = trans_speed * sin_th;
        let ax = trans_acceleration * cos_th;
        let ay = trans_acceleration * sin_th;

        let angular_acceleration = (angular_speed - self.angular_speed) / self.period;

        self.trans_speed = trans_speed;
        self.angular_speed = angular_speed;

        State {
            x: SubState {
                x: self.x,
                v: vx,
                a: ax,
            },
            y: SubState {
                x: self.y,
                v: vy,
                a: ay,
            },
            theta: SubState {
                x: self.theta,
                v: angular_speed,
                a: angular_acceleration,
            },
        }
    }
}

pub struct EstimatorBuilder<LE, RE, I, P, COF, POS, WI, X, Y> {
    left_encoder: LE,
    right_encoder: RE,
    imu: I,
    period: P,
    cut_off_frequency: COF,
    initial_posture: POS,
    initial_x: X,
    initial_y: Y,
    wheel_interval: WI,
}

impl EstimatorBuilder<(), (), (), (), (), (), (), (), ()> {
    pub fn new() -> Self {
        Self {
            left_encoder: (),
            right_encoder: (),
            imu: (),
            period: (),
            cut_off_frequency: (),
            initial_posture: (),
            initial_x: (),
            initial_y: (),
            wheel_interval: (),
        }
    }
}

impl<LE, RE, I> EstimatorBuilder<LE, RE, I, Time, Frequency, Angle, Distance, (), ()>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
{
    pub fn build(self) -> Estimator<LE, RE, I> {
        let alpha =
            1.0 / (2.0 * core::f32::consts::PI * self.period * self.cut_off_frequency + 1.0);
        Estimator {
            initial_x: Default::default(),
            initial_y: Default::default(),
            initial_theta: self.initial_posture,
            x: Default::default(),
            y: Default::default(),
            theta: self.initial_posture,
            period: self.period,
            wheel_interval: self.wheel_interval,
            alpha,
            filtered_encoder_angular_speed: Default::default(),
            filtered_imu_angular_speed: Default::default(),
            imu_angular_speed: Default::default(),
            trans_speed: Default::default(),
            angular_speed: Default::default(),
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
        }
    }
}

impl<LE, RE, I> EstimatorBuilder<LE, RE, I, Time, Frequency, Angle, Distance, Distance, Distance>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
{
    pub fn build(self) -> Estimator<LE, RE, I> {
        let alpha =
            1.0 / (2.0 * core::f32::consts::PI * self.period * self.cut_off_frequency + 1.0);
        Estimator {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            initial_theta: self.initial_posture,
            x: self.initial_x,
            y: self.initial_y,
            theta: self.initial_posture,
            period: self.period,
            wheel_interval: self.wheel_interval,
            alpha,
            filtered_encoder_angular_speed: Default::default(),
            filtered_imu_angular_speed: Default::default(),
            imu_angular_speed: Default::default(),
            trans_speed: Default::default(),
            angular_speed: Default::default(),
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
        }
    }
}

impl<LE, RE, I> EstimatorBuilder<LE, RE, I, Time, Frequency, (), Distance, (), ()>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
{
    pub fn build(self) -> Estimator<LE, RE, I> {
        let alpha =
            1.0 / (2.0 * core::f32::consts::PI * self.period * self.cut_off_frequency + 1.0);
        Estimator {
            initial_x: Default::default(),
            initial_y: Default::default(),
            initial_theta: Default::default(),
            x: Default::default(),
            y: Default::default(),
            theta: Default::default(),
            period: self.period,
            wheel_interval: self.wheel_interval,
            alpha,
            filtered_encoder_angular_speed: Default::default(),
            filtered_imu_angular_speed: Default::default(),
            imu_angular_speed: Default::default(),
            trans_speed: Default::default(),
            angular_speed: Default::default(),
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
        }
    }
}

impl<RE, I, P, COF, POS, WI, X, Y> EstimatorBuilder<(), RE, I, P, COF, POS, WI, X, Y> {
    pub fn left_encoder<LE>(
        self,
        left_encoder: LE,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, WI, X, Y>
    where
        LE: Encoder,
    {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, I, P, COF, POS, WI, X, Y> EstimatorBuilder<LE, (), I, P, COF, POS, WI, X, Y> {
    pub fn right_encoder<RE>(
        self,
        right_encoder: RE,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, WI, X, Y>
    where
        RE: Encoder,
    {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, P, COF, POS, WI, X, Y> EstimatorBuilder<LE, RE, (), P, COF, POS, WI, X, Y> {
    pub fn imu<I>(self, imu: I) -> EstimatorBuilder<LE, RE, I, P, COF, POS, WI, X, Y>
    where
        I: IMU,
    {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, I, COF, POS, WI, X, Y> EstimatorBuilder<LE, RE, I, (), COF, POS, WI, X, Y> {
    pub fn period(self, period: Time) -> EstimatorBuilder<LE, RE, I, Time, COF, POS, WI, X, Y> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, I, P, POS, WI, X, Y> EstimatorBuilder<LE, RE, I, P, (), POS, WI, X, Y> {
    pub fn cut_off_frequency(
        self,
        cut_off_frequency: Frequency,
    ) -> EstimatorBuilder<LE, RE, I, P, Frequency, POS, WI, X, Y> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, I, P, COF, WI, X, Y> EstimatorBuilder<LE, RE, I, P, COF, (), WI, X, Y> {
    pub fn initial_posture(
        self,
        initial_posture: Angle,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, Angle, WI, X, Y> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, I, P, COF, POS, X, Y> EstimatorBuilder<LE, RE, I, P, COF, POS, (), X, Y> {
    pub fn wheel_interval(
        self,
        wheel_interval: Distance,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, Distance, X, Y> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval,
        }
    }
}
impl<LE, RE, I, P, COF, POS, WI, Y> EstimatorBuilder<LE, RE, I, P, COF, POS, WI, (), Y> {
    pub fn initial_x(
        self,
        initial_x: Distance,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, WI, Distance, Y> {
        EstimatorBuilder {
            initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}
impl<LE, RE, I, P, COF, POS, WI, X> EstimatorBuilder<LE, RE, I, P, COF, POS, WI, X, ()> {
    pub fn initial_y(
        self,
        initial_y: Distance,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, WI, X, Distance> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}
