use core::ops::Mul;

use generic_array::ArrayLength;
use spin::Mutex;
use typenum::consts::*;

use super::{RunAgent, RunCommander};
use crate::utils::probability::Probability;
use crate::{
    operators::TrackingOperator,
    trajectory_generators::{RunTrajectory, ShiftTrajectory},
};

pub struct ReturnOperator<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
>(
    TrackingOperator<
        RunCommander<'a, Size>,
        RunAgent<
            'a,
            LeftEncoder,
            RightEncoder,
            Imu,
            LeftMotor,
            RightMotor,
            DistanceSensor,
            Size,
            Math,
        >,
    >,
)
where
    Size: Mul<Size> + Clone,
    Size::Output: Mul<U2> + ArrayLength<ShiftTrajectory<RunTrajectory<Math>, Math>>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
    Math: crate::utils::math::Math;
