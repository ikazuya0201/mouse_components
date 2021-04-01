use core::ops::Mul;

use generic_array::ArrayLength;
use spin::Mutex;
use typenum::consts::*;

use super::{ReturnSetupAgent, ReturnSetupCommander};
use crate::operators::TrackingOperator;
use crate::utils::probability::Probability;

pub struct ReturnSetupOperator<
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
        ReturnSetupCommander<'a, Size>,
        ReturnSetupAgent<
            'a,
            LeftEncoder,
            RightEncoder,
            Imu,
            LeftMotor,
            RightMotor,
            DistanceSensor,
            Math,
            Size,
        >,
    >,
)
where
    Size: Mul<Size>,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
    Math: crate::utils::math::Math;
