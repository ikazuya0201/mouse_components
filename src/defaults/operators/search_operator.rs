use core::ops::Mul;

use generic_array::ArrayLength;
use spin::Mutex;
use typenum::consts::*;

use super::{SearchAgent, SearchCommander};
use crate::utils::probability::Probability;

pub struct SearchOperator<
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
    crate::operators::TrackingOperator<
        SearchCommander<'a, Size>,
        SearchAgent<
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
    Size: Mul<Size>,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
    Math: crate::utils::math::Math;
