//! Implementations of [Controller](crate::tracker::Controller) and their builders.

use core::ops::{Add, Div, Mul, Sub};

use serde::{Deserialize, Serialize};
use uom::si::{
    f32::{ElectricPotential, Length, Ratio, Time},
    Quantity,
};

use crate::tracker::{ControlTarget, Controller};
use crate::{Construct, Deconstruct};

pub trait Motor {
    fn apply(&mut self, electric_potential: ElectricPotential);
}

struct SisoController<T>
where
    T: Div<Time>,
    <T as Div<Time>>::Output: Div<ElectricPotential> + Div<Time>,
    ElectricPotential: Div<T>
        + Div<<T as Div<Time>>::Output>
        + Div<<<T as Div<Time>>::Output as Div<Time>>::Output>,
{
    error_sum: T,
    model_gain: <<T as Div<Time>>::Output as Div<ElectricPotential>>::Output,
    time_constant: Time,
    kp: <ElectricPotential as Div<<T as Div<Time>>::Output>>::Output,
    ki: <ElectricPotential as Div<T>>::Output,
    kd: <ElectricPotential as Div<<<T as Div<Time>>::Output as Div<Time>>::Output>>::Output,
    period: Time,
}

impl<T> SisoController<T>
where
    T: Div<Time> + Copy + Add<T, Output = T>,
    <T as Div<Time>>::Output: Div<Time>
        + Copy
        + Add<<T as Div<Time>>::Output, Output = <T as Div<Time>>::Output>
        + Sub<<T as Div<Time>>::Output, Output = <T as Div<Time>>::Output>
        + Div<ElectricPotential>
        + Div<
            <<T as Div<Time>>::Output as Div<ElectricPotential>>::Output,
            Output = ElectricPotential,
        > + Mul<Time, Output = T>,
    <<T as Div<Time>>::Output as Div<Time>>::Output: Mul<Time, Output = <T as Div<Time>>::Output>
        + Sub<
            <<T as Div<Time>>::Output as Div<Time>>::Output,
            Output = <<T as Div<Time>>::Output as Div<Time>>::Output,
        > + Copy,
    ElectricPotential: Div<T>
        + Div<<T as Div<Time>>::Output>
        + Div<<<T as Div<Time>>::Output as Div<Time>>::Output>,
    <ElectricPotential as Div<T>>::Output: Copy + Mul<T, Output = ElectricPotential>,
    <ElectricPotential as Div<<T as Div<Time>>::Output>>::Output:
        Copy + Mul<<T as Div<Time>>::Output, Output = ElectricPotential>,
    <<T as Div<Time>>::Output as Div<ElectricPotential>>::Output: Copy,
    <ElectricPotential as Div<<<T as Div<Time>>::Output as Div<Time>>::Output>>::Output:
        Copy + Mul<<<T as Div<Time>>::Output as Div<Time>>::Output, Output = ElectricPotential>,
{
    fn calculate(
        &mut self,
        r: <T as Div<Time>>::Output,
        dr: <<T as Div<Time>>::Output as Div<Time>>::Output,
        y: <T as Div<Time>>::Output,
        dy: <<T as Div<Time>>::Output as Div<Time>>::Output,
    ) -> ElectricPotential {
        let vol_f = (dr * self.time_constant + r) / self.model_gain;
        let vol_p = self.kp * (r - y);
        let vol_i = self.ki * self.error_sum;
        let vol_d = self.kd * (dr - dy);
        self.error_sum = self.error_sum + (r - y) * self.period;
        vol_f + vol_p + vol_i + vol_d
    }
}

/// Parameter for control SISO system.
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct ControlParameters {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub model_k: f32,
    pub model_t1: f32,
}

// TODO: Write test.
/// An implementation of [Controller](crate::tracker::Controller).
/// This controls the plant as 2 SISO system.
pub struct MultiSisoController<LeftMotor, RightMotor> {
    left_motor: LeftMotor,
    right_motor: RightMotor,
    translational_controller: SisoController<Length>,
    rotational_controller: SisoController<Ratio>,
}

impl<LeftMotor, RightMotor> MultiSisoController<LeftMotor, RightMotor> {
    pub fn new(
        left_motor: LeftMotor,
        right_motor: RightMotor,
        trans_param: ControlParameters,
        rot_param: ControlParameters,
        period: Time,
    ) -> Self {
        Self {
            left_motor,
            right_motor,
            translational_controller: SisoController {
                error_sum: Default::default(),
                model_gain: Quantity {
                    value: trans_param.model_k,
                    ..Default::default()
                },
                time_constant: Quantity {
                    value: trans_param.model_t1,
                    ..Default::default()
                },
                kp: Quantity {
                    value: trans_param.kp,
                    ..Default::default()
                },
                ki: Quantity {
                    value: trans_param.ki,
                    ..Default::default()
                },
                kd: Quantity {
                    value: trans_param.kd,
                    ..Default::default()
                },
                period,
            },
            rotational_controller: SisoController {
                error_sum: Default::default(),
                model_gain: Quantity {
                    value: rot_param.model_k,
                    ..Default::default()
                },
                time_constant: Quantity {
                    value: rot_param.model_t1,
                    ..Default::default()
                },
                kp: Quantity {
                    value: rot_param.kp,
                    ..Default::default()
                },
                ki: Quantity {
                    value: rot_param.ki,
                    ..Default::default()
                },
                kd: Quantity {
                    value: rot_param.kd,
                    ..Default::default()
                },
                period,
            },
        }
    }

    pub fn release(self) -> (LeftMotor, RightMotor) {
        let Self {
            left_motor,
            right_motor,
            ..
        } = self;
        (left_motor, right_motor)
    }
}

impl<LeftMotor, RightMotor> Controller for MultiSisoController<LeftMotor, RightMotor>
where
    LeftMotor: Motor,
    RightMotor: Motor,
{
    fn control(&mut self, r: &ControlTarget, y: &ControlTarget) {
        let vol_t = self.translational_controller.calculate(r.v, r.a, y.v, y.a);
        let vol_r = self.rotational_controller.calculate(
            r.omega.into(),
            r.alpha.into(),
            y.omega.into(),
            y.alpha.into(),
        );
        self.left_motor.apply(vol_t - vol_r);
        self.right_motor.apply(vol_t + vol_r);
    }
}

/// Config for [MultiSisoController].
#[derive(Clone, PartialEq, Debug)]
pub struct MultiSisoControllerConfig {
    pub translational_parameters: ControlParameters,
    pub rotational_parameters: ControlParameters,
    pub period: Time,
}

/// Resource for [MultiSisoController].
pub struct MultiSisoControllerResource<LeftMotor, RightMotor> {
    pub left_motor: LeftMotor,
    pub right_motor: RightMotor,
}

impl<LeftMotor, RightMotor, Config, State, Resource> Construct<Config, State, Resource>
    for MultiSisoController<LeftMotor, RightMotor>
where
    Config: AsRef<MultiSisoControllerConfig>,
    Resource: AsMut<Option<MultiSisoControllerResource<LeftMotor, RightMotor>>>,
{
    fn construct<'a>(config: &'a Config, _state: &'a State, resource: &'a mut Resource) -> Self {
        let config = config.as_ref();
        let MultiSisoControllerResource {
            left_motor,
            right_motor,
        } = resource.as_mut().take().expect("Should never panic");
        Self::new(
            left_motor,
            right_motor,
            config.translational_parameters.clone(),
            config.rotational_parameters.clone(),
            config.period,
        )
    }
}

impl<LeftMotor, RightMotor, State, Resource> Deconstruct<State, Resource>
    for MultiSisoController<LeftMotor, RightMotor>
where
    State: Default,
    Resource: From<MultiSisoControllerResource<LeftMotor, RightMotor>>,
{
    fn deconstruct(self) -> (State, Resource) {
        let (left_motor, right_motor) = self.release();
        (
            Default::default(),
            MultiSisoControllerResource {
                left_motor,
                right_motor,
            }
            .into(),
        )
    }
}
