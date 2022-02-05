#[allow(unused_imports)]
use micromath::F32Ext;
use serde::{Deserialize, Serialize};
use uom::si::f32::{
    Acceleration, Angle, AngularAcceleration, AngularVelocity, Length, Time, Velocity,
};

#[derive(Clone, Debug, Default, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct State {
    pub x: LengthState,
    pub y: LengthState,
    pub theta: AngleState,
    pub slip_angle: Angle,
    pub translational_velocity: Velocity,
}

#[derive(Clone, Debug, Default, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct LengthState {
    pub x: Length,
    pub v: Velocity,
    pub a: Acceleration,
}

#[derive(Clone, Debug, Default, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct AngleState {
    pub x: Angle,
    pub v: AngularVelocity,
    pub a: AngularAcceleration,
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct SensorValue {
    pub left_distance: Length,
    pub right_distance: Length,
    pub translational_acceleration: Acceleration,
    pub angular_velocity: AngularVelocity,
}

#[derive(Debug)]
pub struct Estimator {
    period: Time,
    alpha: f32,
    slip_angle_const: Acceleration,
    approx_th: AngularVelocity,
}

impl Estimator {
    pub fn new(
        period: Time,
        alpha: f32,
        slip_angle_const: Acceleration,
        approx_th: AngularVelocity,
    ) -> Self {
        Self {
            period,
            alpha,
            slip_angle_const,
            approx_th,
        }
    }

    pub fn estimate(
        &self,
        mut state: State,
        &SensorValue {
            left_distance: ldist,
            right_distance: rdist,
            translational_acceleration: trans_acc,
            angular_velocity: ang_vel,
        }: &SensorValue,
    ) -> State {
        let average_left_velocity = ldist / self.period;
        let average_right_velocity = rdist / self.period;

        let average_trans_velocity = (average_left_velocity + average_right_velocity) / 2.0;

        state.translational_velocity = self.alpha
            * (state.translational_velocity + trans_acc * self.period)
            + (1.0 - self.alpha) * average_trans_velocity;

        state.slip_angle = {
            let rev_period = 1.0 / self.period;
            Angle::from(
                (AngularVelocity::from(state.slip_angle * rev_period) + ang_vel)
                    / (rev_period + self.slip_angle_const / state.translational_velocity.abs()),
            )
        };

        let dtheta = Angle::from(ang_vel * self.period);
        let trans_distance = if ang_vel.abs() < self.approx_th {
            // straight approximation
            state.translational_velocity * self.period
        } else {
            // arc approximation
            2.0 * state.translational_velocity * (dtheta / 2.0).value.sin() / ang_vel
        };

        let theta_m = state.theta.x - state.slip_angle + dtheta / 2.0;
        let sin_th = theta_m.value.sin();
        let cos_th = theta_m.value.cos();
        state.x.x += trans_distance * cos_th;
        state.y.x += trans_distance * sin_th;
        state.theta.x += dtheta;

        state.x.v = state.translational_velocity * cos_th;
        state.y.v = state.translational_velocity * sin_th;
        state.x.a = trans_acc * cos_th;
        state.y.a = trans_acc * sin_th;

        state.theta.a = AngularAcceleration::from((ang_vel - state.theta.v) / self.period);
        state.theta.v = ang_vel;

        state
    }
}
