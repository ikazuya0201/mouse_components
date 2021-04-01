use core::fmt::Debug;
use core::ops::Mul;

use generic_array::ArrayLength;
use spin::Mutex;
use typenum::{consts::*, PowerOfTwo, Unsigned};

use super::initialize::{init_search_agent, init_search_commander};
use crate::commanders::CostNode;
use crate::defaults::aliases::{SearchAgent, SearchCommander};
use crate::defaults::{
    config::Config,
    resource::{Resource, ResourceBuilder},
    state::State,
};
use crate::mazes::WallNode;
use crate::nodes::Pattern;
use crate::nodes::{RunNode, SearchNode};
use crate::sensors::{DistanceSensor as IDistanceSensor, Encoder, Motor, IMU};
use crate::utils::probability::Probability;
use crate::wall_manager::{Wall, WallManager};

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

impl<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, Size>
    SearchOperator<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Math,
        Size,
    >
where
    Math: crate::utils::math::Math,
    LeftEncoder: Encoder,
    RightEncoder: Encoder,
    Imu: IMU,
    LeftMotor: Motor,
    RightMotor: Motor,
    DistanceSensor: IDistanceSensor,
    Size: Mul<Size> + Mul<U2> + Mul<U4> + Clone + PartialEq + Unsigned + PowerOfTwo,
    <Size as Mul<Size>>::Output: Mul<U2> + Mul<U16> + Mul<U4>,
    <Size as Mul<U2>>::Output: ArrayLength<Wall<Size>>,
    <Size as Mul<U4>>::Output: ArrayLength<(RunNode<Size>, u16)>
        + ArrayLength<WallNode<Wall<Size>, (RunNode<Size>, Pattern)>>,
    <<Size as Mul<Size>>::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
    <<Size as Mul<Size>>::Output as Mul<U4>>::Output: ArrayLength<SearchNode<Size>>,
    <<Size as Mul<Size>>::Output as Mul<U16>>::Output: ArrayLength<CostNode<u16, RunNode<Size>>>
        + ArrayLength<CostNode<u16, SearchNode<Size>>>
        + ArrayLength<SearchNode<Size>>
        + ArrayLength<u16>
        + ArrayLength<Option<RunNode<Size>>>,
    Imu::Error: Debug,
    LeftEncoder::Error: Debug,
    RightEncoder::Error: Debug,
{
    pub fn new(
        config: &Config<Size>,
        state: &State<Size>,
        resource: Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor>,
        wall_manager: &'a WallManager<Size>,
    ) -> Self {
        let commander = init_search_commander(config, state, wall_manager);
        let agent = init_search_agent(config, state, resource, wall_manager);
        Self(crate::operators::TrackingOperator::new(commander, agent))
    }

    pub fn release(
        self,
    ) -> (
        State<Size>,
        Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor>,
    ) {
        let Self(inner) = self;
        let (commander, agent) = inner.release();
        let (current_node, _) = commander.release();
        let (_, robot) = agent.release();
        let (estimator, tracker, wall_detector) = robot.release();
        let (left_encoder, right_encoder, imu, robot_state) = estimator.release();
        let (left_motor, right_motor) = tracker.release();
        let distance_sensors = wall_detector.release().release();
        let resource = ResourceBuilder::new()
            .left_encoder(left_encoder)
            .right_encoder(right_encoder)
            .imu(imu)
            .left_motor(left_motor)
            .right_motor(right_motor)
            .distance_sensors(distance_sensors)
            .build()
            .expect("Should never panic");
        let state = State::new(current_node, robot_state);
        (state, resource)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_and_release() {
        use super::super::mocks::{DistanceSensorMock, EncoderMock, ImuMock, MotorMock};
        use crate::defaults::config::generate_default_config;
        use crate::nodes::{AbsoluteDirection, RunNode};
        use crate::utils::math::MathFake;
        use crate::utils::probability::Probability;
        use crate::wall_manager::WallManager;

        let config = generate_default_config();
        let resource = ResourceBuilder::new()
            .left_encoder(EncoderMock)
            .right_encoder(EncoderMock)
            .imu(ImuMock)
            .left_motor(MotorMock)
            .right_motor(MotorMock)
            .distance_sensors(core::array::IntoIter::new([DistanceSensorMock]).collect())
            .build()
            .unwrap();
        let state = State::new(
            RunNode::new(0, 0, AbsoluteDirection::North).unwrap().into(),
            Default::default(),
        );
        let wall_manager = WallManager::new(Probability::new(0.1).unwrap());
        let operator = SearchOperator::<_, _, _, _, _, _, MathFake, _>::new(
            &config,
            &state,
            resource,
            &wall_manager,
        );
        let (_, _) = operator.release();
    }
}
