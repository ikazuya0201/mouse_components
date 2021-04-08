use core::fmt::Debug;

use super::initialize::{init_search_agent, init_search_commander};
use crate::defaults::aliases::{SearchAgent, SearchCommander};
use crate::defaults::{
    config::Config,
    resource::{Resource, ResourceBuilder},
    state::State,
};
use crate::sensors::{DistanceSensor as IDistanceSensor, Encoder, Motor, IMU};
use crate::wall_manager::WallManager;

pub struct SearchOperator<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    const N: usize,
>(
    crate::operators::TrackingOperator<
        SearchCommander<'a, N>,
        SearchAgent<
            'a,
            LeftEncoder,
            RightEncoder,
            Imu,
            LeftMotor,
            RightMotor,
            DistanceSensor,
            Math,
            N,
        >,
    >,
)
where
    Math: crate::utils::math::Math;

impl<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Math,
        const N: usize,
    >
    SearchOperator<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Math,
        N,
    >
where
    Math: crate::utils::math::Math,
    LeftEncoder: Encoder,
    RightEncoder: Encoder,
    Imu: IMU,
    LeftMotor: Motor,
    RightMotor: Motor,
    DistanceSensor: IDistanceSensor,
    Imu::Error: Debug,
    LeftEncoder::Error: Debug,
    RightEncoder::Error: Debug,
{
    pub fn new(
        config: &Config<N>,
        state: &State<N>,
        resource: Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor>,
        wall_manager: &'a WallManager<N>,
    ) -> Self {
        let commander = init_search_commander(config, state, wall_manager);
        let agent = init_search_agent(config, state, resource, wall_manager);
        Self(crate::operators::TrackingOperator::new(commander, agent))
    }

    pub fn release(
        self,
    ) -> (
        State<N>,
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
        use super::super::mocks::default_resource;
        use crate::defaults::config::default_config;
        use crate::nodes::{AbsoluteDirection, RunNode};
        use crate::utils::math::MathFake;
        use crate::utils::probability::Probability;
        use crate::wall_manager::WallManager;

        let config = default_config();
        let resource = default_resource();
        let state = State::new(
            RunNode::new(0, 0, AbsoluteDirection::North).unwrap().into(),
            Default::default(),
        );
        let wall_manager = WallManager::new(Probability::new(0.1).unwrap());
        let operator = SearchOperator::<_, _, _, _, _, _, MathFake, 4>::new(
            &config,
            &state,
            resource,
            &wall_manager,
        );
        let (_, _) = operator.release();
    }
}
