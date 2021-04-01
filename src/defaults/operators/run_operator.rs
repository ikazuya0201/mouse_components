use core::fmt::Debug;
use core::ops::Mul;

use generic_array::ArrayLength;
use spin::Mutex;
use typenum::{consts::*, PowerOfTwo, Unsigned};

use super::initialize::{init_return_commander, init_run_agent, init_run_commander};
use crate::commanders::CostNode;
use crate::defaults::aliases::{RunAgent, RunCommander};
use crate::defaults::{
    config::Config,
    resource::{Resource, ResourceBuilder},
    state::State,
};
use crate::mazes::WallNode;
use crate::nodes::Pattern;
use crate::nodes::{RunNode, SearchNode};
use crate::sensors::{DistanceSensor as IDistanceSensor, Encoder, Motor, IMU};
use crate::trajectory_generators::RunKind;
use crate::utils::probability::Probability;
use crate::wall_manager::{Wall, WallManager};

macro_rules! impl_run_operator {
    ($operator: ident: $commander_fn: ident) => {
        pub struct $operator<
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
                RunCommander<'a, Size>,
                RunAgent<
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
            Size: Mul<Size> + Clone,
            Size::Output: Mul<U2>,
            <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
            Math: crate::utils::math::Math;

        impl<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, Size>
            $operator<
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
                + ArrayLength<Option<usize>>
                + ArrayLength<u16>
                + ArrayLength<(RunNode<Size>, RunKind)>
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
                let commander = $commander_fn(config, state, wall_manager);
                let agent = init_run_agent(config, state, resource, wall_manager);
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
                let state = State::new(current_node.into(), robot_state);
                (state, resource)
            }
        }
    }
}

impl_run_operator!(RunOperator: init_run_commander);
impl_run_operator!(ReturnOperator: init_return_commander);
