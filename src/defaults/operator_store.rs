//! Implementations of [OperatorStore](crate::administrator::OperatorStore) and mode.

use core::fmt::Debug;

use serde::{Deserialize, Serialize};

use crate::administrator::{OperatorStore as IOperatorStore, Successor};
use crate::defaults::{
    alias::*, config::ConfigContainer, operator::Operators, resource::ResourceContainer,
    state::StateContainer,
};
use crate::sensors::*;
use crate::{Construct, Deconstruct};

/// Mode for describing current state of operator.
#[derive(Clone, Copy, PartialEq, Eq, Debug, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Mode {
    Search(u8),
    ReturnSetup(u8),
    Return(u8),
    RunSetup(u8),
    Run(u8),
    Idle,
}

impl Successor for Mode {
    fn successor(&self) -> Self {
        use Mode::*;

        match *self {
            Search(count) => ReturnSetup(count),
            ReturnSetup(count) => Return(count),
            Return(count) => {
                if count == 0 {
                    Idle
                } else {
                    RunSetup(count - 1)
                }
            }
            RunSetup(count) => Run(count),
            Run(count) => ReturnSetup(count),
            _ => Idle,
        }
    }
}

/// An implementation of [OperatorStore](crate::administrator::OperatorStore).
pub struct OperatorStore<const N: usize> {
    config: ConfigContainer<N>,
}

impl<const N: usize> OperatorStore<N> {
    pub fn new(config: ConfigContainer<N>) -> Self {
        Self { config }
    }
}

impl<
        LeftEncoder,
        RightEncoder,
        ImuType,
        LeftMotor,
        RightMotor,
        DistanceSensorType,
        const N: usize,
    >
    IOperatorStore<
        Mode,
        Operators<LeftEncoder, RightEncoder, ImuType, LeftMotor, RightMotor, DistanceSensorType, N>,
    > for OperatorStore<N>
where
    LeftEncoder: Encoder,
    RightEncoder: Encoder,
    ImuType: Imu,
    RightMotor: Motor,
    LeftMotor: Motor,
    DistanceSensorType: DistanceSensor,
    LeftEncoder::Error: Debug,
    RightEncoder::Error: Debug,
    ImuType::Error: Debug,
    DistanceSensorType::Error: Debug,
{
    fn exchange(
        &self,
        operator: Operators<
            LeftEncoder,
            RightEncoder,
            ImuType,
            LeftMotor,
            RightMotor,
            DistanceSensorType,
            N,
        >,
        mode: Mode,
    ) -> Operators<LeftEncoder, RightEncoder, ImuType, LeftMotor, RightMotor, DistanceSensorType, N>
    {
        let (mut state, mut resource) = operator.deconstruct();
        let state: StateContainer<N> = state.build().expect("Should never panic").into();
        let mut resource: ResourceContainer<
            LeftEncoder,
            RightEncoder,
            ImuType,
            LeftMotor,
            RightMotor,
            DistanceSensorType,
            N,
        > = resource.build().expect("Should never panic").into();

        match mode {
            Mode::Search(_) => Operators::Search(SearchOperator::construct(
                &self.config,
                &state,
                &mut resource,
            )),
            Mode::ReturnSetup(_) => Operators::ReturnSetup(ReturnSetupOperator::construct(
                &self.config,
                &state,
                &mut resource,
            )),
            Mode::Return(_) => Operators::Return(ReturnOperator::construct(
                &self.config,
                &state,
                &mut resource,
            )),
            Mode::RunSetup(_) => Operators::RunSetup(RunSetupOperator::construct(
                &self.config,
                &state,
                &mut resource,
            )),
            Mode::Run(_) => {
                Operators::Run(RunOperator::construct(&self.config, &state, &mut resource))
            }
            _ => todo!(),
        }
    }
}
