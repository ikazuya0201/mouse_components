//! Default implementation of operator.

use core::convert::Infallible;
use core::fmt::Debug;

use crate::administrator::{Operator as IOperator, OperatorError};
use crate::agents::TrackingAgentError;
use crate::commanders::{RunCommanderError, SetupCommanderError};
use crate::controllers::FailSafeError;
use crate::defaults::alias::{
    ReturnOperator, ReturnSetupOperator, RunOperator, RunSetupOperator, SearchOperator,
};
use crate::operators::TrackingOperatorError;
use crate::sensors::*;
use crate::trajectory_managers::{SearchTrajectoryManagerError, TrackingTrajectoryManagerError};

/// Enum which contains all operators.
pub enum Operators<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    const N: usize,
> {
    Search(
        SearchOperator<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>,
    ),
    ReturnSetup(
        ReturnSetupOperator<
            LeftEncoder,
            RightEncoder,
            Imu,
            LeftMotor,
            RightMotor,
            DistanceSensor,
            N,
        >,
    ),
    Return(
        ReturnOperator<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>,
    ),
    RunSetup(
        RunSetupOperator<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>,
    ),
    Run(RunOperator<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>),
}

type SearchOperatorError = TrackingOperatorError<
    TrackingAgentError<FailSafeError, SearchTrajectoryManagerError>,
    Infallible,
>;

type SetupOperatorError = TrackingOperatorError<
    TrackingAgentError<FailSafeError, TrackingTrajectoryManagerError>,
    SetupCommanderError,
>;

type RunOperatorError = TrackingOperatorError<
    TrackingAgentError<FailSafeError, TrackingTrajectoryManagerError>,
    RunCommanderError,
>;

/// Error on [Operators].
#[derive(Debug)]
pub enum OperatorsError {
    Search(SearchOperatorError),
    Setup(SetupOperatorError),
    Run(RunOperatorError),
}

impl From<SearchOperatorError> for OperatorsError {
    fn from(value: SearchOperatorError) -> Self {
        OperatorsError::Search(value)
    }
}

impl From<SetupOperatorError> for OperatorsError {
    fn from(value: SetupOperatorError) -> Self {
        OperatorsError::Setup(value)
    }
}

impl From<RunOperatorError> for OperatorsError {
    fn from(value: RunOperatorError) -> Self {
        OperatorsError::Run(value)
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
    > IOperator
    for Operators<LeftEncoder, RightEncoder, ImuType, LeftMotor, RightMotor, DistanceSensorType, N>
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
    type Error = OperatorsError;

    fn tick(&self) -> Result<(), Self::Error> {
        use Operators::*;

        match self {
            Search(operator) => operator.tick()?,
            ReturnSetup(operator) => operator.tick()?,
            Return(operator) => operator.tick()?,
            RunSetup(operator) => operator.tick()?,
            Run(operator) => operator.tick()?,
        }
        Ok(())
    }

    fn run(&self) -> Result<(), OperatorError<Self::Error>> {
        use Operators::*;

        fn convert<T, U: From<T>>(error: OperatorError<T>) -> OperatorError<U> {
            use OperatorError::*;

            match error {
                Incompleted => Incompleted,
                Other(error) => Other(U::from(error)),
            }
        }

        match self {
            Search(operator) => operator.run().map_err(convert),
            ReturnSetup(operator) => operator.run().map_err(convert),
            Return(operator) => operator.run().map_err(convert),
            RunSetup(operator) => operator.run().map_err(convert),
            Run(operator) => operator.run().map_err(convert),
        }
    }
}
