//! Components for implementing a code of [Micromouse contest](https://www.ntf.or.jp/mouse/).

#![cfg_attr(not(test), no_std)]
#![deny(warnings)]
#![cfg_attr(nightly, feature(iter_advance_by))]

extern crate alloc;

pub mod administrator;
pub mod agents;
pub mod command_converter;
pub mod commanders;
pub mod controllers;
pub mod defaults;
pub mod estimator;
pub mod mazes;
pub mod nodes;
pub mod operators;
pub mod prelude;
pub mod robot;
pub mod tracker;
pub mod trajectory_generators;
pub mod trajectory_managers;
#[macro_use]
pub mod utils;
pub mod pattern_converters;
pub mod wall_detector;
pub mod wall_manager;

/// Traits in this crate.
pub mod traits {
    use super::*;

    #[doc(no_inline)]
    pub use administrator::{Operator, OperatorStore, Selector};
    #[doc(no_inline)]
    pub use commanders::{
        AsIndex, GeometricGraph, Graph, NextNode, NodeChecker, RouteNode, UncheckedNodeFinder,
    };
    #[doc(no_inline)]
    pub use mazes::{GeometricNode, GraphNode, WallChecker, WallFinderNode, WallSpaceNode};
    #[doc(no_inline)]
    pub use operators::{TrackingAgent, TrackingCommander};
    #[doc(no_inline)]
    pub use tracker::Controller;
}

/// Types in this crate.
pub mod types {
    use super::*;
    /// Data types.
    pub mod data {
        use super::*;

        #[doc(no_inline)]
        pub use self::utils::{
            direction::{AbsoluteDirection, RelativeDirection},
            robot_state::{AngleState, LengthState, RobotState},
        };
        #[doc(no_inline)]
        pub use controllers::ControlParameters;
        #[doc(no_inline)]
        pub use nodes::{Node, Pattern, Position, RotationKind, RunNode, SearchNode};
        #[doc(no_inline)]
        pub use tracker::ControlTarget;
        #[doc(no_inline)]
        pub use trajectory_generators::Pose;
        #[doc(no_inline)]
        pub use trajectory_generators::{
            AngleTarget, LengthTarget, RunKind, RunState, SearchKind, SlalomDirection, SlalomKind,
            SlalomParameters, Target,
        };
        #[doc(no_inline)]
        pub use wall_manager::Wall;
    }

    /// Configs for construction.
    pub mod configs {
        use super::*;

        #[doc(no_inline)]
        pub use command_converter::CommandConverterConfig;
        #[doc(no_inline)]
        pub use commanders::{
            ReturnCommanderConfig, ReturnSetupCommanderConfig, RunCommanderConfig,
            RunSetupCommanderConfig, SearchCommanderConfig,
        };
        #[doc(no_inline)]
        pub use controllers::MultiSisoControllerConfig;
        #[doc(no_inline)]
        pub use estimator::EstimatorConfig;
        #[doc(no_inline)]
        pub use tracker::TrackerConfig;
        #[doc(no_inline)]
        pub use trajectory_generators::{
            ReturnSetupTrajectoryGeneratorConfig, RunTrajectoryGeneratorConfig,
            SearchTrajectoryGeneratorConfig,
        };
        #[doc(no_inline)]
        pub use wall_detector::WallDetectorConfig;
    }

    /// States for construction.
    pub mod states {
        use super::*;

        #[doc(no_inline)]
        pub use commanders::CommanderState;
        #[doc(no_inline)]
        pub use estimator::EstimatorState;
    }

    /// Resources for construction.
    pub mod resources {
        use super::*;

        #[doc(no_inline)]
        pub use controllers::MultiSisoControllerResource;
        #[doc(no_inline)]
        pub use estimator::EstimatorResource;
        #[doc(no_inline)]
        pub use wall_detector::WallDetectorResource;
    }
}

/// Traits of sensors.
pub mod sensors {
    use super::*;

    #[doc(no_inline)]
    pub use controllers::Motor;
    #[doc(no_inline)]
    pub use estimator::{Encoder, Imu};
    #[doc(no_inline)]
    pub use wall_detector::DistanceSensor;
}

const MAZE_WIDTH_UPPER_BOUND: usize = 32;

/// A trait for constructing itself by config, state and resource.
pub trait Construct<Config, State, Resource>: Sized {
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self;
}

/// A trait for deconstructing itself into state and resource.
pub trait Deconstruct<State, Resource> {
    fn deconstruct(self) -> (State, Resource);
}

/// A conversion for merging two values.
pub trait Merge {
    fn merge(self, rhs: Self) -> Self;
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_deconstruct_with_default {
    ($name: ident $(<$($param: ident),*>)?) => {
        impl<$($($param,)*)? State, Resource> crate::Deconstruct<State, Resource> for $name $(<$($param),*>)?
            where State: Default,
                  Resource: Default,
        {
            fn deconstruct(self) -> (State, Resource) {
                (Default::default(), Default::default())
            }
        }
    }
}
