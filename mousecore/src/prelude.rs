//! Prelude of this crate.
//!
//! ### Example
//! ```
//! use mousecore::prelude::*;
//! ```

#[doc(no_inline)]
pub use crate::administrator::{
    Operator as _components_administrator_Operator,
    OperatorStore as _components_administrator_OperatorStore,
    Selector as _components_administrator_Selector,
};

#[doc(no_inline)]
pub use crate::operators::{
    TrackingAgent as _components_operators_TrackingAgent,
    TrackingCommander as _components_operators_TrackingCommander,
};

#[doc(no_inline)]
pub use crate::commanders::{
    AsIndex as _components_commanders_AsIndex,
    GeometricGraph as _components_commanders_GeometricGraph, Graph as _components_commanders_Graph,
    NextNode as _components_commanders_NextNode, NodeChecker as _components_commanders_NodeChecker,
    RouteNode as _components_commanders_RouteNode,
    UncheckedNodeFinder as _components_commanders_UncheckedNodeFinder,
};

#[doc(no_inline)]
pub use crate::robot::{
    Estimator as _components_robot_StateEstimator, Tracker as _components_robot_Tracker,
};

#[doc(no_inline)]
pub use crate::tracker::Controller as _components_tracker_Controller;

#[doc(no_inline)]
pub use crate::sensors::{
    DistanceSensor as _components_sensors_DistanceSensor, Encoder as _components_sensors_Encoder,
    Imu as _components_sensors_Imu, Motor as _components_sensors_Motor,
};

#[doc(no_inline)]
pub use crate::mazes::{
    GeometricNode as _components_maze_GeometricNode, GraphNode as _components_maze_GraphNode,
    WallChecker as _components_maze_WallManager, WallFinderNode as _components_maze_WallFinderNode,
    WallSpaceNode as _components_maze_WallSpaceNode,
};

#[doc(no_inline)]
pub use crate::trajectory_managers::{
    CommandConverter as _components_trajectory_managers_CommandConverter,
    SearchTrajectoryGenerator as _components_trajectory_managers_SearchTrajectoryGenerator,
    TrackingTrajectoryGenerator as _components_trajectory_managers_TrackingTrajectoryGenerator,
};

#[doc(no_inline)]
pub use crate::{
    Construct as _components_Construct, Deconstruct as _components_Deconstruct,
    Merge as _components_Merge,
};

#[doc(no_inline)]
pub use crate::agents::{
    Robot as _components_agents_Robot, TrackingAgent as _components_agents_TrackingAgent,
    TrackingTrajectoryManager as _components_agents_TrackingTrajectoryManager,
};
