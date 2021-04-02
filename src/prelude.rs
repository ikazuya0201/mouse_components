//! Prelude of this crate.
//!
//! ### Example
//! ```
//! use components::prelude::*;
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
    BoundedNode as _components_commanders_BoundedNode, Graph as _components_commanders_Graph,
    NextNode as _components_commanders_NextNode, NodeChecker as _components_commanders_NodeChecker,
    RouteNode as _components_commanders_RouteNode,
    UncheckedNodeFinder as _components_commanders_UncheckedNodeFinder,
};

#[doc(no_inline)]
pub use crate::robot::{
    StateEstimator as _components_robot_StateEstimator, Tracker as _components_robot_Tracker,
};

#[doc(no_inline)]
pub use crate::tracker::Controller as _components_tracker_Controller;

#[doc(no_inline)]
pub use crate::sensors::{
    DistanceSensor as _components_sensors_DistanceSensor, Encoder as _components_sensors_Encoder,
    Motor as _components_sensors_Motor, IMU as _components_sensors_IMU,
};

#[doc(no_inline)]
pub use crate::utils::math::Math as _utils_math_Math;

#[doc(no_inline)]
pub use crate::mazes::{
    GraphNode as _components_maze_GraphNode, WallChecker as _components_maze_WallManager,
    WallFinderNode as _components_maze_WallFinderNode,
    WallSpaceNode as _components_maze_WallSpaceNode,
};

#[doc(no_inline)]
pub use crate::trajectory_managers::{
    CommandConverter as _components_trajectory_managers_CommandConverter,
    SearchTrajectoryGenerator as _components_trajectory_managers_SearchTrajectoryGenerator,
    TrackingTrajectoryGenerator as _components_trajectory_managers_TrackingTrajectoryGenerator,
};

#[doc(no_inline)]
pub use crate::wall_detector::ObstacleDetector as _components_wall_detector_ObstacleDetector;
