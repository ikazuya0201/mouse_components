//! An implementation of config.

use uom::si::f32::{
    Acceleration, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Time, Velocity,
};

use crate::command_converter::CommandConverterConfig;
use crate::commanders::{
    ReturnCommanderConfig, ReturnSetupCommanderConfig, RunCommanderConfig, SearchCommanderConfig,
};
use crate::controllers::{RotationControllerConfig, TranslationControllerConfig};
use crate::estimator::EstimatorConfig;
use crate::mazes::MazeConfig;
use crate::pose_converter::PoseConverterConfig;
use crate::tracker::TrackerConfig;
use crate::trajectory_generators::{
    ReturnSetupTrajectoryGeneratorConfig, RunTrajectoryGeneratorConfig,
    SearchTrajectoryGeneratorConfig,
};
use crate::trajectory_generators::{SlalomDirection, SlalomKind, SlalomParameters};
use crate::utils::builder::{ok_or, BuilderResult};

/// An implementation of config.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config<'a, Node, Route, Pattern, Cost> {
    square_width: Length,
    front_offset: Length,
    start: Node,
    return_goal: Node,
    goals: &'a [Node],
    search_initial_route: Route,
    search_final_route: Route,
    trans_kp: f32,
    trans_ki: f32,
    trans_kd: f32,
    period: Time,
    trans_model_gain: f32,
    trans_model_time_constant: Time,
    rot_kp: f32,
    rot_ki: f32,
    rot_kd: f32,
    rot_model_gain: f32,
    rot_model_time_constant: Time,
    estimator_correction_weight: f32,
    wheel_interval: Option<Length>,
    estimator_trans_velocity_alpha: f32,
    cost_fn: fn(Pattern) -> Cost,
    wall_width: Length,
    ignore_radius_from_pillar: Length,
    ignore_length_from_wall: Length,
    kx: f32,
    kdx: f32,
    ky: f32,
    kdy: f32,
    valid_control_lower_bound: Velocity,
    fail_safe_distance: Length,
    low_zeta: f32,
    low_b: f32,
    run_slalom_velocity: Velocity,
    max_velocity: Velocity,
    max_acceleration: Acceleration,
    max_jerk: Jerk,
    slalom_angular_velocity_ref: AngularVelocity,
    slalom_angular_acceleration_ref: AngularAcceleration,
    slalom_angular_jerk_ref: AngularJerk,
    slalom_parameters_map: fn(SlalomKind, SlalomDirection) -> SlalomParameters,
    search_velocity: Velocity,
    spin_angular_velocity: AngularVelocity,
    spin_angular_acceleration: AngularAcceleration,
    spin_angular_jerk: AngularJerk,
}

macro_rules! impl_into {
    ($into: ident { $($into_field: ident: $from_field: ident,)* }) => {
        impl<'a, Node, Route, Pattern, Cost> Into<$into>
            for &'a Config<'a, Node, Route, Pattern, Cost>
        {
            fn into(self) -> $into {
                $into {
                    $($into_field: self.$from_field.clone(),)+
                }
            }
        }
    };

    ($into: ident < $($param: ident),* > { $($into_field: ident: $from_field: ident,)* }) => {
        impl<'a, Node, Route, Pattern, Cost> Into<$into<$($param),*>>
            for &'a Config<'a, Node, Route, Pattern, Cost>
        where $($param: Clone),*
        {
            fn into(self) -> $into<$($param),*> {
                $into {
                    $($into_field: self.$from_field.clone(),)+
                }
            }
        }
    };

    ($into: ident < $lt: lifetime, $($param: ident),* > { $($into_field: ident: $from_field: ident,)* }) => {
        impl<'a, Node, Route, Pattern, Cost> Into<$into<$lt, $($param),*>>
            for &'a Config<'a, Node, Route, Pattern, Cost>
        where $($param: Clone),*
        {
            fn into(self) -> $into<$lt, $($param),*> {
                $into {
                    $($into_field: self.$from_field.clone(),)+
                }
            }
        }
    };
}

impl_into! {
    CommandConverterConfig {
        square_width: square_width,
        front_offset: front_offset,
    }
}

impl_into! {
    ReturnSetupCommanderConfig<Node> {
        start: return_goal,
    }
}

impl_into! {
    RunCommanderConfig<'a, Node> {
        start: start,
        goals: goals,
    }
}

impl_into! {
    ReturnCommanderConfig<Node> {
        start: return_goal,
    }
}

impl_into! {
    SearchCommanderConfig<'a, Node, Route> {
        start: start,
        goals: goals,
        initial_route: search_initial_route,
        final_route: search_final_route,
    }
}

impl_into! {
    RotationControllerConfig {
        kp: rot_kp,
        ki: rot_ki,
        kd: rot_kd,
        period: period,
        model_gain: rot_model_gain,
        model_time_constant: rot_model_time_constant,
    }
}

impl_into! {
    TranslationControllerConfig {
        kp: trans_kp,
        ki: trans_ki,
        kd: trans_kd,
        period: period,
        model_gain: trans_model_gain,
        model_time_constant: trans_model_time_constant,
    }
}

impl_into! {
    EstimatorConfig {
        period: period,
        weight: estimator_correction_weight,
        wheel_interval: wheel_interval,
        alpha: estimator_trans_velocity_alpha,
    }
}

impl_into! {
    MazeConfig<Pattern, Cost> {
        cost_fn: cost_fn,
    }
}

impl_into! {
    PoseConverterConfig {
        square_width: square_width,
        wall_width: wall_width,
        ignore_radius_from_pillar: ignore_radius_from_pillar,
        ignore_length_from_wall: ignore_length_from_wall,
    }
}

impl_into! {
    TrackerConfig {
        kx: kx,
        kdx: kdx,
        ky: ky,
        kdy: kdy,
        fail_safe_distance: fail_safe_distance,
        valid_control_lower_bound: valid_control_lower_bound,
        low_b: low_b,
        low_zeta: low_zeta,
        period: period,
    }
}

impl_into! {
    ReturnSetupTrajectoryGeneratorConfig {
        max_angular_velocity: spin_angular_velocity,
        max_angular_acceleration: spin_angular_acceleration,
        max_angular_jerk: spin_angular_jerk,
        period: period,
    }
}

impl_into! {
    SearchTrajectoryGeneratorConfig {
        angular_velocity_ref: slalom_angular_velocity_ref,
        angular_acceleration_ref: slalom_angular_acceleration_ref,
        angular_jerk_ref: slalom_angular_jerk_ref,
        slalom_parameters_map: slalom_parameters_map,
        max_velocity: max_velocity,
        max_acceleration: max_acceleration,
        max_jerk: max_jerk,
        period: period,
        search_velocity: search_velocity,
        front_offset: front_offset,
        square_width: square_width,
        spin_angular_velocity: spin_angular_velocity,
        spin_angular_acceleration: spin_angular_acceleration,
        spin_angular_jerk: spin_angular_jerk,
    }
}

impl_into! {
    RunTrajectoryGeneratorConfig {
        run_slalom_velocity: run_slalom_velocity,
        max_velocity: max_velocity,
        max_acceleration: max_acceleration,
        max_jerk: max_jerk,
        angular_velocity_ref: slalom_angular_velocity_ref,
        angular_acceleration_ref: slalom_angular_acceleration_ref,
        angular_jerk_ref: slalom_angular_jerk_ref,
        slalom_parameters_map: slalom_parameters_map,
        period: period,
    }
}

/// A builder for [Config](Config).
///
/// # Example
/// ```
/// use typenum::*;
/// use uom::si::f32::{
///     Length, Velocity, Acceleration, Jerk, AngularVelocity, AngularAcceleration, AngularJerk,
///     Time,
/// };
/// use uom::si::{
///     acceleration::meter_per_second_squared,
///     angular_acceleration::degree_per_second_squared, angular_jerk::degree_per_second_cubed,
///     angular_velocity::degree_per_second, jerk::meter_per_second_cubed, length::meter,
///     time::second, velocity::meter_per_second,
/// };
/// use components::nodes::RunNode;
/// use components::types::data::{AbsoluteDirection, Pattern, SearchKind};
/// use components::config::ConfigBuilder;
///
/// use AbsoluteDirection::*;
///
/// type Size = U4;
///
/// let start = RunNode::<Size>::new(0, 0, North).unwrap();
/// let return_goal = RunNode::<Size>::new(0, 0, South).unwrap();
/// let goals = vec![
///     RunNode::<Size>::new(2, 0, South).unwrap(),
///     RunNode::<Size>::new(2, 0, West).unwrap(),
/// ];
///
/// fn cost(_pattern: Pattern) -> u16 {
///     unreachable!()
/// }
///
/// let config = ConfigBuilder::new()
///     .start(start)
///     .return_goal(return_goal)
///     .goals(&goals)
///     .search_initial_route(SearchKind::Init)
///     .search_final_route(SearchKind::Final)
///     .cost_fn(cost)
///     .estimator_trans_velocity_alpha(0.1)
///     .period(Time::new::<second>(0.001))
///     .estimator_correction_weight(0.1)
///     .translational_kp(0.9)
///     .translational_ki(0.05)
///     .translational_kd(0.01)
///     .translational_model_gain(1.0)
///     .translational_model_time_constant(Time::new::<second>(0.001))
///     .rotational_kp(0.2)
///     .rotational_ki(0.2)
///     .rotational_kd(0.0)
///     .rotational_model_gain(1.0)
///     .rotational_model_time_constant(Time::new::<second>(0.001))
///     .tracker_kx(15.0)
///     .tracker_kdx(4.0)
///     .tracker_ky(15.0)
///     .tracker_kdy(4.0)
///     .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.03))
///     .low_zeta(1.0)
///     .low_b(1e-3)
///     .fail_safe_distance(uom::si::f32::Length::new::<meter>(0.05))
///     .search_velocity(Velocity::new::<meter_per_second>(0.12))
///     .max_velocity(Velocity::new::<meter_per_second>(2.0))
///     .max_acceleration(Acceleration::new::<meter_per_second_squared>(0.7))
///     .max_jerk(Jerk::new::<meter_per_second_cubed>(1.0))
///     .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(180.0))
///     .spin_angular_acceleration(AngularAcceleration::new::<degree_per_second_squared>(
///         1800.0,
///     ))
///     .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(7200.0))
///     .run_slalom_velocity(Velocity::new::<meter_per_second>(1.0))
///     .build()
///     .unwrap();
/// ```

pub struct ConfigBuilder<'a, Node, Route, Pattern, Cost> {
    square_width: Option<Length>,
    front_offset: Option<Length>,
    start: Option<Node>,
    return_goal: Option<Node>,
    goals: Option<&'a [Node]>,
    search_initial_route: Option<Route>,
    search_final_route: Option<Route>,
    trans_kp: Option<f32>,
    trans_ki: Option<f32>,
    trans_kd: Option<f32>,
    period: Option<Time>,
    trans_model_gain: Option<f32>,
    trans_model_time_constant: Option<Time>,
    rot_kp: Option<f32>,
    rot_ki: Option<f32>,
    rot_kd: Option<f32>,
    rot_model_gain: Option<f32>,
    rot_model_time_constant: Option<Time>,
    estimator_correction_weight: Option<f32>,
    wheel_interval: Option<Length>,
    estimator_trans_velocity_alpha: Option<f32>,
    cost_fn: Option<fn(Pattern) -> Cost>,
    wall_width: Option<Length>,
    ignore_radius_from_pillar: Option<Length>,
    ignore_length_from_wall: Option<Length>,
    kx: Option<f32>,
    kdx: Option<f32>,
    ky: Option<f32>,
    kdy: Option<f32>,
    valid_control_lower_bound: Option<Velocity>,
    fail_safe_distance: Option<Length>,
    low_zeta: Option<f32>,
    low_b: Option<f32>,
    run_slalom_velocity: Option<Velocity>,
    max_velocity: Option<Velocity>,
    max_acceleration: Option<Acceleration>,
    max_jerk: Option<Jerk>,
    slalom_angular_velocity_ref: Option<AngularVelocity>,
    slalom_angular_acceleration_ref: Option<AngularAcceleration>,
    slalom_angular_jerk_ref: Option<AngularJerk>,
    slalom_parameters_map: Option<fn(SlalomKind, SlalomDirection) -> SlalomParameters>,
    search_velocity: Option<Velocity>,
    spin_angular_velocity: Option<AngularVelocity>,
    spin_angular_acceleration: Option<AngularAcceleration>,
    spin_angular_jerk: Option<AngularJerk>,
}

macro_rules! impl_setter {
    (
        $(#[$meta:meta])*
        $method_name: ident: $field_name: ident: $type: ty
    ) => {
        $(#[$meta])*
        pub fn $method_name(&mut self, $field_name: $type) -> &mut Self {
            self.$field_name = Some($field_name);
            self
        }
    };

    (
        $(#[$meta:meta])*
        $field_name: ident: $type: ty
    ) => {
        impl_setter!(
            $(#[$meta])*
            $field_name: $field_name: $type
        );
    };
}

impl<'a, Node, Route, Pattern, Cost> ConfigBuilder<'a, Node, Route, Pattern, Cost> {
    impl_setter!(
        /// **Optional**,
        /// Default: 90 \[mm\] (half size).
        ///
        /// Sets the width of one square of maze.
        square_width: Length
    );
    impl_setter!(
        /// **Optional**,
        /// Default: 0.0 \[mm\] (no offset).
        ///
        /// Sets the front offset of search trajectory.
        front_offset: Length
    );
    impl_setter!(
        /// **Required**,
        /// Sets the start node for search.
        start: Node
    );
    impl_setter!(
        /// **Required**,
        /// Sets a goal for return to start.
        return_goal: Node
    );
    impl_setter!(
        /// **Required**,
        /// Sets the goal nodes for search.
        goals: &'a [Node]
    );
    impl_setter!(
        /// **Required**,
        /// Sets a route for initial trajectory.
        search_initial_route: Route
    );
    impl_setter!(
        /// **Required**,
        /// Sets a route for final trajectory.
        search_final_route: Route
    );
    impl_setter!(
        /// **Required**,
        /// Sets the period for control.
        period: Time
    );
    impl_setter!(
        /// **Required**,
        /// Sets the P gain for translation.
        translational_kp: trans_kp: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the I gain for translation.
        translational_ki: trans_ki: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the D gain for translation.
        translational_kd: trans_kd: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the model gain for translation.
        ///
        /// The model is used for the input of feed-forward.
        ///
        /// Assumes the model is the first-order delay system.
        translational_model_gain: trans_model_gain: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the model time constant for translation.
        ///
        /// The model is used for the input of feed-forward.
        ///
        /// Assumes the model is the first-order delay system.
        translational_model_time_constant: trans_model_time_constant: Time
    );
    impl_setter!(
        /// **Required**,
        /// Sets the P gain for rotation.
        rotational_kp: rot_kp: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the I gain for rotation.
        rotational_ki: rot_ki: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the D gain for rotation.
        rotational_kd: rot_kd: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the model gain for rotation.
        ///
        /// The model is used for the input of feed-forward.
        ///
        /// Assumes the model is the first-order delay system.
        rotational_model_gain: rot_model_gain: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the model time constant for rotation.
        ///
        /// The model is used for the input of feed-forward.
        ///
        /// Assumes the model is the first-order delay system.
        rotational_model_time_constant: rot_model_time_constant: Time
    );
    impl_setter!(
        /// **Optional**,
        /// Default: 0.0.
        ///
        /// Sets the weight for correcting the estimated state by the information of distance
        /// sensor.
        estimator_correction_weight: f32
    );
    impl_setter!(
        /// **Optional**,
        /// Default: None,
        ///
        /// Sets the interval of 2 (or 4) wheels.
        /// This value is used for estimation of the machine's posture.
        wheel_interval: Length
    );
    impl_setter!(
        /// **Required**,
        /// Sets the alpha for low pass filter of translational velocity.
        estimator_trans_velocity_alpha: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the cost function for search.
        cost_fn: fn(Pattern) -> Cost
    );
    impl_setter!(
        /// **Optional**,
        /// Default: 6 \[mm\] (half size).
        ///
        /// Sets the wall width of the maze.
        wall_width: Length
    );
    impl_setter!(
        /// **Optional**,
        /// Default: 10 \[mm\].
        ///
        /// Sets the range for ignoring the sensor value.
        /// [PoseConverter](crate::pose_converter::PoseConverter) ignores the sensor value in the
        /// a given radius from pillar.
        ignore_radius_from_pillar: Length
    );
    impl_setter!(
        /// **Optional**,
        /// Default: 8 \[mm\].
        ///
        /// Sets the distance for ignoring the sensor value.
        /// [PoseConverter](crate::pose_converter::PoseConverter) ignores the sensor value in the
        /// given distance from walls.
        ignore_length_from_wall: Length
    );
    impl_setter!(
        /// **Required**,
        /// Sets a control gain for tracking.
        tracker_kx: kx: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets a control gain for tracking.
        tracker_kdx: kdx: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets a control gain for tracking.
        tracker_ky: ky: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets a control gain for tracking.
        tracker_kdy: kdy: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the lower bound for valid control.
        ///
        /// The main control algorithm does not work near 0.0 \[m\/s\].
        /// [Tracker](crate::tracker::Tracker) switches the algorithm in this velocity.
        valid_control_lower_bound: Velocity
    );
    impl_setter!(
        /// **Required**,
        /// Sets the fail safe distance.
        ///
        /// [Tracker](crate::tracker::Tracker) fails when the length between the target and the
        /// estimated state exceed this value.
        fail_safe_distance: Length
    );
    impl_setter!(
        /// **Required**,
        /// Sets a control value for the algorithm in low velocity.
        low_zeta: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets a control value for the algorithm in low velocity.
        low_b: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the velocity for slalom in fast run.
        run_slalom_velocity: Velocity
    );
    impl_setter!(
        /// **Required**,
        /// Sets the upper bound of velocity.
        max_velocity: Velocity
    );
    impl_setter!(
        /// **Required**,
        /// Sets the upper bound of acceleration.
        max_acceleration: Acceleration
    );
    impl_setter!(
        /// **Required**,
        /// Sets the upper bound of jerk.
        max_jerk: Jerk
    );
    impl_setter!(
        /// **Optional**,
        /// Sets the base angular velocity for slalom.
        ///
        /// **Caution!**: This value should be constrained to slalom_parameters_map.
        /// This value can be deprecated.
        slalom_angular_velocity_ref: AngularVelocity
    );
    impl_setter!(
        /// **Optional**,
        /// Sets the base angular acceleration for slalom.
        ///
        /// **Caution!**: This value should be constrained to slalom_parameters_map.
        /// This value can be deprecated.
        slalom_angular_acceleration_ref: AngularAcceleration
    );
    impl_setter!(
        /// **Optional**,
        /// Sets the base angular jerk for slalom.
        ///
        /// **Caution!**: This value should be constrained to slalom_parameters_map.
        /// This value can be deprecated.
        slalom_angular_jerk_ref: AngularJerk
    );
    impl_setter!(
        /// **Optional**,
        /// Default:
        /// [slalom_parameters_map2](crate::trajectory_generators::slalom_parameters_map2).
        ///
        /// Sets the parameters for slalom.
        slalom_parameters_map: fn(SlalomKind, SlalomDirection) -> SlalomParameters
    );
    impl_setter!(
        /// **Required**,
        /// Sets the velocity for search.
        search_velocity: Velocity
    );
    impl_setter!(
        /// **Required**,
        /// Sets the max angular velocity for spin.
        spin_angular_velocity: AngularVelocity
    );
    impl_setter!(
        /// **Required**,
        /// Sets the max angular acceleration for spin.
        spin_angular_acceleration: AngularAcceleration
    );
    impl_setter!(
        /// **Required**,
        /// Sets the max angular jerk for spin.
        spin_angular_jerk: AngularJerk
    );

    /// Generates new builder whose values are set as None.
    pub fn new() -> Self {
        Self {
            square_width: None,
            front_offset: None,
            start: None,
            return_goal: None,
            goals: None,
            search_initial_route: None,
            search_final_route: None,
            trans_kp: None,
            trans_ki: None,
            trans_kd: None,
            period: None,
            trans_model_gain: None,
            trans_model_time_constant: None,
            rot_kp: None,
            rot_ki: None,
            rot_kd: None,
            rot_model_gain: None,
            rot_model_time_constant: None,
            estimator_correction_weight: None,
            wheel_interval: None,
            estimator_trans_velocity_alpha: None,
            cost_fn: None,
            wall_width: None,
            ignore_radius_from_pillar: None,
            ignore_length_from_wall: None,
            kx: None,
            kdx: None,
            ky: None,
            kdy: None,
            valid_control_lower_bound: None,
            fail_safe_distance: None,
            low_zeta: None,
            low_b: None,
            run_slalom_velocity: None,
            max_velocity: None,
            max_acceleration: None,
            max_jerk: None,
            slalom_angular_velocity_ref: None,
            slalom_angular_acceleration_ref: None,
            slalom_angular_jerk_ref: None,
            slalom_parameters_map: None,
            search_velocity: None,
            spin_angular_velocity: None,
            spin_angular_acceleration: None,
            spin_angular_jerk: None,
        }
    }

    /// Builds [Config](Config).
    ///
    /// This method can be failed when required parameters are not given.
    pub fn build(&mut self) -> BuilderResult<Config<'a, Node, Route, Pattern, Cost>> {
        macro_rules! get {
            ($field_name: ident) => {{
                ok_or(self.$field_name.take(), core::stringify!($field_name))?
            }};
        }

        use crate::pose_converter::{
            DEFAULT_IGNORE_LENGTH, DEFAULT_IGNORE_RADIUS, DEFAULT_SQUARE_WIDTH, DEFAULT_WALL_WIDTH,
        };
        use crate::trajectory_generators::{
            slalom_parameters_map2, DEFAULT_ANGULAR_ACCELERATION_REF, DEFAULT_ANGULAR_JERK_REF,
            DEFAULT_ANGULAR_VELOCITY_REF,
        };

        Ok(Config {
            square_width: self.square_width.unwrap_or(DEFAULT_SQUARE_WIDTH),
            front_offset: self.front_offset.unwrap_or(Default::default()),
            start: get!(start),
            return_goal: get!(return_goal),
            goals: get!(goals),
            search_initial_route: get!(search_initial_route),
            search_final_route: get!(search_final_route),
            trans_kp: get!(trans_kp),
            trans_ki: get!(trans_ki),
            trans_kd: get!(trans_kd),
            period: get!(period),
            trans_model_gain: get!(trans_model_gain),
            trans_model_time_constant: get!(trans_model_time_constant),
            rot_kp: get!(rot_kp),
            rot_ki: get!(rot_ki),
            rot_kd: get!(rot_kd),
            rot_model_gain: get!(rot_model_gain),
            rot_model_time_constant: get!(rot_model_time_constant),
            estimator_correction_weight: self
                .estimator_correction_weight
                .unwrap_or(Default::default()),
            wheel_interval: self.wheel_interval,
            estimator_trans_velocity_alpha: get!(estimator_trans_velocity_alpha),
            cost_fn: get!(cost_fn),
            wall_width: self.wall_width.unwrap_or(DEFAULT_WALL_WIDTH),
            ignore_radius_from_pillar: self
                .ignore_radius_from_pillar
                .unwrap_or(DEFAULT_IGNORE_RADIUS),
            ignore_length_from_wall: self
                .ignore_length_from_wall
                .unwrap_or(DEFAULT_IGNORE_LENGTH),
            kx: get!(kx),
            kdx: get!(kdx),
            ky: get!(ky),
            kdy: get!(kdy),
            valid_control_lower_bound: get!(valid_control_lower_bound),
            fail_safe_distance: get!(fail_safe_distance),
            low_zeta: get!(low_zeta),
            low_b: get!(low_b),
            run_slalom_velocity: get!(run_slalom_velocity),
            max_velocity: get!(max_velocity),
            max_acceleration: get!(max_acceleration),
            max_jerk: get!(max_jerk),
            slalom_angular_velocity_ref: self
                .slalom_angular_velocity_ref
                .unwrap_or(DEFAULT_ANGULAR_VELOCITY_REF),
            slalom_angular_acceleration_ref: self
                .slalom_angular_acceleration_ref
                .unwrap_or(DEFAULT_ANGULAR_ACCELERATION_REF),
            slalom_angular_jerk_ref: self
                .slalom_angular_jerk_ref
                .unwrap_or(DEFAULT_ANGULAR_JERK_REF),
            slalom_parameters_map: self.slalom_parameters_map.unwrap_or(slalom_parameters_map2),
            search_velocity: get!(search_velocity),
            spin_angular_velocity: get!(spin_angular_velocity),
            spin_angular_acceleration: get!(spin_angular_acceleration),
            spin_angular_jerk: get!(spin_angular_jerk),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder() {
        use typenum::*;
        use uom::si::{
            acceleration::meter_per_second_squared,
            angular_acceleration::degree_per_second_squared, angular_jerk::degree_per_second_cubed,
            angular_velocity::degree_per_second, jerk::meter_per_second_cubed, length::meter,
            time::second, velocity::meter_per_second,
        };

        use crate::nodes::RunNode;
        use crate::types::data::{AbsoluteDirection, Pattern, SearchKind};

        use AbsoluteDirection::*;

        type Size = U4;

        let start = RunNode::<Size>::new(0, 0, North).unwrap();
        let return_goal = RunNode::<Size>::new(0, 0, South).unwrap();
        let goals = vec![
            RunNode::<Size>::new(2, 0, South).unwrap(),
            RunNode::<Size>::new(2, 0, West).unwrap(),
        ];

        fn cost(_pattern: Pattern) -> u16 {
            unreachable!()
        }

        let _ = ConfigBuilder::new()
            .start(start)
            .return_goal(return_goal)
            .goals(&goals)
            .search_initial_route(SearchKind::Init)
            .search_final_route(SearchKind::Final)
            .cost_fn(cost)
            .estimator_trans_velocity_alpha(0.1)
            .period(Time::new::<second>(0.001))
            .estimator_correction_weight(0.1)
            .translational_kp(0.9)
            .translational_ki(0.05)
            .translational_kd(0.01)
            .translational_model_gain(1.0)
            .translational_model_time_constant(Time::new::<second>(0.001))
            .rotational_kp(0.2)
            .rotational_ki(0.2)
            .rotational_kd(0.0)
            .rotational_model_gain(1.0)
            .rotational_model_time_constant(Time::new::<second>(0.001))
            .tracker_kx(15.0)
            .tracker_kdx(4.0)
            .tracker_ky(15.0)
            .tracker_kdy(4.0)
            .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.03))
            .low_zeta(1.0)
            .low_b(1e-3)
            .fail_safe_distance(uom::si::f32::Length::new::<meter>(0.05))
            .search_velocity(Velocity::new::<meter_per_second>(0.12))
            .max_velocity(Velocity::new::<meter_per_second>(2.0))
            .max_acceleration(Acceleration::new::<meter_per_second_squared>(0.7))
            .max_jerk(Jerk::new::<meter_per_second_cubed>(1.0))
            .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(180.0))
            .spin_angular_acceleration(AngularAcceleration::new::<degree_per_second_squared>(
                1800.0,
            ))
            .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(7200.0))
            .run_slalom_velocity(Velocity::new::<meter_per_second>(1.0))
            .build()
            .unwrap();
    }
}
