//! An implementation of config.

use heapless::Vec;
use serde::{Deserialize, Serialize};
use uom::si::f32::{
    Acceleration, AngularAcceleration, AngularJerk, AngularVelocity, Frequency, Jerk, Length, Time,
    Velocity,
};

use crate::commanders::GOAL_SIZE_UPPER_BOUND;
use crate::controllers::ControlParameters;
use crate::impl_setter;
use crate::impl_with_getter;
use crate::nodes::RunNode;
use crate::pattern_converters::LinearPatternConverter;
use crate::types::configs::*;
use crate::types::data::SearchKind;
use crate::utils::builder::{ok_or, BuilderResult};

fn default_square_width() -> Length {
    crate::wall_detector::DEFAULT_SQUARE_WIDTH
}

fn default_wall_width() -> Length {
    crate::wall_detector::DEFAULT_WALL_WIDTH
}

fn default_ignore_radius_from_pillar() -> Length {
    crate::wall_detector::DEFAULT_IGNORE_RADIUS
}

fn default_ignore_length_from_wall() -> Length {
    crate::wall_detector::DEFAULT_IGNORE_LENGTH
}

impl_with_getter! {
    /// An implementation of config.
    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Config<const N: usize> {
        #[serde(default = "default_square_width")]
        square_width: Length,
        #[serde(default)]
        front_offset: Length,
        start: RunNode<N>,
        return_goal: RunNode<N>,
        goals: Vec<RunNode<N>, GOAL_SIZE_UPPER_BOUND>,
        search_initial_route: SearchKind,
        search_final_route: SearchKind,
        translational_parameters: ControlParameters,
        rotational_parameters: ControlParameters,
        period: Time,
        #[serde(default)]
        estimator_correction_weight: f32,
        #[serde(default)]
        wheel_interval: Option<Length>,
        estimator_cut_off_frequency: Frequency,
        #[serde(default)]
        pattern_converter: LinearPatternConverter<u16>,
        #[serde(default = "default_wall_width")]
        wall_width: Length,
        #[serde(default = "default_ignore_radius_from_pillar")]
        ignore_radius_from_pillar: Length,
        #[serde(default = "default_ignore_length_from_wall")]
        ignore_length_from_wall: Length,
        tracker_gain: f32,
        tracker_dgain: f32,
        valid_control_lower_bound: Velocity,
        fail_safe_distance: Length,
        low_zeta: f32,
        low_b: f32,
        run_slalom_velocity: Velocity,
        max_velocity: Velocity,
        max_acceleration: Acceleration,
        max_jerk: Jerk,
        search_velocity: Velocity,
        spin_angular_velocity: AngularVelocity,
        spin_angular_acceleration: AngularAcceleration,
        spin_angular_jerk: AngularJerk,
        slip_angle_const: Acceleration,
    }
}

macro_rules! impl_with_as_ref {
    (
        $(#[$meta:meta])*
        pub struct $name : ident <const N: usize> {
            $($field_name: ident : $type: ty,)*
        }
    ) => {
        $(#[$meta])*
        pub struct $name<const N: usize> {
            $($field_name: $type,)*
        }

        $(
            impl<const N: usize> AsRef<$type> for $name<N> {
                fn as_ref(&self) -> &$type {
                    &self.$field_name
                }
            }
        )*
    }
}

impl_with_as_ref! {
    pub struct ConfigContainer<const N: usize> {
        command_converter: CommandConverterConfig,
        search_commander: SearchCommanderConfig<RunNode<N>, SearchKind>,
        run_commander: RunCommanderConfig<RunNode<N>>,
        run_setup_commander: RunSetupCommanderConfig<RunNode<N>>,
        return_setup_commander: ReturnSetupCommanderConfig<RunNode<N>>,
        return_commander: ReturnCommanderConfig<RunNode<N>>,
        controller: MultiSisoControllerConfig,
        estimator: EstimatorConfig,
        tracker: TrackerConfig,
        search_trajectory_generator: SearchTrajectoryGeneratorConfig,
        run_trajectory_generator: RunTrajectoryGeneratorConfig,
        return_setup_generator: ReturnSetupTrajectoryGeneratorConfig,
        wall_detector: WallDetectorConfig,
        pattern_converter: LinearPatternConverter<u16>,
    }
}

impl<const N: usize> Into<ConfigContainer<N>> for Config<N> {
    fn into(self) -> ConfigContainer<N> {
        let Config {
            square_width,
            front_offset,
            start,
            return_goal,
            goals,
            search_initial_route,
            search_final_route,
            period,
            translational_parameters,
            rotational_parameters,
            estimator_correction_weight,
            wheel_interval,
            estimator_cut_off_frequency,
            pattern_converter,
            wall_width,
            ignore_radius_from_pillar,
            ignore_length_from_wall,
            tracker_gain,
            tracker_dgain,
            valid_control_lower_bound,
            fail_safe_distance,
            low_zeta,
            low_b,
            run_slalom_velocity,
            max_velocity,
            max_acceleration,
            max_jerk,
            search_velocity,
            spin_angular_velocity,
            spin_angular_acceleration,
            spin_angular_jerk,
            slip_angle_const,
        } = self;
        ConfigContainer {
            command_converter: CommandConverterConfig {
                square_width,
                front_offset,
            },
            search_commander: SearchCommanderConfig {
                start: start.clone(),
                goals: goals.clone(),
                initial_route: search_initial_route,
                final_route: search_final_route,
            },
            run_commander: RunCommanderConfig {
                goals: goals.clone(),
            },
            run_setup_commander: RunSetupCommanderConfig {
                goals: goals.clone(),
            },
            return_setup_commander: ReturnSetupCommanderConfig {
                return_goal: return_goal.clone(),
            },
            return_commander: ReturnCommanderConfig {
                return_goal: return_goal.clone(),
            },
            controller: MultiSisoControllerConfig {
                period,
                translational_parameters,
                rotational_parameters,
            },
            estimator: EstimatorConfig {
                period,
                cut_off_frequency: estimator_cut_off_frequency,
                wheel_interval,
                correction_weight: estimator_correction_weight,
                slip_angle_const,
            },
            tracker: TrackerConfig {
                gain: tracker_gain,
                dgain: tracker_dgain,
                period,
                valid_control_lower_bound,
                fail_safe_distance,
                low_zeta,
                low_b,
            },
            search_trajectory_generator: SearchTrajectoryGeneratorConfig {
                max_acceleration,
                max_jerk,
                period,
                search_velocity,
                front_offset,
                square_width,
                spin_angular_velocity,
                spin_angular_acceleration,
                spin_angular_jerk,
            },
            run_trajectory_generator: RunTrajectoryGeneratorConfig {
                run_slalom_velocity,
                max_velocity,
                max_acceleration,
                max_jerk,
                period,
                square_width,
                front_offset,
            },
            return_setup_generator: ReturnSetupTrajectoryGeneratorConfig {
                max_angular_velocity: spin_angular_velocity,
                max_angular_acceleration: spin_angular_acceleration,
                max_angular_jerk: spin_angular_jerk,
                period,
            },
            wall_detector: WallDetectorConfig {
                square_width,
                wall_width,
                ignore_radius_from_pillar,
                ignore_length_from_wall,
            },
            pattern_converter,
        }
    }
}

/// A builder for [Config](Config).
///
/// # Example
/// ```
/// use typenum::*;
/// use uom::si::f32::{
///     Length, Velocity, Acceleration, Jerk, AngularVelocity, AngularAcceleration, AngularJerk,
///     Time, Frequency,
/// };
/// use uom::si::{
///     acceleration::meter_per_second_squared,
///     angular_acceleration::degree_per_second_squared, angular_jerk::degree_per_second_cubed,
///     angular_velocity::degree_per_second, jerk::meter_per_second_cubed, length::meter,
///     time::second, velocity::meter_per_second, frequency::hertz,
/// };
/// use components::nodes::RunNode;
/// use components::types::data::{AbsoluteDirection, Pattern, SearchKind, ControlParameters};
/// use components::defaults::config::ConfigBuilder;
/// use components::pattern_converters::LinearPatternConverter;
///
/// use AbsoluteDirection::*;
///
/// let start = RunNode::<4>::new(0, 0, North).unwrap();
/// let return_goal = RunNode::<4>::new(0, 0, South).unwrap();
/// let goals = vec![
///     RunNode::<4>::new(2, 0, South).unwrap(),
///     RunNode::<4>::new(2, 0, West).unwrap(),
/// ];
///
/// let config = ConfigBuilder::new()
///     .start(start)
///     .return_goal(return_goal)
///     .goals(goals.into_iter().collect())
///     .search_initial_route(SearchKind::Init)
///     .search_final_route(SearchKind::Final)
///     .pattern_converter(LinearPatternConverter::default())
///     .estimator_cut_off_frequency(Frequency::new::<hertz>(50.0))
///     .period(Time::new::<second>(0.001))
///     .estimator_correction_weight(0.1)
///     .translational_parameters(ControlParameters {
///         kp: 0.9,
///         ki: 0.05,
///         kd: 0.01,
///         model_k: 1.0,
///         model_t1: 0.001,
///     })
///     .rotational_parameters(ControlParameters {
///         kp: 0.2,
///         ki: 0.2,
///         kd: 0.01,
///         model_k: 1.0,
///         model_t1: 0.001,
///     })
///     .tracker_gain(15.0)
///     .tracker_dgain(4.0)
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
///     .slip_angle_const(Acceleration::new::<meter_per_second_squared>(100.0))
///     .build()
///     .unwrap();
/// ```

pub struct ConfigBuilder<const N: usize> {
    square_width: Option<Length>,
    front_offset: Option<Length>,
    start: Option<RunNode<N>>,
    return_goal: Option<RunNode<N>>,
    goals: Option<Vec<RunNode<N>, GOAL_SIZE_UPPER_BOUND>>,
    search_initial_route: Option<SearchKind>,
    search_final_route: Option<SearchKind>,
    period: Option<Time>,
    translational_parameters: Option<ControlParameters>,
    rotational_parameters: Option<ControlParameters>,
    estimator_correction_weight: Option<f32>,
    wheel_interval: Option<Length>,
    estimator_cut_off_frequency: Option<Frequency>,
    pattern_converter: Option<LinearPatternConverter<u16>>,
    wall_width: Option<Length>,
    ignore_radius_from_pillar: Option<Length>,
    ignore_length_from_wall: Option<Length>,
    tracker_gain: Option<f32>,
    tracker_dgain: Option<f32>,
    valid_control_lower_bound: Option<Velocity>,
    fail_safe_distance: Option<Length>,
    low_zeta: Option<f32>,
    low_b: Option<f32>,
    run_slalom_velocity: Option<Velocity>,
    max_velocity: Option<Velocity>,
    max_acceleration: Option<Acceleration>,
    max_jerk: Option<Jerk>,
    search_velocity: Option<Velocity>,
    spin_angular_velocity: Option<AngularVelocity>,
    spin_angular_acceleration: Option<AngularAcceleration>,
    spin_angular_jerk: Option<AngularJerk>,
    slip_angle_const: Option<Acceleration>,
}

impl<const N: usize> ConfigBuilder<N> {
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
        start: RunNode<N>
    );
    impl_setter!(
        /// **Required**,
        /// Sets a goal for return to start.
        return_goal: RunNode<N>
    );
    impl_setter!(
        /// **Required**,
        /// Sets the goal nodes for search.
        goals: Vec<RunNode<N>, GOAL_SIZE_UPPER_BOUND>
    );
    impl_setter!(
        /// **Required**,
        /// Sets a route for initial trajectory.
        search_initial_route: SearchKind
    );
    impl_setter!(
        /// **Required**,
        /// Sets a route for final trajectory.
        search_final_route: SearchKind
    );
    impl_setter!(
        /// **Required**,
        /// Sets the period for control.
        period: Time
    );
    impl_setter!(
        /// **Required**,
        /// Sets the control parameters for translation.
        translational_parameters: ControlParameters
    );
    impl_setter!(
        /// **Required**,
        /// Sets the control parameters for translation.
        rotational_parameters: ControlParameters
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
        /// Sets a cut off frequency for low pass filter of translational velocity.
        estimator_cut_off_frequency: Frequency
    );
    impl_setter!(
        /// **Required**,
        /// Sets a pattern converter.
        pattern_converter: LinearPatternConverter<u16>
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
        /// Ignores the sensor value in the
        /// a given radius from pillar.
        ignore_radius_from_pillar: Length
    );
    impl_setter!(
        /// **Optional**,
        /// Default: 8 \[mm\].
        ///
        /// Sets the distance for ignoring the sensor value.
        /// Ignores the sensor value in the
        /// given distance from walls.
        ignore_length_from_wall: Length
    );
    impl_setter!(
        /// **Required**,
        /// Sets a control gain for tracking.
        tracker_gain: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets a control gain for tracking.
        tracker_dgain: f32
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
    impl_setter!(
        /// **Required**,
        /// Sets the constant value for estimating slip angle.
        slip_angle_const: Acceleration
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
            period: None,
            translational_parameters: None,
            rotational_parameters: None,
            estimator_correction_weight: None,
            wheel_interval: None,
            estimator_cut_off_frequency: None,
            pattern_converter: None,
            wall_width: None,
            ignore_radius_from_pillar: None,
            ignore_length_from_wall: None,
            tracker_gain: None,
            tracker_dgain: None,
            valid_control_lower_bound: None,
            fail_safe_distance: None,
            low_zeta: None,
            low_b: None,
            run_slalom_velocity: None,
            max_velocity: None,
            max_acceleration: None,
            max_jerk: None,
            search_velocity: None,
            spin_angular_velocity: None,
            spin_angular_acceleration: None,
            spin_angular_jerk: None,
            slip_angle_const: None,
        }
    }

    /// Builds [Config](Config).
    ///
    /// This method can be failed when required parameters are not given.
    pub fn build(&mut self) -> BuilderResult<Config<N>> {
        macro_rules! get {
            ($field_name: ident) => {{
                ok_or(self.$field_name.take(), core::stringify!($field_name))?
            }};
        }

        use crate::wall_detector::{
            DEFAULT_IGNORE_LENGTH, DEFAULT_IGNORE_RADIUS, DEFAULT_SQUARE_WIDTH, DEFAULT_WALL_WIDTH,
        };

        Ok(Config {
            square_width: self.square_width.unwrap_or(DEFAULT_SQUARE_WIDTH),
            front_offset: self.front_offset.unwrap_or(Default::default()),
            start: get!(start),
            return_goal: get!(return_goal),
            goals: get!(goals),
            search_initial_route: get!(search_initial_route),
            search_final_route: get!(search_final_route),
            period: get!(period),
            translational_parameters: get!(translational_parameters),
            rotational_parameters: get!(rotational_parameters),
            estimator_correction_weight: self
                .estimator_correction_weight
                .unwrap_or(Default::default()),
            wheel_interval: self.wheel_interval,
            estimator_cut_off_frequency: get!(estimator_cut_off_frequency),
            pattern_converter: self.pattern_converter.take().unwrap_or(Default::default()),
            wall_width: self.wall_width.unwrap_or(DEFAULT_WALL_WIDTH),
            ignore_radius_from_pillar: self
                .ignore_radius_from_pillar
                .unwrap_or(DEFAULT_IGNORE_RADIUS),
            ignore_length_from_wall: self
                .ignore_length_from_wall
                .unwrap_or(DEFAULT_IGNORE_LENGTH),
            tracker_gain: get!(tracker_gain),
            tracker_dgain: get!(tracker_dgain),
            valid_control_lower_bound: get!(valid_control_lower_bound),
            fail_safe_distance: get!(fail_safe_distance),
            low_zeta: get!(low_zeta),
            low_b: get!(low_b),
            run_slalom_velocity: get!(run_slalom_velocity),
            max_velocity: get!(max_velocity),
            max_acceleration: get!(max_acceleration),
            max_jerk: get!(max_jerk),
            search_velocity: get!(search_velocity),
            spin_angular_velocity: get!(spin_angular_velocity),
            spin_angular_acceleration: get!(spin_angular_acceleration),
            spin_angular_jerk: get!(spin_angular_jerk),
            slip_angle_const: get!(slip_angle_const),
        })
    }
}

#[cfg(test)]
pub use tests::default_config;

#[cfg(test)]
mod tests {
    use super::*;

    pub fn default_config() -> Config<4> {
        use uom::si::{
            acceleration::meter_per_second_squared,
            angular_acceleration::degree_per_second_squared, angular_jerk::degree_per_second_cubed,
            angular_velocity::degree_per_second, frequency::hertz, jerk::meter_per_second_cubed,
            length::meter, time::second, velocity::meter_per_second,
        };

        use crate::types::data::AbsoluteDirection;

        use AbsoluteDirection::*;

        let start = RunNode::<4>::new(0, 0, North).unwrap();
        let return_goal = RunNode::<4>::new(0, 0, South).unwrap();
        let goals = vec![
            RunNode::<4>::new(2, 0, South).unwrap(),
            RunNode::<4>::new(2, 0, West).unwrap(),
        ];

        ConfigBuilder::new()
            .start(start)
            .return_goal(return_goal)
            .goals(goals.into_iter().collect())
            .search_initial_route(SearchKind::Init)
            .search_final_route(SearchKind::Final)
            .pattern_converter(LinearPatternConverter::default())
            .estimator_cut_off_frequency(Frequency::new::<hertz>(50.0))
            .period(Time::new::<second>(0.001))
            .estimator_correction_weight(0.1)
            .translational_parameters(ControlParameters {
                kp: 0.9,
                ki: 0.05,
                kd: 0.01,
                model_k: 1.0,
                model_t1: 0.001,
            })
            .rotational_parameters(ControlParameters {
                kp: 0.2,
                ki: 0.2,
                kd: 0.01,
                model_k: 1.0,
                model_t1: 0.001,
            })
            .tracker_gain(15.0)
            .tracker_dgain(4.0)
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
            .slip_angle_const(Acceleration::new::<meter_per_second_squared>(100.0))
            .build()
            .unwrap()
    }

    #[test]
    fn test_builder() {
        default_config();
    }
}
