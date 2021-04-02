//! An implementation of config.

use heapless::Vec;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use uom::si::f32::{
    Acceleration, AngularAcceleration, AngularJerk, AngularVelocity, Frequency, Jerk, Length, Time,
    Velocity,
};

use crate::commanders::GoalSizeUpperBound;
use crate::impl_setter;
use crate::impl_with_getter;
use crate::nodes::RunNode;
use crate::pattern_converters::LinearPatternConverter;
use crate::types::data::SearchKind;
use crate::utils::builder::{ok_or, BuilderResult};

impl_with_getter! {
    /// An implementation of config.
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    #[derive(Debug, Clone, PartialEq)]
    pub struct Config<Size> {
        square_width: Length,
        front_offset: Length,
        start: RunNode<Size>,
        return_goal: RunNode<Size>,
        goals: Vec<RunNode<Size>,GoalSizeUpperBound>,
        search_initial_route: SearchKind,
        search_final_route: SearchKind,
        translational_kp: f32,
        translational_ki: f32,
        translational_kd: f32,
        period: Time,
        translational_model_gain: f32,
        translational_model_time_constant: Time,
        rotational_kp: f32,
        rotational_ki: f32,
        rotational_kd: f32,
        rotational_model_gain: f32,
        rotational_model_time_constant: Time,
        estimator_correction_weight: f32,
        wheel_interval: Option<Length>,
        estimator_cut_off_frequency: Frequency,
        pattern_converter: LinearPatternConverter<u16>,
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
        search_velocity: Velocity,
        spin_angular_velocity: AngularVelocity,
        spin_angular_acceleration: AngularAcceleration,
        spin_angular_jerk: AngularJerk,
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
/// use components::types::data::{AbsoluteDirection, Pattern, SearchKind};
/// use components::defaults::config::ConfigBuilder;
/// use components::pattern_converters::LinearPatternConverter;
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

pub struct ConfigBuilder<Size> {
    square_width: Option<Length>,
    front_offset: Option<Length>,
    start: Option<RunNode<Size>>,
    return_goal: Option<RunNode<Size>>,
    goals: Option<Vec<RunNode<Size>, GoalSizeUpperBound>>,
    search_initial_route: Option<SearchKind>,
    search_final_route: Option<SearchKind>,
    translational_kp: Option<f32>,
    translational_ki: Option<f32>,
    translational_kd: Option<f32>,
    period: Option<Time>,
    translational_model_gain: Option<f32>,
    translational_model_time_constant: Option<Time>,
    rotational_kp: Option<f32>,
    rotational_ki: Option<f32>,
    rotational_kd: Option<f32>,
    rotational_model_gain: Option<f32>,
    rotational_model_time_constant: Option<Time>,
    estimator_correction_weight: Option<f32>,
    wheel_interval: Option<Length>,
    estimator_cut_off_frequency: Option<Frequency>,
    pattern_converter: Option<LinearPatternConverter<u16>>,
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
    search_velocity: Option<Velocity>,
    spin_angular_velocity: Option<AngularVelocity>,
    spin_angular_acceleration: Option<AngularAcceleration>,
    spin_angular_jerk: Option<AngularJerk>,
}

impl<Size> ConfigBuilder<Size> {
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
        start: RunNode<Size>
    );
    impl_setter!(
        /// **Required**,
        /// Sets a goal for return to start.
        return_goal: RunNode<Size>
    );
    impl_setter!(
        /// **Required**,
        /// Sets the goal nodes for search.
        goals: Vec<RunNode<Size>, GoalSizeUpperBound>
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
        /// Sets the P gain for translation.
        translational_kp: translational_kp: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the I gain for translation.
        translational_ki: translational_ki: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the D gain for translation.
        translational_kd: translational_kd: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the model gain for translation.
        ///
        /// The model is used for the input of feed-forward.
        ///
        /// Assumes the model is the first-order delay system.
        translational_model_gain: translational_model_gain: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the model time constant for translation.
        ///
        /// The model is used for the input of feed-forward.
        ///
        /// Assumes the model is the first-order delay system.
        translational_model_time_constant: translational_model_time_constant: Time
    );
    impl_setter!(
        /// **Required**,
        /// Sets the P gain for rotation.
        rotational_kp: rotational_kp: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the I gain for rotation.
        rotational_ki: rotational_ki: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the D gain for rotation.
        rotational_kd: rotational_kd: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the model gain for rotation.
        ///
        /// The model is used for the input of feed-forward.
        ///
        /// Assumes the model is the first-order delay system.
        rotational_model_gain: rotational_model_gain: f32
    );
    impl_setter!(
        /// **Required**,
        /// Sets the model time constant for rotation.
        ///
        /// The model is used for the input of feed-forward.
        ///
        /// Assumes the model is the first-order delay system.
        rotational_model_time_constant: rotational_model_time_constant: Time
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
            translational_kp: None,
            translational_ki: None,
            translational_kd: None,
            period: None,
            translational_model_gain: None,
            translational_model_time_constant: None,
            rotational_kp: None,
            rotational_ki: None,
            rotational_kd: None,
            rotational_model_gain: None,
            rotational_model_time_constant: None,
            estimator_correction_weight: None,
            wheel_interval: None,
            estimator_cut_off_frequency: None,
            pattern_converter: None,
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
            search_velocity: None,
            spin_angular_velocity: None,
            spin_angular_acceleration: None,
            spin_angular_jerk: None,
        }
    }

    /// Builds [Config](Config).
    ///
    /// This method can be failed when required parameters are not given.
    pub fn build(&mut self) -> BuilderResult<Config<Size>> {
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
            translational_kp: get!(translational_kp),
            translational_ki: get!(translational_ki),
            translational_kd: get!(translational_kd),
            period: get!(period),
            translational_model_gain: get!(translational_model_gain),
            translational_model_time_constant: get!(translational_model_time_constant),
            rotational_kp: get!(rotational_kp),
            rotational_ki: get!(rotational_ki),
            rotational_kd: get!(rotational_kd),
            rotational_model_gain: get!(rotational_model_gain),
            rotational_model_time_constant: get!(rotational_model_time_constant),
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
            search_velocity: get!(search_velocity),
            spin_angular_velocity: get!(spin_angular_velocity),
            spin_angular_acceleration: get!(spin_angular_acceleration),
            spin_angular_jerk: get!(spin_angular_jerk),
        })
    }
}

#[cfg(test)]
pub use tests::default_config;

#[cfg(test)]
mod tests {
    use super::*;

    type Size = typenum::consts::U4;

    pub fn default_config() -> Config<Size> {
        use uom::si::{
            acceleration::meter_per_second_squared,
            angular_acceleration::degree_per_second_squared, angular_jerk::degree_per_second_cubed,
            angular_velocity::degree_per_second, frequency::hertz, jerk::meter_per_second_cubed,
            length::meter, time::second, velocity::meter_per_second,
        };

        use crate::types::data::AbsoluteDirection;

        use AbsoluteDirection::*;

        let start = RunNode::<Size>::new(0, 0, North).unwrap();
        let return_goal = RunNode::<Size>::new(0, 0, South).unwrap();
        let goals = vec![
            RunNode::<Size>::new(2, 0, South).unwrap(),
            RunNode::<Size>::new(2, 0, West).unwrap(),
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
            .unwrap()
    }

    #[test]
    fn test_builder() {
        default_config();
    }
}
