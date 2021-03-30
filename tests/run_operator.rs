extern crate alloc;

#[macro_use]
extern crate typenum;

use core::f32::consts::PI;

use components::{
    agents::TrackingAgent,
    command_converter::CommandConverter,
    commanders::RunCommander,
    controllers::{RotationControllerBuilder, TranslationControllerBuilder},
    estimator::EstimatorBuilder,
    mazes::CheckedMaze,
    nodes::{RunNode, SearchNode},
    obstacle_detector::ObstacleDetector,
    operators::TrackingOperator,
    prelude::*,
    robot::Robot,
    tracker::TrackerBuilder,
    trajectory_generators::{slalom_parameters_map, RunTrajectoryGeneratorBuilder},
    trajectory_managers::TrackingTrajectoryManager,
    types::data::{AbsoluteDirection, AngleState, LengthState, Pattern, Pose, RobotState},
    utils::probability::Probability,
    wall_detector::WallDetectorBuilder,
    wall_manager::WallManager,
};
use typenum::consts::*;
use uom::si::f32::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Frequency, Jerk,
    Length, Time, Velocity,
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    angular_acceleration::radian_per_second_squared,
    angular_jerk::radian_per_second_cubed,
    angular_velocity::radian_per_second,
    frequency::hertz,
    jerk::meter_per_second_cubed,
    length::{meter, millimeter},
    time::{millisecond, second},
    velocity::meter_per_second,
};
use utils::math::MathFake;
use utils::sensors::AgentSimulator;

fn cost(pattern: Pattern) -> u16 {
    use Pattern::*;

    match pattern {
        Straight(x) => 10 * x,
        StraightDiagonal(x) => 7 * x,
        Search90 => 8,
        FastRun45 => 12,
        FastRun90 => 15,
        FastRun135 => 20,
        FastRun180 => 25,
        FastRunDiagonal90 => 15,
        SpinBack => 15,
    }
}

#[test]
fn test_run_operator() {
    type Size = U4;
    type MaxPathLength = op!(Size * Size);

    let start_state = RobotState {
        x: LengthState {
            x: Length::new::<millimeter>(45.0),
            ..Default::default()
        },
        y: LengthState {
            x: Length::new::<millimeter>(45.0),
            ..Default::default()
        },
        theta: AngleState {
            x: Angle::new::<degree>(90.0),
            ..Default::default()
        },
    };
    let period = Time::new::<millisecond>(1.0);
    let trans_model_gain = 1.0;
    let trans_model_time_constant = Time::new::<second>(0.3694);
    let rot_model_gain = 10.0;
    let rot_model_time_constant = Time::new::<second>(0.1499);
    let distance_sensors_poses = vec![
        Pose {
            x: Length::new::<millimeter>(0.0),
            y: Length::new::<millimeter>(23.0),
            theta: Angle::new::<degree>(0.0),
        },
        Pose {
            x: Length::new::<millimeter>(8.0),
            y: Length::new::<millimeter>(20.0),
            theta: Angle::new::<degree>(-45.0),
        },
        Pose {
            x: Length::new::<millimeter>(-8.0),
            y: Length::new::<millimeter>(20.0),
            theta: Angle::new::<degree>(45.0),
        },
        Pose {
            x: Length::new::<millimeter>(-11.5),
            y: Length::new::<millimeter>(13.0),
            theta: Angle::new::<degree>(90.0),
        },
        Pose {
            x: Length::new::<millimeter>(11.5),
            y: Length::new::<millimeter>(13.0),
            theta: Angle::new::<degree>(-90.0),
        },
    ];
    let existence_threshold = Probability::new(0.1).unwrap();
    let wheel_interval = Length::new::<millimeter>(33.5);

    let input_str = "+---+---+---+---+
|               |
+   +---+---+   +
|   |       |   |
+   +   +   +   +
|   |   |       |
+   +   +---+   +
|   |       |   |
+---+---+---+---+";

    let wall_storage = WallManager::<Size>::with_str(existence_threshold, input_str);

    let simulator = AgentSimulator::new(
        start_state.clone(),
        period,
        trans_model_gain,
        trans_model_time_constant,
        rot_model_gain,
        rot_model_time_constant,
        WallManager::<Size>::with_str(existence_threshold, input_str),
        distance_sensors_poses,
    );

    let (
        stepper,
        _observer,
        right_encoder,
        left_encoder,
        imu,
        right_motor,
        left_motor,
        distance_sensors,
    ) = simulator.split(wheel_interval);

    let agent = {
        let robot = {
            let estimator = {
                EstimatorBuilder::default()
                    .left_encoder(left_encoder)
                    .right_encoder(right_encoder)
                    .imu(imu)
                    .period(period)
                    .cut_off_frequency(Frequency::new::<hertz>(50.0))
                    .initial_state(start_state)
                    .wheel_interval(wheel_interval)
                    .build()
                    .unwrap()
            };

            let tracker = {
                let trans_controller = TranslationControllerBuilder::new()
                    .kp(0.9)
                    .ki(0.05)
                    .kd(0.01)
                    .period(period)
                    .model_gain(trans_model_gain)
                    .model_time_constant(trans_model_time_constant)
                    .build();

                let rot_controller = RotationControllerBuilder::new()
                    .kp(0.2)
                    .ki(0.2)
                    .kd(0.0)
                    .period(period)
                    .model_gain(rot_model_gain)
                    .model_time_constant(rot_model_time_constant)
                    .build();

                TrackerBuilder::default()
                    .right_motor(right_motor)
                    .left_motor(left_motor)
                    .period(period)
                    .kx(40.0)
                    .kdx(4.0)
                    .ky(40.0)
                    .kdy(4.0)
                    .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.03))
                    .translation_controller(trans_controller)
                    .rotation_controller(rot_controller)
                    .low_zeta(1.0)
                    .low_b(1e-3)
                    .fail_safe_distance(Length::new::<meter>(0.05))
                    .build()
                    .unwrap()
            };

            let wall_detector = {
                let obstacle_detector = ObstacleDetector::<_, MathFake>::new(distance_sensors);
                WallDetectorBuilder::new()
                    .wall_manager(&wall_storage)
                    .obstacle_detector(obstacle_detector)
                    .build::<MathFake, _>()
                    .unwrap()
            };
            Robot::new(estimator, tracker, wall_detector)
        };
        let trajectory_manager = {
            let trajectory_generator = RunTrajectoryGeneratorBuilder::default()
                .period(period)
                .max_velocity(Velocity::new::<meter_per_second>(2.0))
                .max_acceleration(Acceleration::new::<meter_per_second_squared>(0.7))
                .max_jerk(Jerk::new::<meter_per_second_cubed>(1.0))
                .slalom_parameters_map(slalom_parameters_map)
                .angular_velocity_ref(AngularVelocity::new::<radian_per_second>(3.0 * PI))
                .angular_acceleration_ref(AngularAcceleration::new::<radian_per_second_squared>(
                    36.0 * PI,
                ))
                .angular_jerk_ref(AngularJerk::new::<radian_per_second_cubed>(1200.0 * PI))
                .run_slalom_velocity(Velocity::new::<meter_per_second>(1.0))
                .build::<MaxPathLength>()
                .expect("Should never panic");
            TrackingTrajectoryManager::new(trajectory_generator, CommandConverter::default())
        };
        TrackingAgent::new(trajectory_manager, robot)
    };

    use AbsoluteDirection::*;

    let commander = {
        let maze = CheckedMaze::<_, _, _, SearchNode<Size>>::new(&wall_storage, cost);
        let start = RunNode::<Size>::new(0, 0, North).unwrap();
        let goals = vec![
            RunNode::<Size>::new(2, 0, South).unwrap(),
            RunNode::<Size>::new(2, 0, West).unwrap(),
        ];
        RunCommander::new(start, &goals, maze)
    };

    let operator = TrackingOperator::new(commander, agent);
    while operator.run().is_err() {
        stepper.step();
        assert!(!operator.tick().is_err());
    }
}
