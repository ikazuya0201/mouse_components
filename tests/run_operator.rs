extern crate std;

use std::rc::Rc;

use components::{
    agents::TrackingAgent,
    command_converter::CommandConverter,
    commanders::RunCommander,
    controllers::MultiSisoController,
    estimator::EstimatorBuilder,
    mazes::CheckedMaze,
    nodes::{RunNode, SearchNode},
    operators::TrackingOperator,
    pattern_converters::DefaultPatternConverter,
    prelude::*,
    robot::Robot,
    tracker::TrackerBuilder,
    trajectory_generators::{DefaultSlalomParametersGenerator, RunTrajectoryGeneratorBuilder},
    trajectory_managers::TrackingTrajectoryManager,
    types::data::{
        AbsoluteDirection, AngleState, ControlParameters, LengthState, Pose, Position, RobotState,
    },
    utils::probability::Probability,
    utils::random::Random,
    wall_detector::WallDetectorBuilder,
    wall_manager::WallManager,
};
use uom::si::f32::{
    Acceleration, Angle, ElectricPotential, Frequency, Jerk, Length, Time, Velocity,
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    electric_potential::volt,
    frequency::hertz,
    jerk::meter_per_second_cubed,
    length::{meter, millimeter},
    time::{millisecond, second},
    velocity::meter_per_second,
};
use utils::sensors::AgentSimulator;

macro_rules! impl_run_operator_test {
    ($name: ident < $size: literal > { $input: expr, }) => {
        #[test]
        fn $name() {
            let (input_str, goal, square_width) = $input;

            let square_width = Length::new::<meter>(square_width);
            let wall_width = Length::new::<meter>(0.006);
            let ignore_radius_from_pillar = Length::new::<meter>(0.01);
            let ignore_length_from_wall = Length::new::<meter>(0.008);

            let start_state = RobotState {
                x: LengthState {
                    x: Random {
                        mean: square_width / 2.0,
                        standard_deviation: Default::default(),
                    },
                    ..Default::default()
                },
                y: LengthState {
                    x: Random {
                        mean: square_width / 2.0,
                        standard_deviation: Default::default(),
                    },
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
                    x: Length::new::<millimeter>(23.0),
                    y: Length::new::<millimeter>(0.0),
                    theta: Angle::new::<degree>(0.0),
                },
                Pose {
                    x: Length::new::<millimeter>(20.0),
                    y: Length::new::<millimeter>(-8.0),
                    theta: Angle::new::<degree>(-45.0),
                },
                Pose {
                    x: Length::new::<millimeter>(20.0),
                    y: Length::new::<millimeter>(8.0),
                    theta: Angle::new::<degree>(45.0),
                },
                Pose {
                    x: Length::new::<millimeter>(13.0),
                    y: Length::new::<millimeter>(11.5),
                    theta: Angle::new::<degree>(90.0),
                },
                Pose {
                    x: Length::new::<millimeter>(13.0),
                    y: Length::new::<millimeter>(-11.5),
                    theta: Angle::new::<degree>(-90.0),
                },
            ];
            let existence_threshold = Probability::new(0.1).unwrap();
            let wheel_interval = Length::new::<millimeter>(33.5);

            let wall_storage = Rc::new(WallManager::<$size>::with_str(
                existence_threshold,
                input_str,
            ));

            let simulator = AgentSimulator::new(
                start_state.clone(),
                period,
                trans_model_gain,
                trans_model_time_constant,
                rot_model_gain,
                rot_model_time_constant,
                WallManager::<$size>::with_str(existence_threshold, input_str),
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
            ) = simulator.split(
                wheel_interval,
                ElectricPotential::new::<volt>(3.7),
                square_width,
                wall_width,
                ignore_radius_from_pillar,
                ignore_length_from_wall,
            );

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
                            .slip_angle_const(Acceleration::new::<meter_per_second_squared>(100.0))
                            .build()
                            .unwrap()
                    };

                    let tracker = {
                        let controller = MultiSisoController::new(
                            left_motor,
                            right_motor,
                            ControlParameters {
                                kp: 4.8497,
                                ki: 29.5783,
                                kd: 0.0,
                                model_k: 1.865,
                                model_t1: 0.4443,
                            },
                            ControlParameters {
                                kp: 0.21134,
                                ki: 2.9317,
                                kd: 0.0,
                                model_k: 82.39,
                                model_t1: 0.2855,
                            },
                            period,
                            ElectricPotential::new::<volt>(10.0),
                        );

                        TrackerBuilder::new()
                            .period(period)
                            .gain(40.0)
                            .dgain(4.0)
                            .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.15))
                            .controller(controller)
                            .low_zeta(1.0)
                            .low_b(1e-3)
                            .build()
                            .unwrap()
                    };

                    let wall_detector = {
                        WallDetectorBuilder::new()
                            .wall_manager(Rc::clone(&wall_storage))
                            .distance_sensors(distance_sensors.into_iter().collect())
                            .build::<$size>()
                            .unwrap()
                    };
                    Robot::new(estimator, tracker, wall_detector)
                };
                let trajectory_manager = {
                    let trajectory_generator = RunTrajectoryGeneratorBuilder::default()
                        .period(period)
                        .max_velocity(Velocity::new::<meter_per_second>(1.0))
                        .max_acceleration(Acceleration::new::<meter_per_second_squared>(50.0))
                        .max_jerk(Jerk::new::<meter_per_second_cubed>(100.0))
                        .parameters_generator(DefaultSlalomParametersGenerator::new(
                            square_width,
                            Default::default(),
                        ))
                        .run_slalom_velocity(Velocity::new::<meter_per_second>(0.5))
                        .square_width(square_width)
                        .build()
                        .expect("Should never panic");
                    TrackingTrajectoryManager::new(
                        trajectory_generator,
                        CommandConverter::new(square_width, Default::default()),
                    )
                };
                TrackingAgent::new(trajectory_manager, robot)
            };

            use AbsoluteDirection::*;

            let commander = {
                let maze = CheckedMaze::<_, _, SearchNode<$size>>::new(
                    Rc::clone(&wall_storage),
                    DefaultPatternConverter::default(),
                );
                let start = RunNode::new(0, 0, North).unwrap();
                let goal = Position::new(goal.0, goal.1).unwrap();
                RunCommander::new(start, goal, maze)
            };

            let operator = TrackingOperator::new(commander, agent);

            while operator.run().is_err() {
                stepper.step();
                if let Err(err) = operator.tick() {
                    panic!("Should never panic: {:?}", err);
                }
            }
        }
    };
}

impl_run_operator_test! {
    test_run_operator1<4> {
        (
            include_str!("../mazes/maze1.dat"),
            (2, 0),
            0.09,
        ),
    }
}

impl_run_operator_test! {
    test_run_operator2<16> {
        (
            include_str!("../mazes/maze2.dat"),
            (14, 14),
            0.09,
        ),
    }
}

impl_run_operator_test! {
    test_run_operator3<4> {
        (
            include_str!("../mazes/maze3.dat"),
            (2, 0),
            0.09,
        ),
    }
}

impl_run_operator_test! {
    test_run_operator_with_accel_slalom<4> {
        (
            include_str!("../mazes/maze4.dat"),
            (2, 0),
            0.09,
        ),
    }
}

impl_run_operator_test! {
    test_run_operator_with_classic_size<4> {
        (
            include_str!("../mazes/maze1.dat"),
            (2, 0),
            0.18,
        ),
    }
}
