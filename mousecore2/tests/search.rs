use mousecore2::{
    control::{ControlParameters, Controller, Target, Tracker},
    estimate::{AngleState, Estimator, LengthState, SensorValue, State},
    solve::search::{
        Commander, Coordinate, Posture, SearchState, Searcher, TrajectoryKind, WallState,
    },
    trajectory::{
        slalom::{SlalomConfig, SlalomDirection, SlalomGenerator, SlalomKind},
        spin::SpinGenerator,
        straight::StraightGenerator,
        ShiftTrajectory, StopTrajectory,
    },
    wall::{Pose, WallDetector, Walls},
};
use mousesim2::Simulator;
use uom::si::f32::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, ElectricPotential,
    Jerk, Length, Time, Velocity,
};
use uom::si::{
    acceleration::meter_per_second_squared, angle::degree,
    angular_acceleration::degree_per_second_squared, angular_jerk::degree_per_second_cubed,
    angular_velocity::degree_per_second, electric_potential::volt, jerk::meter_per_second_cubed,
    length::millimeter, time::second, velocity::meter_per_second,
};

#[test]
fn test_search1() {
    test_search::<16>(
        include_str!("../mazes/maze16_1.dat"),
        &[(15, 14), (14, 15)],
        Default::default(),
    );
}

#[test]
fn test_search2() {
    test_search::<16>(
        include_str!("../mazes/maze16_2.dat"),
        &[(15, 14), (14, 15)],
        Default::default(),
    );
}

#[test]
fn test_search3() {
    test_search::<16>(
        include_str!("../mazes/maze16_3.dat"),
        &[(15, 14), (14, 15)],
        Default::default(),
    );
}

#[test]
fn test_search4() {
    test_search::<32>(
        include_str!("../mazes/maze32_1.dat"),
        &[(37, 28), (36, 29)],
        Default::default(),
    );
}

#[test]
fn test_search5() {
    test_search::<32>(
        include_str!("../mazes/maze32_2.dat"),
        &[(15, 14), (14, 15)],
        Default::default(),
    );
}

#[test]
fn test_search_offset1() {
    test_search::<32>(
        include_str!("../mazes/maze32_1.dat"),
        &[(37, 28), (36, 29)],
        Length::new::<millimeter>(10.0),
    );
}

#[test]
fn test_search_offset2() {
    test_search::<32>(
        include_str!("../mazes/maze32_2.dat"),
        &[(15, 14), (14, 15)],
        Length::new::<millimeter>(10.0),
    );
}

fn test_search<const W: u8>(input: &'static str, goals: &[(u8, u8)], front_offset: Length) {
    let goals = goals
        .into_iter()
        .map(|&(x, y)| Coordinate::new(x, y).unwrap())
        .collect::<Vec<_>>();
    // common settings
    let period = Time::new::<second>(0.001);
    let trans_k = 1.865;
    let trans_t1 = Time::new::<second>(0.4443);
    let rot_k = 82.39;
    let rot_t1 = Time::new::<second>(0.2855);
    let state = State {
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
    let sensor_poses = vec![
        Pose {
            x: Length::new::<millimeter>(23.0),
            y: Length::new::<millimeter>(0.0),
            theta: Angle::new::<degree>(0.0),
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

    let mut state = state;
    let mut walls = Walls::<W>::new();
    let mut tracker = Tracker::builder()
        .period(period)
        .gain(40.0)
        .dgain(4.0)
        .zeta(1.0)
        .b(1.0)
        .xi_threshold(Velocity::new::<meter_per_second>(0.2))
        .build();
    let mut controller = Controller::builder()
        .trans_params(ControlParameters {
            kp: 4.8497,
            ki: 29.5783,
            kd: 0.0,
            model_k: trans_k,
            model_t1: trans_t1.value,
        })
        .rot_params(ControlParameters {
            kp: 0.21134,
            ki: 2.9317,
            kd: 0.0,
            model_k: rot_k,
            model_t1: rot_t1.value,
        })
        .period(period)
        .build();
    let mut estimator = Estimator::builder().period(period).build();
    let mut detector = WallDetector::<W>::default();
    let searcher = Searcher::<W>::new(Coordinate::new(0, 1).unwrap(), &goals);

    let square_width = Length::new::<millimeter>(90.0);

    let (init, front, right, left, back) = {
        let v_max = Velocity::new::<meter_per_second>(0.3);
        let a_max = Acceleration::new::<meter_per_second_squared>(10.0);
        let j_max = Jerk::new::<meter_per_second_cubed>(100.0);

        let slalom_config = SlalomConfig::new(square_width, front_offset);
        let slalom = SlalomGenerator::new(period, v_max, a_max, j_max);
        let spin = SpinGenerator::new(
            AngularVelocity::new::<degree_per_second>(1440.0),
            AngularAcceleration::new::<degree_per_second_squared>(14400.0),
            AngularJerk::new::<degree_per_second_cubed>(28800.0),
            period,
        );
        let straight = StraightGenerator::new(v_max, a_max, j_max, period);
        (
            straight.generate(square_width / 2.0 + front_offset, Default::default(), v_max),
            straight.generate(square_width, v_max, v_max),
            slalom.generate_constant_slalom(
                slalom_config.parameters(SlalomKind::Search90, SlalomDirection::Right),
                v_max,
            ),
            slalom.generate_constant_slalom(
                slalom_config.parameters(SlalomKind::Search90, SlalomDirection::Left),
                v_max,
            ),
            straight
                .generate(square_width / 2.0 - front_offset, v_max, Default::default())
                .chain(StopTrajectory::new(
                    Pose {
                        x: square_width / 2.0 - front_offset,
                        ..Default::default()
                    },
                    period,
                    Time::new::<second>(0.1),
                ))
                .chain(ShiftTrajectory::new(
                    Pose {
                        x: square_width / 2.0 - front_offset,
                        ..Default::default()
                    },
                    spin.generate(Angle::new::<degree>(180.0)),
                ))
                .chain(StopTrajectory::new(
                    Pose {
                        x: square_width / 2.0 - front_offset,
                        y: Default::default(),
                        theta: Angle::new::<degree>(180.0),
                    },
                    period,
                    Time::new::<second>(0.1),
                ))
                .chain(ShiftTrajectory::new(
                    Pose {
                        x: square_width / 2.0 - front_offset,
                        y: Default::default(),
                        theta: Angle::new::<degree>(180.0),
                    },
                    straight.generate(square_width / 2.0 + front_offset, Default::default(), v_max),
                )),
        )
    };

    let mut trajectory: Box<dyn Iterator<Item = Target>> = Box::new(ShiftTrajectory::new(
        Pose {
            x: state.x.x,
            y: state.y.x,
            theta: state.theta.x,
        },
        init,
    ));
    let mut next: Option<Box<dyn Iterator<Item = Target>>> = None;
    let mut commander = None::<Commander<W>>;
    let mut robot = SearchState::new(Coordinate::<W>::new(0, 1).unwrap(), Posture::North).unwrap();
    let mut simulator = Simulator::<W>::builder()
        .period(period)
        .trans_k(trans_k)
        .trans_t1(trans_t1)
        .rot_k(rot_k)
        .rot_t1(rot_t1)
        .walls(input)
        .wheel_interval(Length::new::<millimeter>(33.5))
        .current(state.clone())
        .last(state.clone())
        .max_voltage(ElectricPotential::new::<volt>(3.7))
        .build();

    loop {
        // estimate
        let sensor_value = {
            let distance = simulator.distance();
            SensorValue {
                left_distance: distance.left,
                right_distance: distance.right,
                translational_acceleration: simulator.translational_acceleration(),
                angular_velocity: simulator.angular_velocity(),
            }
        };
        estimator.estimate(&mut state, &sensor_value);

        // track
        let target = if let Some(target) = trajectory.next() {
            target
        } else if let Some(next) = next.take() {
            trajectory = next;
            trajectory.next().unwrap()
        } else {
            unreachable!()
        };
        let (control_target, control_state) = tracker.track(&state, &target);
        let vol = controller.control(&control_target, &control_state);
        simulator.apply(&vol);

        // search
        match commander
            .as_ref()
            .map(|commander| commander.next_coordinate(|coord| walls.wall_state(coord)))
        {
            Some(Ok(Some(next_coord))) => {
                let pose = Pose::from_search_state::<W>(robot, square_width, front_offset);
                let dir = robot.update(&next_coord).unwrap();
                macro_rules! gen {
                    ($traj: ident) => {
                        Box::new(ShiftTrajectory::new(pose, $traj.clone()))
                    };
                }
                next = Some(match dir {
                    TrajectoryKind::Front => gen!(front),
                    TrajectoryKind::Right => gen!(right),
                    TrajectoryKind::Left => gen!(left),
                    TrajectoryKind::Back => gen!(back),
                });
                commander.take();
            }
            Some(Ok(None)) => (),
            Some(Err(err)) => unreachable!("{:?}", err),
            None if next.is_none() => {
                match searcher.search(&robot.coordinate(), |coord| walls.wall_state(coord)) {
                    Ok(Some(next)) => commander = Some(next),
                    Ok(None) => break,
                    Err(err) => unreachable!("{:?}", err),
                }
            }
            _ => (),
        }

        // detect wall
        for pose in &sensor_poses {
            let cos_th = state.theta.x.value.cos();
            let sin_th = state.theta.x.value.sin();
            let pose = Pose {
                x: state.x.x + pose.x * cos_th - pose.y * sin_th,
                y: state.y.x + pose.x * sin_th + pose.y * cos_th,
                theta: state.theta.x + pose.theta,
            };

            if let Some(distance) = simulator.distance_from_wall(&pose) {
                if let Some((coord, wall_state)) =
                    detector.detect_and_update(&distance.mean, &distance.stddev, &pose)
                {
                    walls.update(&coord, &wall_state);
                }
            }
        }

        simulator.step();
    }

    assert_eq!(
        searcher.shortest_path(|coord| !matches!(
            walls.wall_state(coord),
            WallState::Checked { exists: false }
        )),
        searcher.shortest_path(|coord| !matches!(
            simulator.walls().wall_state(coord),
            WallState::Checked { exists: false }
        ))
    );
}
