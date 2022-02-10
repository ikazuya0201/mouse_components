use mousecore2::{
    control::{ControlParameters, Controller, Target, Tracker},
    estimate::{AngleState, Estimator, LengthState, SensorValue, State},
    solve::{
        run::{shortest_path, AbsoluteDirection, EdgeKind, Node, TrajectoryKind},
        search::WallState,
    },
    trajectory::{
        slalom::{SlalomConfig, SlalomGenerator},
        straight::StraightGenerator,
        ShiftTrajectory,
    },
    wall::{Pose, Walls},
};
use mousesim2::Simulator;
use uom::si::f32::{Acceleration, Angle, ElectricPotential, Jerk, Length, Time, Velocity};
use uom::si::{
    acceleration::meter_per_second_squared, angle::degree, electric_potential::volt,
    jerk::meter_per_second_cubed, length::millimeter, time::second, velocity::meter_per_second,
};

use AbsoluteDirection::*;

#[test]
fn test_run1() {
    test_run::<16>(
        include_str!("../mazes/maze16_1.dat"),
        &[
            (14, 14, North),
            (14, 14, East),
            (14, 14, West),
            (14, 14, South),
        ],
    );
}

#[test]
fn test_run2() {
    test_run::<16>(
        include_str!("../mazes/maze16_3.dat"),
        &[
            (14, 14, North),
            (14, 14, East),
            (14, 14, West),
            (14, 14, South),
        ],
    );
}

fn test_run<const W: u8>(input: &'static str, goals: &[(u8, u8, AbsoluteDirection)]) {
    let goals = goals
        .into_iter()
        .map(|&(x, y, dir)| Node::new(x, y, dir).unwrap())
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

    let mut state = state;
    let walls = input.parse::<Walls<W>>().unwrap();
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

    let square_width = Length::new::<millimeter>(90.0);

    let v_max = Velocity::new::<meter_per_second>(0.3);
    let a_max = Acceleration::new::<meter_per_second_squared>(10.0);
    let j_max = Jerk::new::<meter_per_second_cubed>(100.0);

    let slalom_config = SlalomConfig::new(square_width, Default::default());
    let slalom = SlalomGenerator::new(period, v_max, a_max, j_max);
    let straight = StraightGenerator::new(v_max, a_max, j_max, period);

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

    let path = shortest_path(
        Node::new(0, 0, North).unwrap(),
        |node| goals.iter().any(|goal| node == goal),
        |coord| {
            matches!(
                walls.wall_state(coord),
                WallState::Checked { exists: true } | WallState::Unchecked
            )
        },
        |kind| match kind {
            EdgeKind::Straight(x) => *x as u16 * 10,
            EdgeKind::StraightDiagonal(x) => *x as u16 * 7,
            EdgeKind::Slalom45 => 12,
            EdgeKind::Slalom90 => 15,
            EdgeKind::Slalom135 => 20,
            EdgeKind::Slalom180 => 25,
            EdgeKind::SlalomDiagonal90 => 15,
        },
    )
    .unwrap();

    let mut trajectories = std::collections::VecDeque::new();
    let mut velocity = Default::default();
    let run_slalom_velocity = Velocity::new::<meter_per_second>(0.5);
    for i in 0..path.len() - 1 {
        use TrajectoryKind::*;

        let pose = Pose::from_node(path[i], square_width);
        let kind = path[i]
            .trajectory_kind(&path[i + 1])
            .unwrap_or_else(|| unreachable!("{:?}", (path[i], path[i + 1])));
        let (start, middle, end) = if i <= 1 {
            (velocity, run_slalom_velocity, run_slalom_velocity)
        } else if i + 2 == path.len() {
            (velocity, velocity * 0.666_666_7, Default::default())
        } else if i + 3 == path.len() {
            (velocity, velocity * 0.5, velocity * 0.5)
        } else {
            (
                run_slalom_velocity,
                run_slalom_velocity,
                run_slalom_velocity,
            )
        };
        macro_rules! gen {
            ($input: expr) => {{
                let (trajectory, terminal_velocity) = $input;
                (
                    Box::new(ShiftTrajectory::new(pose, trajectory)),
                    terminal_velocity,
                )
            }};
        }
        let (trajectory, terminal_velocity): (Box<dyn Iterator<Item = Target>>, _) = match kind {
            Straight(x) => {
                gen!(straight.generate_with_terminal_velocity(x as f32 * square_width, start, end))
            }
            StraightDiagonal(x) => gen!(straight.generate_with_terminal_velocity(
                x as f32 * square_width / 2.0f32.sqrt(),
                start,
                end
            )),
            Slalom(kind, dir) => {
                gen!(slalom.generate_slalom_with_terminal_velocity(
                    slalom_config.parameters(kind, dir),
                    start,
                    middle,
                    end
                ))
            }
        };
        velocity = terminal_velocity;
        trajectories.push_back(trajectory);
    }

    let vol_th = ElectricPotential::new::<volt>(10.0);

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
        let target = if let Some(trajectory) = trajectories.front_mut() {
            if let Some(target) = trajectory.next() {
                target
            } else {
                trajectories.pop_front();
                if let Some(trajectory) = trajectories.front_mut() {
                    trajectory.next().unwrap()
                } else {
                    break;
                }
            }
        } else {
            unreachable!()
        };
        let (control_target, control_state) = tracker.track(&state, &target);
        let vol = controller.control(&control_target, &control_state);
        assert!(vol.left <= vol_th && vol.right <= vol_th);
        simulator.apply(&vol);

        simulator.step();
    }
}
