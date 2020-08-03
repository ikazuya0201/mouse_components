use core::ops::{Div, Mul};

use quantities::{
    Acceleration, Angle, Distance, Jerk, Quantity, Speed, SquaredTime, Time, TimeDifferentiable,
};

use super::trajectory::{SubTarget, Target};
use crate::{dddt, ddt, dt};

pub struct StraightFunctionGenerator<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable,
    ddt!(T): TimeDifferentiable,
    dddt!(T): Quantity,
{
    v_max: dt!(T),
    a_max: ddt!(T),
    j_max: dddt!(T),
}

impl<T> StraightFunctionGenerator<T>
where
    T: TimeDifferentiable + Div<T, Output = f32>,
    dt!(T): TimeDifferentiable + Mul<Time, Output = T> + Div<dddt!(T), Output = SquaredTime>,
    ddt!(T): TimeDifferentiable + Mul<Time, Output = dt!(T)>,
    dddt!(T): Quantity + Mul<Time, Output = ddt!(T)>,
    f32: From<T> + From<dt!(T)> + From<ddt!(T)> + From<dddt!(T)>,
{
    const LOOP_COUNT: u8 = 25;

    pub fn new(v_max: dt!(T), a_max: ddt!(T), j_max: dddt!(T)) -> Self {
        Self {
            v_max,
            a_max,
            j_max,
        }
    }

    //NOTO: all given values should not be negative
    pub fn generate(
        &self,
        x_start: T,
        distance: T,
        v_start: dt!(T),
        v_end: dt!(T),
    ) -> (impl Fn(Time) -> SubTarget<T>, Time) {
        let v_reach = self.calculate_reachable_speed(v_start, v_end, distance);
        let v_end = if v_start < v_end {
            if v_reach < v_end {
                v_reach
            } else {
                v_end
            }
        } else if v_reach > v_end {
            v_reach
        } else {
            v_end
        };
        let mut low = if v_start < v_end { v_end } else { v_start };
        let mut high = self.v_max;
        for _ in 0..Self::LOOP_COUNT {
            let mid = (low + high) / 2.0;
            let d = self.calculate_acceleration_distance(v_start, mid)
                + self.calculate_acceleration_distance(mid, v_end);
            if d <= distance {
                low = mid;
            } else {
                high = mid;
            }
        }

        let v_max = low;
        let v_end = if v_end < v_max { v_end } else { v_max };

        let dist1 = self.calculate_acceleration_distance(v_start, v_max);
        let dist3 = self.calculate_acceleration_distance(v_max, v_end);
        let dist2 = distance - dist1 - dist3;

        let (accel_fn, dt1) = self.generate_acceleration(x_start, v_start, v_max);
        let (const_fn, dt2) = Self::generate_constant(x_start + dist1, dist2, v_max);
        let (decel_fn, dt3) = self.generate_acceleration(x_start + distance - dist3, v_max, v_end);

        let t1 = dt1;
        let t2 = t1 + dt2;
        let t3 = t2 + dt3;

        (
            move |t: Time| {
                if t < Default::default() {
                    SubTarget {
                        x: x_start,
                        v: v_start,
                        a: Default::default(),
                        j: Default::default(),
                    }
                } else if t <= t1 {
                    accel_fn(t)
                } else if t <= t2 {
                    const_fn(t - t1)
                } else if t <= t3 {
                    decel_fn(t - t2)
                } else {
                    SubTarget {
                        x: x_start + distance,
                        v: v_end,
                        a: Default::default(),
                        j: Default::default(),
                    }
                }
            },
            t3,
        )
    }

    fn generate_acceleration(
        &self,
        x_start: T,
        v_start: dt!(T),
        v_end: dt!(T),
    ) -> (impl Fn(Time) -> SubTarget<T>, Time) {
        let tc = self.a_max / self.j_max;
        let vd = self.a_max * tc;
        let (t1, t2, t3) = if (v_end - v_start).abs() >= vd {
            let tm = (v_end - v_start).abs() / self.a_max - tc;
            (tc, tc + tm, 2.0 * tc + tm)
        } else {
            let td = ((v_end - v_start).abs() / self.j_max).sqrt();
            (td, td, 2.0 * td)
        };

        let (a_m, j_m) = if v_start <= v_end {
            (self.a_max, self.j_max)
        } else {
            (-self.a_max, -self.j_max)
        };

        let distance = (v_start + v_end) * t3 / 2.0;
        let x_end = x_start + distance;

        (
            move |t: Time| -> SubTarget<T> {
                if t < Default::default() {
                    SubTarget {
                        x: x_start,
                        v: v_start,
                        a: Default::default(),
                        j: Default::default(),
                    }
                } else if t <= t1 {
                    SubTarget {
                        j: j_m,
                        a: j_m * t,
                        v: v_start + j_m * t * t / 2.0,
                        x: x_start + v_start * t + j_m * t * t * t / 6.0,
                    }
                } else if t <= t2 {
                    let dt1 = t - t1;
                    SubTarget {
                        j: Default::default(),
                        a: a_m,
                        v: v_start + a_m * t1 / 2.0 + a_m * dt1,
                        x: x_start
                            + v_start * t1
                            + j_m * t1 * t1 * t1 / 6.0
                            + (v_start + a_m * t1 / 2.0) * dt1
                            + a_m * dt1 * dt1 / 2.0,
                    }
                } else if t <= t3 {
                    let dt3 = t3 - t;
                    SubTarget {
                        j: -j_m,
                        a: j_m * dt3,
                        v: v_end - j_m * dt3 * dt3 / 2.0,
                        x: x_end - v_end * dt3 + j_m * dt3 * dt3 * dt3 / 6.0,
                    }
                } else {
                    SubTarget {
                        x: x_end,
                        v: v_end,
                        a: Default::default(),
                        j: Default::default(),
                    }
                }
            },
            t3,
        )
    }

    fn calculate_acceleration_distance(&self, v_start: dt!(T), v_end: dt!(T)) -> T {
        let tc = self.a_max / self.j_max;
        let vd = self.a_max * tc;
        let t = if (v_end - v_start).abs() >= vd {
            let tm = (v_end - v_start).abs() / self.a_max - tc;
            2.0 * tc + tm
        } else {
            2.0 * ((v_end - v_start).abs() / self.j_max).sqrt()
        };
        (v_start + v_end) * t / 2.0
    }

    fn generate_constant(
        x_start: T,
        distance: T,
        v: dt!(T),
    ) -> (impl Fn(Time) -> SubTarget<T>, Time) {
        let t_end = if (distance / v).is_negative() {
            Default::default()
        } else {
            distance / v
        };
        (
            move |t: Time| {
                if t > t_end || t < Default::default() {
                    SubTarget {
                        x: x_start + v * t_end,
                        v,
                        a: Default::default(),
                        j: Default::default(),
                    }
                } else {
                    SubTarget {
                        j: Default::default(),
                        a: Default::default(),
                        v,
                        x: x_start + v * t,
                    }
                }
            },
            t_end,
        )
    }

    //TODO: devise faster algorithm
    pub fn calculate_reachable_speed(&self, v_start: dt!(T), v_end: dt!(T), distance: T) -> dt!(T) {
        if v_start < v_end {
            let mut low = v_start;
            let mut high = self.v_max;
            for _ in 0..Self::LOOP_COUNT {
                let mid = (low + high) / 2.0;
                let d = self.calculate_acceleration_distance(v_start, mid);
                if d < distance {
                    low = mid;
                } else {
                    high = mid;
                }
            }
            low
        } else {
            let mut low3 = Default::default();
            let mut high3 = v_start;
            for _ in 0..Self::LOOP_COUNT {
                let low_mid = (low3 * 2.0 + high3) / 3.0;
                let high_mid = high3 - low_mid;
                let low_d = self.calculate_acceleration_distance(v_start, low_mid);
                let high_d = self.calculate_acceleration_distance(v_start, high_mid);
                if low_d < high_d {
                    low3 = low_mid;
                } else {
                    high3 = high_mid;
                }
            }
            let mut low = Default::default();
            let mut high = low3;
            for _ in 0..Self::LOOP_COUNT {
                let mid = (low + high) / 2.0;
                let d = self.calculate_acceleration_distance(v_start, mid);
                if d < distance {
                    low = mid;
                } else {
                    high = mid;
                }
            }
            let cand1 = low;

            let mut low = low3;
            let mut high = v_start;
            for _ in 0..Self::LOOP_COUNT {
                let mid = (low + high) / 2.0;
                let d = self.calculate_acceleration_distance(v_start, mid);
                if d < distance {
                    high = mid;
                } else {
                    low = mid;
                }
            }
            let cand2 = low;

            let d_cand1 = self.calculate_acceleration_distance(v_start, cand1);
            let d_cand2 = self.calculate_acceleration_distance(v_start, cand2);

            //admissible 1% relative error
            const EPSILON: f32 = 0.01;

            let diff_cand1: f32 = (distance - d_cand1).abs() / distance;
            let diff_cand2: f32 = (distance - d_cand2).abs() / distance;

            if diff_cand1 < EPSILON {
                if diff_cand2 < EPSILON {
                    if (v_end - cand1).abs() < (v_end - cand2).abs() {
                        cand1
                    } else {
                        cand2
                    }
                } else {
                    cand1
                }
            } else if diff_cand2 < EPSILON {
                cand2
            } else if diff_cand1 < diff_cand2 {
                cand1
            } else {
                cand2
            }
        }
    }
}

pub struct StraightTrajectoryGenerator {
    function_generator: StraightFunctionGenerator<Distance>,
    period: Time,
}

impl StraightTrajectoryGenerator {
    pub fn new(v_max: Speed, a_max: Acceleration, j_max: Jerk, period: Time) -> Self {
        Self {
            function_generator: StraightFunctionGenerator::new(v_max, a_max, j_max),
            period,
        }
    }
}

impl StraightTrajectoryGenerator {
    fn calculate_distance(x_dist: Distance, y_dist: Distance) -> Distance {
        let x_dist_raw = f32::from(x_dist);
        let y_dist_raw = f32::from(y_dist);

        Distance::from_meters(libm::sqrtf(
            x_dist_raw * x_dist_raw + y_dist_raw * y_dist_raw,
        ))
    }

    //NOTO: v_start and v_end should not be negative
    pub fn generate(
        &self,
        x_start: Distance,
        y_start: Distance,
        x_end: Distance,
        y_end: Distance,
        v_start: Speed,
        v_end: Speed,
    ) -> impl Iterator<Item = Target> {
        let x_dist = x_end - x_start;
        let y_dist = y_end - y_start;

        let dist = Self::calculate_distance(x_dist, y_dist);

        let (trajectory_fn, t_end) =
            self.function_generator
                .generate(Default::default(), dist, v_start, v_end);

        let x_ratio = x_dist / dist;
        let y_ratio = y_dist / dist;

        let theta = Angle::from_radian(libm::atan2f(y_dist.as_meters(), x_dist.as_meters()));

        StraightTrajectory::new(
            trajectory_fn,
            t_end,
            self.period,
            x_ratio,
            y_ratio,
            x_start,
            y_start,
            theta,
        )
    }

    #[allow(unused)]
    pub fn generate_constant(
        &self,
        x_start: Distance,
        y_start: Distance,
        x_end: Distance,
        y_end: Distance,
        v: Speed,
        period: Time,
    ) -> impl Iterator<Item = Target> {
        let x_dist = x_end - x_start;
        let y_dist = y_end - y_start;

        let dist = Self::calculate_distance(x_dist, y_dist);

        let (trajectory_fn, t_end) =
            StraightFunctionGenerator::<Distance>::generate_constant(Default::default(), dist, v);

        let x_ratio = x_dist / dist;
        let y_ratio = y_dist / dist;

        let theta = Angle::from_radian(libm::atan2f(y_dist.as_meters(), x_dist.as_meters()));

        StraightTrajectory::new(
            trajectory_fn,
            t_end,
            period,
            x_ratio,
            y_ratio,
            x_start,
            y_start,
            theta,
        )
    }
}

pub struct StraightTrajectory<F> {
    trajectory_fn: F,
    t: Time,
    t_end: Time,
    period: Time,
    x_ratio: f32,
    y_ratio: f32,
    x_start: Distance,
    y_start: Distance,
    theta: Angle,
}

impl<F> StraightTrajectory<F> {
    fn new(
        trajectory_fn: F,
        t_end: Time,
        period: Time,
        x_ratio: f32,
        y_ratio: f32,
        x_start: Distance,
        y_start: Distance,
        theta: Angle,
    ) -> Self {
        Self {
            trajectory_fn,
            t: Time::from_seconds(0.0),
            t_end,
            period,
            x_ratio,
            y_ratio,
            x_start,
            y_start,
            theta,
        }
    }
}

impl<F> Iterator for StraightTrajectory<F>
where
    F: Fn(Time) -> SubTarget<Distance>,
{
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        if self.t > self.t_end {
            return None;
        }
        let t = self.t;
        self.t += self.period;
        let target = (self.trajectory_fn)(t);
        Some(Target {
            x: SubTarget {
                x: self.x_start + target.x * self.x_ratio,
                v: target.v * self.x_ratio,
                a: target.a * self.x_ratio,
                j: target.j * self.x_ratio,
            },
            y: SubTarget {
                x: self.y_start + target.x * self.y_ratio,
                v: target.v * self.y_ratio,
                a: target.a * self.y_ratio,
                j: target.j * self.y_ratio,
            },
            theta: SubTarget {
                x: self.theta,
                ..Default::default()
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use quantities::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Jerk, Speed,
    };

    const EPSILON: f32 = 5e-4; //low accuracy...

    fn test_trajectory_fn<T>(
        period: Time,
        v_max: dt!(T),
        a_max: ddt!(T),
        j_max: dddt!(T),
        x_start: T,
        x_end: T,
        v_start: dt!(T),
        v_end: dt!(T),
        output: bool,
    ) where
        T: TimeDifferentiable + From<f32> + Div<T, Output = f32>,
        dt!(T): TimeDifferentiable
            + Mul<Time, Output = T>
            + Div<dddt!(T), Output = SquaredTime>
            + From<f32>,
        ddt!(T): TimeDifferentiable + Mul<Time, Output = dt!(T)> + From<f32>,
        dddt!(T): Quantity + Mul<Time, Output = ddt!(T)> + From<f32>,
        f32: From<T> + From<dt!(T)> + From<ddt!(T)> + From<dddt!(T)>,
    {
        let generator = StraightFunctionGenerator::new(v_max, a_max, j_max);
        let (trajectory_fn, t_end) = generator.generate(x_start, x_end, v_start, v_end);
        let mut current = Time::from_seconds(0.0);
        let mut before = trajectory_fn(current);
        let t_eps = T::from(EPSILON);
        let ddt_eps = <<T as Div<Time>>::Output as Div<Time>>::Output::from(EPSILON);
        let dddt_eps =
            <<<T as Div<Time>>::Output as Div<Time>>::Output as Div<Time>>::Output::from(EPSILON);
        assert!(
            (trajectory_fn(Default::default()).x - x_start).abs() < t_eps,
            "lhs: {:?}, rhs: {:?}",
            trajectory_fn(Default::default()).x,
            x_start
        );
        while current < t_end {
            let target = trajectory_fn(current);
            if output {
                let x_raw: f32 = target.x.into();
                let v_raw: f32 = target.v.into();
                let a_raw: f32 = target.a.into();
                let j_raw: f32 = target.j.into();
                println!("{}, {}, {}, {}", x_raw, v_raw, a_raw, j_raw,);
            }
            assert!(
                ((target.v - before.v) / period).abs() <= a_max.abs() + ddt_eps,
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.v - before.v) / period).abs(),
                a_max.abs() + ddt_eps,
            );
            assert!(
                ((target.a - before.a) / period).abs() <= j_max.abs() + dddt_eps,
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.a - before.a) / period).abs(),
                j_max.abs() + dddt_eps,
            );
            before = target;
            current += period;
        }
    }

    #[test]
    fn test_straight_trajectory_long() {
        test_trajectory_fn(
            Time::from_seconds(0.001),
            Speed::from_meter_per_second(1.0),
            Acceleration::from_meter_per_second_squared(1.0),
            Jerk::from_meter_per_second_cubed(1.0),
            Distance::from_meters(0.0),
            Distance::from_meters(3.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(0.0),
            false,
        );
    }

    #[test]
    fn test_straight_trajectory_with_angle() {
        test_trajectory_fn(
            Time::from_seconds(0.001),
            AngularSpeed::from_radian_per_second(1.0),
            AngularAcceleration::from_radian_per_second_squared(1.0),
            AngularJerk::from_radian_per_second_cubed(1.0),
            Angle::from_radian(0.0),
            Angle::from_radian(3.0),
            AngularSpeed::from_radian_per_second(0.0),
            AngularSpeed::from_radian_per_second(0.0),
            false,
        );
    }

    #[test]
    fn test_straight_trajectory_short() {
        test_trajectory_fn(
            Time::from_seconds(0.001),
            Speed::from_meter_per_second(1.0),
            Acceleration::from_meter_per_second_squared(1.0),
            Jerk::from_meter_per_second_cubed(1.0),
            Distance::from_meters(0.0),
            Distance::from_meters(1.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(0.0),
            false,
        );
    }

    #[test]
    fn test_straight_trajectory_with_only_acceleration() {
        test_trajectory_fn(
            Time::from_seconds(0.001),
            Speed::from_meter_per_second(1.0),
            Acceleration::from_meter_per_second_squared(0.5),
            Jerk::from_meter_per_second_cubed(1.0),
            Distance::from_meters(0.0),
            Distance::from_meters(1.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(1.0),
            false,
        );
    }

    #[test]
    fn test_straight_trajectory_with_only_deceleration() {
        test_trajectory_fn(
            Time::from_seconds(0.001),
            Speed::from_meter_per_second(1.0),
            Acceleration::from_meter_per_second_squared(0.5),
            Jerk::from_meter_per_second_cubed(1.0),
            Distance::from_meters(0.0),
            Distance::from_meters(1.0),
            Speed::from_meter_per_second(1.0),
            Speed::from_meter_per_second(0.0),
            false,
        );
    }

    #[test]
    fn test_straight_trajectory_with_minus_value() {
        test_trajectory_fn(
            Time::from_seconds(0.001),
            Speed::from_meter_per_second(1.0),
            Acceleration::from_meter_per_second_squared(1.0),
            Jerk::from_meter_per_second_cubed(1.0),
            Distance::from_meters(0.0),
            Distance::from_meters(-3.0),
            Speed::from_meter_per_second(1.0),
            Speed::from_meter_per_second(0.0),
            false,
        );
    }

    #[test]
    fn test_straight_trajectory_with_corner_case1() {
        test_trajectory_fn(
            Time::from_seconds(0.001),
            AngularSpeed::from_degree_per_second(90.0),
            AngularAcceleration::from_degree_per_second_squared(45.0),
            AngularJerk::from_radian_per_second_cubed(10.0),
            Angle::from_degree(90.0),
            Angle::from_degree(90.0),
            AngularSpeed::from_radian_per_second(0.0),
            AngularSpeed::from_radian_per_second(0.0),
            false,
        );
    }

    #[test]
    fn test_straight_trajectory_with_corner_case2() {
        test_trajectory_fn(
            Time::from_seconds(0.001),
            Speed::from_meter_per_second(2.0),
            Acceleration::from_meter_per_second_squared(1.0),
            Jerk::from_meter_per_second_cubed(0.5),
            Distance::from_meters(0.0),
            Distance::from_meters(0.06),
            Speed::from_meter_per_second(0.6),
            Speed::from_meter_per_second(0.0),
            false,
        )
    }

    #[test]
    fn test_straight_trajectory_2d() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(2.0);
        let a_max = Acceleration::from_meter_per_second_squared(1.0);
        let j_max = Jerk::from_meter_per_second_cubed(0.5);
        let generator = StraightTrajectoryGenerator::new(v_max, a_max, j_max, period);
        let _trajectory = generator.generate(
            Distance::from_meters(-0.06),
            Distance::from_meters(0.16),
            Distance::from_meters(-0.12),
            Distance::from_meters(0.16),
            Speed::from_meter_per_second(0.6),
            Speed::from_meter_per_second(0.0),
        );
    }
}
