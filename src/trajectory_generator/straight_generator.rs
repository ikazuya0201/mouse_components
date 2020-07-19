use core::ops::{Div, Mul};

use quantities::{
    Acceleration, Distance, Jerk, Quantity, Speed, SquaredTime, Time, TimeDifferentiable,
};

use super::trajectory::Target;
use crate::utils::vector::Vector2;
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
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable + Mul<Time, Output = T> + Div<dddt!(T), Output = SquaredTime>,
    ddt!(T): TimeDifferentiable + Mul<Time, Output = dt!(T)>,
    dddt!(T): Quantity + Mul<Time, Output = ddt!(T)>,
    f32: From<dt!(T)> + From<ddt!(T)> + From<dddt!(T)>,
{
    const LOOP_COUNT: u8 = 30;

    pub fn new(v_max: dt!(T), a_max: ddt!(T), j_max: dddt!(T)) -> Self {
        Self {
            v_max,
            a_max,
            j_max,
        }
    }

    pub fn generate(
        &self,
        x_start: T,
        distance: T,
        v_start: dt!(T),
        v_end: dt!(T),
    ) -> (impl Fn(Time) -> Target<T>, Time) {
        let is_negative = distance.is_negative();
        let (x_start, distance, v_start, v_end) = if is_negative {
            (-x_start, -distance, -v_start, -v_end)
        } else {
            (x_start, distance, v_start, v_end)
        };

        let mut left = if v_start < v_end { v_end } else { v_start };
        let mut right = self.calculate_reachable_speed(v_start, distance);
        if left > right {
            left = right;
        }
        for _ in 0..Self::LOOP_COUNT {
            let mid = (left + right) / 2.0;
            let d = self.calculate_acceleration_distance(v_start, mid)
                + self.calculate_acceleration_distance(mid, v_end);
            if d <= distance {
                left = mid;
            } else {
                right = mid;
            }
        }

        let v_max = left;
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
                let target = if t < Default::default() {
                    Target {
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
                    Target {
                        x: x_start + distance,
                        v: v_end,
                        a: Default::default(),
                        j: Default::default(),
                    }
                };
                if is_negative {
                    Target {
                        x: -target.x,
                        v: -target.v,
                        a: -target.a,
                        j: -target.j,
                    }
                } else {
                    target
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
    ) -> (impl Fn(Time) -> Target<T>, Time) {
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
            move |t: Time| -> Target<T> {
                if t < Default::default() {
                    Target {
                        x: x_start,
                        v: v_start,
                        a: Default::default(),
                        j: Default::default(),
                    }
                } else if t <= t1 {
                    Target {
                        j: j_m,
                        a: j_m * t,
                        v: v_start + j_m * t * t / 2.0,
                        x: x_start + v_start * t + j_m * t * t * t / 6.0,
                    }
                } else if t <= t2 {
                    let dt1 = t - t1;
                    Target {
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
                    Target {
                        j: -j_m,
                        a: j_m * dt3,
                        v: v_end - j_m * dt3 * dt3 / 2.0,
                        x: x_end - v_end * dt3 + j_m * dt3 * dt3 * dt3 / 6.0,
                    }
                } else {
                    Target {
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

    fn generate_constant(x_start: T, distance: T, v: dt!(T)) -> (impl Fn(Time) -> Target<T>, Time) {
        let t_end = if (distance / v).is_negative() {
            Default::default()
        } else {
            distance / v
        };
        (
            move |t: Time| {
                if t > t_end || t < Default::default() {
                    Target {
                        x: x_start + v * t_end,
                        v,
                        a: Default::default(),
                        j: Default::default(),
                    }
                } else {
                    Target {
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

    pub fn calculate_reachable_speed(&self, v_start: dt!(T), distance: T) -> dt!(T) {
        let mut left = v_start;
        let mut right = self.v_max;
        for _ in 0..Self::LOOP_COUNT {
            let mid = (left + right) / 2.0;
            let d = self.calculate_acceleration_distance(v_start, mid);
            if d < distance {
                left = mid;
            } else {
                right = mid;
            }
        }
        left
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

        Distance::from_meters((x_dist_raw * x_dist_raw + y_dist_raw * y_dist_raw).sqrt())
    }

    pub fn generate(
        &self,
        x_start: Distance,
        y_start: Distance,
        x_end: Distance,
        y_end: Distance,
        v_start: Speed,
        v_end: Speed,
    ) -> impl Iterator<Item = Vector2<Target<Distance>>> {
        let x_dist = x_end - x_start;
        let y_dist = y_end - y_start;

        let dist = Self::calculate_distance(x_dist, y_dist);

        let (trajectory_fn, t_end) =
            self.function_generator
                .generate(Default::default(), dist, v_start, v_end);

        let x_ratio = x_dist / dist;
        let y_ratio = y_dist / dist;

        StraightTrajectory::new(
            trajectory_fn,
            t_end,
            self.period,
            x_ratio,
            y_ratio,
            x_start,
            y_start,
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
    ) -> impl Iterator<Item = Vector2<Target<Distance>>> {
        let x_dist = x_end - x_start;
        let y_dist = y_end - y_start;

        let dist = Self::calculate_distance(x_dist, y_dist);

        let (trajectory_fn, t_end) =
            StraightFunctionGenerator::<Distance>::generate_constant(Default::default(), dist, v);

        let x_ratio = x_dist / dist;
        let y_ratio = y_dist / dist;

        StraightTrajectory::new(
            trajectory_fn,
            t_end,
            period,
            x_ratio,
            y_ratio,
            x_start,
            y_start,
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
        }
    }
}

impl<F> Iterator for StraightTrajectory<F>
where
    F: Fn(Time) -> Target<Distance>,
{
    type Item = Vector2<Target<Distance>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.t > self.t_end {
            return None;
        }
        let t = self.t;
        self.t += self.period;
        let target = (self.trajectory_fn)(t);
        Some(Vector2 {
            x: Target {
                x: self.x_start + target.x * self.x_ratio,
                v: target.v * self.x_ratio,
                a: target.a * self.x_ratio,
                j: target.j * self.x_ratio,
            },
            y: Target {
                x: self.y_start + target.x * self.y_ratio,
                v: target.v * self.y_ratio,
                a: target.a * self.y_ratio,
                j: target.j * self.y_ratio,
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use quantities::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Jerk, Speed,
    };

    const EPSILON: f32 = 5e-4; //low accuracy...

    #[test]
    fn test_straight_trajectory_long() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(1.0);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightFunctionGenerator::new(v_max, a_max, j_max);
        let (trajectory_fn, t_end) = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(3.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(0.0),
        );
        let mut current = Time::from_seconds(0.0);
        let mut before = trajectory_fn(current);
        while current < t_end {
            current += period;
            let target = trajectory_fn(current);
            assert!(
                ((target.v - before.v) / period).abs()
                    <= a_max.abs() + Acceleration::from_meter_per_second_squared(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.v - before.v) / period).abs(),
                a_max.abs() + Acceleration::from_meter_per_second_squared(EPSILON),
            );
            assert!(
                ((target.a - before.a) / period).abs()
                    <= j_max.abs() + Jerk::from_meter_per_second_cubed(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.a - before.a) / period).abs(),
                j_max.abs() + Jerk::from_meter_per_second_cubed(EPSILON),
            );
            before = target;
        }
        assert_relative_eq!(Distance::from_meters(3.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_with_angle() {
        let period = Time::from_seconds(0.001);
        let v_max = AngularSpeed::from_radian_per_second(1.0);
        let a_max = AngularAcceleration::from_radian_per_second_squared(1.0);
        let j_max = AngularJerk::from_radian_per_second_cubed(1.0);
        let generator = StraightFunctionGenerator::new(v_max, a_max, j_max);
        let (trajectory_fn, t_end) = generator.generate(
            Angle::from_radian(0.0),
            Angle::from_radian(3.0),
            AngularSpeed::from_radian_per_second(0.0),
            AngularSpeed::from_radian_per_second(0.0),
        );
        let mut current = Time::from_seconds(0.0);
        let mut before = trajectory_fn(current);
        while current < t_end {
            current += period;
            let target = trajectory_fn(current);
            assert!(
                ((target.v - before.v) / period).abs()
                    <= a_max.abs() + AngularAcceleration::from_radian_per_second_squared(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.v - before.v) / period).abs(),
                a_max.abs() + AngularAcceleration::from_radian_per_second_squared(EPSILON),
            );
            assert!(
                ((target.a - before.a) / period).abs()
                    <= j_max.abs() + AngularJerk::from_radian_per_second_cubed(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.a - before.a) / period).abs(),
                j_max.abs() + AngularJerk::from_radian_per_second_cubed(EPSILON),
            );
            before = target;
        }
        assert_relative_eq!(Angle::from_radian(3.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_short() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(1.0);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightFunctionGenerator::new(v_max, a_max, j_max);
        let (trajectory_fn, t_end) = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(1.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(0.0),
        );
        let mut current = Time::from_seconds(0.0);
        let mut before = trajectory_fn(current);
        while current < t_end {
            current += period;
            let target = trajectory_fn(current);
            assert!(
                ((target.v - before.v) / period).abs()
                    <= a_max.abs() + Acceleration::from_meter_per_second_squared(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.v - before.v) / period).abs(),
                a_max.abs() + Acceleration::from_meter_per_second_squared(EPSILON),
            );
            assert!(
                ((target.a - before.a) / period).abs()
                    <= j_max.abs() + Jerk::from_meter_per_second_cubed(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.a - before.a) / period).abs(),
                j_max.abs() + Jerk::from_meter_per_second_cubed(EPSILON),
            );
            before = target;
        }
        assert_relative_eq!(Distance::from_meters(1.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_with_only_acceleration() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(0.5);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightFunctionGenerator::new(v_max, a_max, j_max);
        let (trajectory_fn, t_end) = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(1.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(1.0),
        );
        let mut current = Time::from_seconds(0.0);
        let mut before = trajectory_fn(current);
        while current < t_end {
            current += period;
            let target = trajectory_fn(current);
            assert!(
                ((target.v - before.v) / period).abs()
                    <= a_max.abs() + Acceleration::from_meter_per_second_squared(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.v - before.v) / period).abs(),
                a_max.abs() + Acceleration::from_meter_per_second_squared(EPSILON),
            );
            assert!(
                ((target.a - before.a) / period).abs()
                    <= j_max.abs() + Jerk::from_meter_per_second_cubed(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.a - before.a) / period).abs(),
                j_max.abs() + Jerk::from_meter_per_second_cubed(EPSILON),
            );
            before = target;
        }
        assert_relative_eq!(Distance::from_meters(1.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_with_only_deceleration() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(0.5);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightFunctionGenerator::new(v_max, a_max, j_max);
        let (trajectory_fn, t_end) = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(1.0),
            Speed::from_meter_per_second(1.0),
            Speed::from_meter_per_second(0.0),
        );
        let mut current = Time::from_seconds(0.0);
        let mut before = trajectory_fn(current);
        while current < t_end {
            current += period;
            let target = trajectory_fn(current);
            assert!(
                ((target.v - before.v) / period).abs()
                    <= a_max.abs() + Acceleration::from_meter_per_second_squared(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.v - before.v) / period).abs(),
                a_max.abs() + Acceleration::from_meter_per_second_squared(EPSILON),
            );
            assert!(
                ((target.a - before.a) / period).abs()
                    <= j_max.abs() + Jerk::from_meter_per_second_cubed(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.a - before.a) / period).abs(),
                j_max.abs() + Jerk::from_meter_per_second_cubed(EPSILON),
            );
            before = target;
        }
        assert_relative_eq!(Distance::from_meters(1.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_with_minus_value() {
        let period = Time::from_seconds(0.001);
        let v_max = AngularSpeed::from_radian_per_second(1.0);
        let a_max = AngularAcceleration::from_radian_per_second_squared(1.0);
        let j_max = AngularJerk::from_radian_per_second_cubed(1.0);
        let generator = StraightFunctionGenerator::new(v_max, a_max, j_max);
        let (trajectory_fn, t_end) = generator.generate(
            Angle::from_radian(0.0),
            Angle::from_radian(-3.0),
            AngularSpeed::from_radian_per_second(0.0),
            AngularSpeed::from_radian_per_second(0.0),
        );
        let mut current = Time::from_seconds(0.0);
        let mut before = trajectory_fn(current);
        while current < t_end {
            current += period;
            let target = trajectory_fn(current);
            assert!(
                ((target.v - before.v) / period).abs()
                    <= a_max.abs() + AngularAcceleration::from_radian_per_second_squared(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.v - before.v) / period).abs(),
                a_max.abs() + AngularAcceleration::from_radian_per_second_squared(EPSILON),
            );
            assert!(
                ((target.a - before.a) / period).abs()
                    <= j_max.abs() + AngularJerk::from_radian_per_second_cubed(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.a - before.a) / period).abs(),
                j_max.abs() + AngularJerk::from_radian_per_second_cubed(EPSILON),
            );
            before = target;
        }
        assert_relative_eq!(Angle::from_radian(-3.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_with_corner_case1() {
        let period = Time::from_seconds(0.001);
        let v_max = AngularSpeed::from_degree_per_second(90.0);
        let a_max = AngularAcceleration::from_degree_per_second_squared(45.0);
        let j_max = AngularJerk::from_radian_per_second_cubed(10.0);
        let generator = StraightFunctionGenerator::new(v_max, a_max, j_max);
        let (trajectory_fn, t_end) = generator.generate(
            Angle::from_degree(90.0),
            Angle::from_degree(90.0),
            AngularSpeed::from_radian_per_second(0.0),
            AngularSpeed::from_radian_per_second(0.0),
        );
        let mut current = Time::from_seconds(0.0);
        let mut before = trajectory_fn(current);
        while current < t_end {
            current += period;
            let target = trajectory_fn(current);
            assert!(
                ((target.x - before.x) / period).abs()
                    <= v_max.abs() + AngularSpeed::from_radian_per_second(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.x - before.x) / period).abs(),
                v_max.abs() + AngularSpeed::from_radian_per_second(EPSILON),
            );
            assert!(
                ((target.v - before.v) / period).abs()
                    <= a_max.abs() + AngularAcceleration::from_radian_per_second_squared(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.v - before.v) / period).abs(),
                a_max.abs() + AngularAcceleration::from_radian_per_second_squared(EPSILON),
            );
            assert!(
                ((target.a - before.a) / period).abs()
                    <= j_max.abs() + AngularJerk::from_radian_per_second_cubed(EPSILON),
                "{:?} {:?}, lhs: {:?}, rhs: {:?}",
                target,
                before,
                ((target.a - before.a) / period).abs(),
                j_max.abs() + AngularJerk::from_radian_per_second_cubed(EPSILON),
            );
            before = target;
        }
        assert_relative_eq!(Angle::from_degree(180.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_2d() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(0.5);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightTrajectoryGenerator::new(v_max, a_max, j_max, period);
        let x_end = Distance::from_meters(1.0);
        let y_end = Distance::from_meters(1.0);
        let trajectory = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(0.0),
            Distance::from_meters(1.0),
            Distance::from_meters(1.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(0.0),
        );
        let last = trajectory.last().unwrap();
        assert_relative_eq!(last.x.x, x_end, epsilon = EPSILON);
        assert_relative_eq!(last.y.x, y_end, epsilon = EPSILON);
    }
}
