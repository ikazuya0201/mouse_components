use core::ops::{Add, Div, Mul};

use quantities::{Quantity, Time, TimeDifferentiable};

use super::trajectory::Target;

pub struct StraightGenerator<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable,
    ddt!(T): TimeDifferentiable,
    dddt!(T): Quantity,
{
    period: Time,
    v_max: dt!(T),
    a_max: ddt!(T),
    j_max: dddt!(T),
}

impl<T> StraightGenerator<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable + Mul<Time, Output = T>,
    ddt!(T): TimeDifferentiable + Mul<Time, Output = dt!(T)>,
    dddt!(T): Quantity + Mul<Time, Output = ddt!(T)>,
    f32: From<dt!(T)> + From<ddt!(T)> + From<dddt!(T)>,
{
    const LOOP_COUNT: u8 = 30;

    pub fn new(period: Time, v_max: dt!(T), a_max: ddt!(T), j_max: dddt!(T)) -> Self {
        Self {
            period,
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
        let (decel_fn, dt3) = self.generate_acceleration(distance - dist3, v_max, v_end);

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
            let td = Time::from_seconds(
                (f32::from((v_end - v_start).abs()) / f32::from(self.j_max)).sqrt(),
            );
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
            2.0 * Time::from_seconds(
                (f32::from((v_end - v_start).abs()) / f32::from(self.j_max)).sqrt(),
            )
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
                        v: v,
                        a: Default::default(),
                        j: Default::default(),
                    }
                } else {
                    Target {
                        j: Default::default(),
                        a: Default::default(),
                        v: v,
                        x: x_start + v * t,
                    }
                }
            },
            t_end,
        )
    }

    pub fn calculate_reachable_speed(
        &self,
        v_start: <T as Div<Time>>::Output,
        distance: T,
    ) -> <T as Div<Time>>::Output {
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

#[cfg(test)]
mod tests {
    use super::*;
    use approx::abs_diff_eq;
    use quantities::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Jerk, Speed,
    };

    const EPSILON: f32 = 1e-4; //low accuracy...

    #[test]
    fn test_straight_trajectory_long() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(1.0);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightGenerator::new(period, v_max, a_max, j_max);
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
        abs_diff_eq!(Distance::from_meters(1.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_with_angle() {
        let period = Time::from_seconds(0.001);
        let v_max = AngularSpeed::from_radian_per_second(1.0);
        let a_max = AngularAcceleration::from_radian_per_second_squared(1.0);
        let j_max = AngularJerk::from_radian_per_second_cubed(1.0);
        let generator = StraightGenerator::new(period, v_max, a_max, j_max);
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
        abs_diff_eq!(Angle::from_radian(1.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_short() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(1.0);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightGenerator::new(period, v_max, a_max, j_max);
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
        abs_diff_eq!(Distance::from_meters(1.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_with_only_acceleration() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(0.5);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightGenerator::new(period, v_max, a_max, j_max);
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
        abs_diff_eq!(Distance::from_meters(1.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_with_only_deceleration() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(0.5);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightGenerator::new(period, v_max, a_max, j_max);
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
        abs_diff_eq!(Distance::from_meters(1.0), before.x, epsilon = EPSILON);
    }

    #[test]
    fn test_straight_trajectory_with_minus_value() {
        let period = Time::from_seconds(0.001);
        let v_max = AngularSpeed::from_radian_per_second(1.0);
        let a_max = AngularAcceleration::from_radian_per_second_squared(1.0);
        let j_max = AngularJerk::from_radian_per_second_cubed(1.0);
        let generator = StraightGenerator::new(period, v_max, a_max, j_max);
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
            println!("{}", target.x.as_degree());
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
        abs_diff_eq!(Angle::from_radian(-3.0), before.x, epsilon = EPSILON);
    }
}
