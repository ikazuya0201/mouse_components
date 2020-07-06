use super::trajectory::Target;
use quantities::{Acceleration, Distance, Jerk, Speed, Time};

pub struct StraightGenerator {
    period: Time,
    v_max: Speed,
    a_max: Acceleration,
    j_max: Jerk,
}

impl StraightGenerator {
    const LOOP_COUNT: u8 = 30;

    pub fn new(period: Time, v_max: Speed, a_max: Acceleration, j_max: Jerk) -> Self {
        Self {
            period,
            v_max,
            a_max,
            j_max,
        }
    }

    pub fn generate(
        &self,
        x_start: Distance,
        distance: Distance,
        v_start: Speed,
        v_end: Speed,
    ) -> impl Iterator<Item = Target> {
        let mut left = if v_start < v_end { v_end } else { v_start };
        let mut right = self.calculate_reachable_speed(v_start, distance);
        if left > right {
            left = right;
        }
        for _ in 0..Self::LOOP_COUNT {
            let mid = (left + right) / 2.0;
            let d =
                AccelerationTrajectory::calculate_distance(v_start, mid, self.a_max, self.j_max)
                    + AccelerationTrajectory::calculate_distance(
                        mid, v_end, self.a_max, self.j_max,
                    );
            if d <= distance {
                left = mid;
            } else {
                right = mid;
            }
        }

        let v_max = left;
        let v_end = if v_end < v_max { v_end } else { v_max };

        let first_dist =
            AccelerationTrajectory::calculate_distance(v_start, v_max, self.a_max, self.j_max);
        let second_dist =
            AccelerationTrajectory::calculate_distance(v_max, v_end, self.a_max, self.j_max);
        self.generate_acceleration(x_start, v_start, v_max)
            .chain(self.generate_constant(
                x_start + first_dist,
                distance - first_dist - second_dist,
                v_max,
            ))
            .chain(self.generate_acceleration(distance - second_dist, v_max, v_end))
    }

    fn generate_acceleration(
        &self,
        x_start: Distance,
        v_start: Speed,
        v_end: Speed,
    ) -> impl Iterator<Item = Target> {
        AccelerationTrajectory::new(self.period, x_start, v_start, v_end, self.a_max, self.j_max)
    }

    fn generate_constant(
        &self,
        x_start: Distance,
        distance: Distance,
        v: Speed,
    ) -> impl Iterator<Item = Target> {
        ConstantSpeedTrajectory::new(self.period, x_start, distance, v)
    }

    pub fn calculate_reachable_speed(&self, v_start: Speed, distance: Distance) -> Speed {
        let mut left = v_start;
        let mut right = self.v_max;
        for _ in 0..Self::LOOP_COUNT {
            let mid = (left + right) / 2.0;
            let d =
                AccelerationTrajectory::calculate_distance(v_start, mid, self.a_max, self.j_max);
            if d < distance {
                left = mid;
            } else {
                right = mid;
            }
        }
        left
    }
}

struct AccelerationTrajectory {
    t: Time,
    dt: Time,
    t1: Time,
    t2: Time,
    t3: Time,
    x_start: Distance,
    x_end: Distance,
    v_start: Speed,
    v_end: Speed,
    a_m: Acceleration,
    j_m: Jerk,
}

impl AccelerationTrajectory {
    fn new(
        dt: Time,
        x_start: Distance,
        v_start: Speed,
        v_end: Speed,
        a_max: Acceleration,
        j_max: Jerk,
    ) -> Self {
        let tc = a_max / j_max;
        let vd = a_max * tc;
        let (t1, t2, t3) = if (v_end - v_start).abs() >= vd {
            let tm = (v_end - v_start).abs() / a_max - tc;
            (tc, tc + tm, 2.0 * tc + tm)
        } else {
            let td = Time::from_seconds(
                ((v_end - v_start).abs().as_meter_per_second() / j_max.as_meter_per_second_cubed())
                    .sqrt(),
            );
            (td, td, 2.0 * td)
        };

        let (a_m, j_m) = if v_start <= v_end {
            (a_max, j_max)
        } else {
            (-a_max, -j_max)
        };

        Self {
            t: Time::from_seconds(0.0),
            dt,
            t1,
            t2,
            t3,
            x_start,
            x_end: x_start + (v_start + v_end) * t3 / 2.0,
            v_start,
            v_end,
            a_m,
            j_m,
        }
    }

    fn calculate_distance(
        v_start: Speed,
        v_end: Speed,
        a_max: Acceleration,
        j_max: Jerk,
    ) -> Distance {
        let tc = a_max / j_max;
        let vd = a_max * tc;
        let t = if (v_end - v_start).abs() >= vd {
            let tm = (v_end - v_start).abs() / a_max - tc;
            2.0 * tc + tm
        } else {
            2.0 * Time::from_seconds(
                ((v_end - v_start).abs().as_meter_per_second() / j_max.as_meter_per_second_cubed())
                    .sqrt(),
            )
        };
        (v_start + v_end) * t / 2.0
    }
}

impl Iterator for AccelerationTrajectory {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        let t = self.t;
        if t > self.t3 {
            return None;
        }
        self.t += self.dt;
        if t <= self.t1 {
            Some(Target {
                j: self.j_m,
                a: self.j_m * t,
                v: self.v_start + self.j_m * t * t / 2.0,
                x: self.x_start + self.v_start * t + self.j_m * t * t * t / 6.0,
            })
        } else if t <= self.t2 {
            let dt1 = t - self.t1;
            Some(Target {
                j: Jerk::from_meter_per_second_cubed(0.0),
                a: self.a_m,
                v: self.v_start + self.a_m * self.t1 / 2.0 + self.a_m * dt1,
                x: self.x_start
                    + self.v_start * self.t1
                    + self.j_m * self.t1 * self.t1 * self.t1 / 6.0
                    + (self.v_start + self.a_m * self.t1 / 2.0) * dt1
                    + self.a_m * dt1 * dt1 / 2.0,
            })
        } else {
            let dt3 = self.t3 - t;
            Some(Target {
                j: -self.j_m,
                a: self.j_m * dt3,
                v: self.v_end - self.j_m * dt3 * dt3 / 2.0,
                x: self.x_end - self.v_end * dt3 + self.j_m * dt3 * dt3 * dt3 / 6.0,
            })
        }
    }
}

struct ConstantSpeedTrajectory {
    t: Time,
    dt: Time,
    t_end: Time,
    x_start: Distance,
    v: Speed,
}

impl ConstantSpeedTrajectory {
    fn new(dt: Time, x_start: Distance, distance: Distance, v: Speed) -> Self {
        Self {
            t: Time::from_seconds(0.0),
            dt,
            t_end: distance / v,
            x_start,
            v,
        }
    }
}

impl Iterator for ConstantSpeedTrajectory {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        let t = self.t;
        if t > self.t_end {
            return None;
        }
        self.t += self.dt;
        Some(Target {
            j: Jerk::from_meter_per_second_cubed(0.0),
            a: Acceleration::from_meter_per_second_squared(0.0),
            v: self.v,
            x: self.x_start + self.v * self.t,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::abs_diff_eq;

    const EPSILON: f32 = 1e-4; //low accuracy...

    #[test]
    fn test_acceleration_trajectory_up() {
        let period = Time::from_seconds(0.001);
        let a_max = Acceleration::from_meter_per_second_squared(1.0);
        let j_max = Jerk::from_meter_per_second_cubed(2.0);
        let mut trajectory = AccelerationTrajectory::new(
            period,
            Distance::from_meters(0.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(1.0),
            a_max,
            j_max,
        );
        let mut before = trajectory.next().unwrap();
        for target in trajectory {
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
    fn test_acceleration_trajectory_down() {
        let period = Time::from_seconds(0.001);
        let a_max = Acceleration::from_meter_per_second_squared(1.0);
        let j_max = Jerk::from_meter_per_second_cubed(2.0);
        let mut trajectory = AccelerationTrajectory::new(
            period,
            Distance::from_meters(0.0),
            Speed::from_meter_per_second(1.0),
            Speed::from_meter_per_second(0.0),
            a_max,
            j_max,
        );
        let mut before = trajectory.next().unwrap();
        for target in trajectory {
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
    fn test_straight_trajectory_long() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(1.0);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightGenerator::new(period, v_max, a_max, j_max);
        let mut trajectory = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(3.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(0.0),
        );
        let mut before = trajectory.next().unwrap();
        for target in trajectory {
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
    fn test_straight_trajectory_short() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(1.0);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightGenerator::new(period, v_max, a_max, j_max);
        let mut trajectory = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(1.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(0.0),
        );
        let mut before = trajectory.next().unwrap();
        for target in trajectory {
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
    fn test_straight_trajectory_with_no_deceleration() {
        let period = Time::from_seconds(0.001);
        let v_max = Speed::from_meter_per_second(1.0);
        let a_max = Acceleration::from_meter_per_second_squared(0.5);
        let j_max = Jerk::from_meter_per_second_cubed(1.0);
        let generator = StraightGenerator::new(period, v_max, a_max, j_max);
        let mut trajectory = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(1.0),
            Speed::from_meter_per_second(0.0),
            Speed::from_meter_per_second(1.0),
        );
        let mut before = trajectory.next().unwrap();
        for target in trajectory {
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
}
