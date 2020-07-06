use super::trajectory::Target;
use quantities::{Acceleration, Distance, Jerk, Speed, Time};

pub struct StraightGenerator {
    period: Time,
    v_max: Speed,
    a_max: Acceleration,
    j_max: Jerk,
}

impl StraightGenerator {
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
        let vd = self.a_max * (self.a_max / self.j_max);
        let dt = if self.v_max >= v_start + vd {
            (v_start + self.v_max) * ((self.v_max - v_start) / self.a_max + self.a_max / self.j_max)
                / 2.0
        } else {
            (v_start + self.v_max)
                * Time::from_seconds(
                    ((self.v_max - v_start).as_meter_per_second()
                        / self.j_max.as_meter_per_second_cubed())
                    .sqrt(),
                )
        } + if self.v_max >= v_end + vd {
            (v_end + self.v_max) * ((self.v_max - v_end) / self.a_max + self.a_max / self.j_max)
                / 2.0
        } else {
            (v_start + self.v_max)
                * Time::from_seconds(
                    ((self.v_max - v_end).as_meter_per_second()
                        / self.j_max.as_meter_per_second_cubed())
                    .sqrt(),
                )
        };
        self.generate_acceleration(x_start, v_start, v_end)
    }

    fn generate_acceleration(
        &self,
        x_start: Distance,
        v_start: Speed,
        v_end: Speed,
    ) -> impl Iterator<Item = Target> {
        AccelerationTrajectory::new(self.period, x_start, v_start, v_end, self.a_max, self.j_max)
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
    a_max: Acceleration,
    j_max: Jerk,
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
        let (t1, t2, t3) = if v_end >= v_start + vd {
            let tm = (v_end - v_start) / a_max - tc;
            (tc, tc + tm, 2.0 * tc + tm)
        } else {
            let td = Time::from_seconds(
                ((v_end - v_start).as_meter_per_second() / j_max.as_meter_per_second_cubed())
                    .sqrt(),
            );
            (td, td, 2.0 * td)
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
            a_max,
            j_max,
        }
    }
}

impl Iterator for AccelerationTrajectory {
    type Item = Target;
    fn next(&mut self) -> Option<Self::Item> {
        let t = self.t;
        self.t += self.dt;
        if t <= self.t1 {
            Some(Target {
                j: self.j_max,
                a: self.j_max * t,
                v: self.v_start + self.j_max * t * t / 2.0,
                x: self.x_start + self.v_start * t + self.j_max * t * t * t / 6.0,
            })
        } else if t <= self.t2 {
            let dt1 = t - self.t1;
            Some(Target {
                j: Jerk::from_meter_per_second_cubed(0.0),
                a: self.a_max,
                v: self.v_start + self.a_max * self.t1 / 2.0 + self.a_max * dt1,
                x: self.x_start
                    + self.v_start * self.t1
                    + self.j_max * self.t1 * self.t1 * self.t1 / 6.0
                    + (self.v_start + self.a_max * self.t1 / 2.0) * dt1
                    + self.a_max * dt1 * dt1 / 2.0,
            })
        } else if t <= self.t3 {
            let dt3 = self.t3 - t;
            Some(Target {
                j: -self.j_max,
                a: self.j_max * dt3,
                v: self.v_end - self.j_max * dt3 * dt3 / 2.0,
                x: self.x_end - self.v_end * dt3 + self.j_max * dt3 * dt3 * dt3 / 6.0,
            })
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 1e-4; //low accuracy...

    #[test]
    fn test_acceleration_trajectory() {
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
    }
}
