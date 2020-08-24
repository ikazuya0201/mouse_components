use core::ops::{Div, Mul};

use quantities::{
    Acceleration, Angle, Distance, Jerk, Quantity, Speed, SquaredTime, Time, TimeDifferentiable,
};

use super::trajectory::{SubTarget, Target};
use crate::{dddt, ddt, dt};

pub struct StraightCalculatorGenerator<T>
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

impl<T> StraightCalculatorGenerator<T>
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

    //NOTO: v_start and v_end should not be negative
    pub fn generate(
        &self,
        x_start: T,
        distance: T,
        v_start: dt!(T),
        v_end: dt!(T),
    ) -> (OverallCalculator<T>, Time) {
        let (distance, sign) = if distance.is_negative() {
            (-distance, -1.0f32)
        } else {
            (distance, 1.0f32)
        };
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

        let (accel_calculator, dt1) = self.generate_acceleration(x_start, v_start, v_max);
        let (const_calculator, dt2) = Self::generate_constant(x_start + dist1, dist2, v_max);
        let (decel_calculator, dt3) =
            self.generate_acceleration(x_start + distance - dist3, v_max, v_end);

        let t1 = dt1;
        let t2 = t1 + dt2;
        let t3 = t2 + dt3;

        (
            OverallCalculator {
                x_start,
                v_start,
                t1,
                t2,
                t3,
                distance,
                v_end,
                accel_calculator,
                const_calculator,
                decel_calculator,
                sign,
            },
            t3,
        )
    }

    fn generate_acceleration(
        &self,
        x_start: T,
        v_start: dt!(T),
        v_end: dt!(T),
    ) -> (AccelerationCalculator<T>, Time) {
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
            AccelerationCalculator {
                x_start,
                v_start,
                x_end,
                v_end,
                a_m,
                j_m,
                t1,
                t2,
                t3,
            },
            t3,
        )
    }

    fn calculate_acceleration_distance(&self, v_start: dt!(T), v_end: dt!(T)) -> T {
        let tc = self.a_max / self.j_max;
        let vd = self.a_max * tc;
        let t = if (v_end - v_start).abs() >= vd {
            (v_end - v_start).abs() / self.a_max + tc
        } else {
            2.0 * ((v_end - v_start).abs() / self.j_max).sqrt()
        };
        (v_start + v_end) * t / 2.0
    }

    fn generate_constant(x_start: T, distance: T, v: dt!(T)) -> (ConstantCalculator<T>, Time) {
        let t_end = if (distance / v).is_negative() {
            Default::default()
        } else {
            distance / v
        };
        (ConstantCalculator { t_end, x_start, v }, t_end)
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
            let mut low = Default::default();
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
            low
        }
    }
}

#[derive(Clone)]
struct AccelerationCalculator<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable,
    ddt!(T): TimeDifferentiable,
    dddt!(T): Quantity,
{
    x_start: T,
    v_start: <T as Div<Time>>::Output,
    x_end: T,
    v_end: <T as Div<Time>>::Output,
    a_m: <<T as Div<Time>>::Output as Div<Time>>::Output,
    j_m: <<<T as Div<Time>>::Output as Div<Time>>::Output as Div<Time>>::Output,
    t1: Time,
    t2: Time,
    t3: Time,
}

impl<T> AccelerationCalculator<T>
where
    T: TimeDifferentiable + Div<T, Output = f32>,
    dt!(T): TimeDifferentiable + Mul<Time, Output = T> + Div<dddt!(T), Output = SquaredTime>,
    ddt!(T): TimeDifferentiable + Mul<Time, Output = dt!(T)>,
    dddt!(T): Quantity + Mul<Time, Output = ddt!(T)>,
    f32: From<T> + From<dt!(T)> + From<ddt!(T)> + From<dddt!(T)>,
{
    fn calculate(&self, t: Time) -> SubTarget<T> {
        if t < Default::default() {
            SubTarget {
                x: self.x_start,
                v: self.v_start,
                a: Default::default(),
                j: Default::default(),
            }
        } else if t <= self.t1 {
            SubTarget {
                j: self.j_m,
                a: self.j_m * t,
                v: self.v_start + self.j_m * t * t / 2.0,
                x: self.x_start + self.v_start * t + self.j_m * t * t * t / 6.0,
            }
        } else if t <= self.t2 {
            let dt1 = t - self.t1;
            SubTarget {
                j: Default::default(),
                a: self.a_m,
                v: self.v_start + self.a_m * self.t1 / 2.0 + self.a_m * dt1,
                x: self.x_start
                    + self.v_start * self.t1
                    + self.j_m * self.t1 * self.t1 * self.t1 / 6.0
                    + (self.v_start + self.a_m * self.t1 / 2.0) * dt1
                    + self.a_m * dt1 * dt1 / 2.0,
            }
        } else if t <= self.t3 {
            let dt3 = self.t3 - t;
            SubTarget {
                j: -self.j_m,
                a: self.j_m * dt3,
                v: self.v_end - self.j_m * dt3 * dt3 / 2.0,
                x: self.x_end - self.v_end * dt3 + self.j_m * dt3 * dt3 * dt3 / 6.0,
            }
        } else {
            SubTarget {
                x: self.x_end,
                v: self.v_end,
                a: Default::default(),
                j: Default::default(),
            }
        }
    }
}

#[derive(Clone)]
struct ConstantCalculator<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable,
{
    t_end: Time,
    x_start: T,
    v: <T as Div<Time>>::Output,
}

impl<T> ConstantCalculator<T>
where
    T: TimeDifferentiable + Div<T, Output = f32>,
    dt!(T): TimeDifferentiable + Mul<Time, Output = T> + Div<dddt!(T), Output = SquaredTime>,
    ddt!(T): TimeDifferentiable + Mul<Time, Output = dt!(T)>,
    dddt!(T): Quantity + Mul<Time, Output = ddt!(T)>,
    f32: From<T> + From<dt!(T)> + From<ddt!(T)> + From<dddt!(T)>,
{
    fn calculate(&self, t: Time) -> SubTarget<T> {
        if t > self.t_end || t < Default::default() {
            SubTarget {
                x: self.x_start + self.v * self.t_end,
                v: self.v,
                a: Default::default(),
                j: Default::default(),
            }
        } else {
            SubTarget {
                j: Default::default(),
                a: Default::default(),
                v: self.v,
                x: self.x_start + self.v * t,
            }
        }
    }
}

#[derive(Clone)]
pub struct OverallCalculator<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable + Clone,
    ddt!(T): TimeDifferentiable,
    dddt!(T): Quantity,
{
    x_start: T,
    v_start: <T as Div<Time>>::Output,
    t1: Time,
    t2: Time,
    t3: Time,
    distance: T,
    v_end: <T as Div<Time>>::Output,
    accel_calculator: AccelerationCalculator<T>,
    const_calculator: ConstantCalculator<T>,
    decel_calculator: AccelerationCalculator<T>,
    sign: f32,
}

impl<T> OverallCalculator<T>
where
    T: TimeDifferentiable + Div<T, Output = f32>,
    dt!(T): TimeDifferentiable + Mul<Time, Output = T> + Div<dddt!(T), Output = SquaredTime>,
    ddt!(T): TimeDifferentiable + Mul<Time, Output = dt!(T)>,
    dddt!(T): Quantity + Mul<Time, Output = ddt!(T)>,
    f32: From<T> + From<dt!(T)> + From<ddt!(T)> + From<dddt!(T)>,
{
    pub fn calculate(&self, t: Time) -> SubTarget<T> {
        let target = if t < Default::default() {
            SubTarget {
                x: self.x_start,
                v: self.v_start,
                a: Default::default(),
                j: Default::default(),
            }
        } else if t <= self.t1 {
            self.accel_calculator.calculate(t)
        } else if t <= self.t2 {
            self.const_calculator.calculate(t - self.t1)
        } else if t <= self.t3 {
            self.decel_calculator.calculate(t - self.t2)
        } else {
            SubTarget {
                x: self.x_start + self.distance,
                v: self.v_end,
                a: Default::default(),
                j: Default::default(),
            }
        };
        SubTarget {
            x: (target.x - self.x_start) * self.sign + self.x_start,
            v: target.v * self.sign,
            a: target.a * self.sign,
            j: target.j * self.sign,
        }
    }
}

pub struct StraightTrajectoryGenerator {
    function_generator: StraightCalculatorGenerator<Distance>,
    period: Time,
}

impl StraightTrajectoryGenerator {
    pub fn new(v_max: Speed, a_max: Acceleration, j_max: Jerk, period: Time) -> Self {
        Self {
            function_generator: StraightCalculatorGenerator::new(v_max, a_max, j_max),
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
    ) -> StraightTrajectory {
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
}

#[derive(Clone)]
pub struct StraightTrajectory {
    trajectory_calculator: OverallCalculator<Distance>,
    t: Time,
    t_end: Time,
    period: Time,
    x_ratio: f32,
    y_ratio: f32,
    x_start: Distance,
    y_start: Distance,
    theta: Angle,
}

impl StraightTrajectory {
    fn new(
        trajectory_calculator: OverallCalculator<Distance>,
        t_end: Time,
        period: Time,
        x_ratio: f32,
        y_ratio: f32,
        x_start: Distance,
        y_start: Distance,
        theta: Angle,
    ) -> Self {
        Self {
            trajectory_calculator,
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

impl Iterator for StraightTrajectory {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        if self.t > self.t_end {
            return None;
        }
        let t = self.t;
        self.t += self.period;
        let target = self.trajectory_calculator.calculate(t);
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
    use quantities::{Acceleration, Distance, Jerk, Speed};

    const EPSILON: f32 = 1e-2; //low accuracy...

    use proptest::prelude::*;

    macro_rules! straight_calculator_tests {
        ($($test_name: ident: $type: ty,)*) => {
            $(
                proptest!{
                    #[ignore]
                    #[test]
                    fn $test_name(
                        a_max in 0.5f32..500.0f32,
                        j_max in 0.5f32..1000.0f32,
                        x_start in 0.0f32..288.0f32,
                        distance in 0.01f32..288.0f32,
                        (v_max, v_start, v_end) in (0.5f32..100.0f32)
                            .prop_flat_map(|v_max| (Just(v_max), 0.0..v_max, 0.0..v_max)),
                    ) {
                        let v_max = <dt!($type)>::from(v_max);
                        let a_max = <ddt!($type)>::from(a_max);
                        let j_max = <dddt!($type)>::from(j_max);
                        let period = Time::from_seconds(0.001);
                        let generator = StraightCalculatorGenerator::new(v_max, a_max, j_max);
                        let (trajectory_calculator, t_end) = generator.generate(
                            <$type>::from(x_start),
                            <$type>::from(distance),
                            <dt!($type)>::from(v_start),
                            <dt!($type)>::from(v_end),
                        );
                        let mut current = Time::from_seconds(0.0);
                        let mut before = trajectory_calculator.calculate(current);

                        let epst = <$type>::from(EPSILON);
                        let epsdt = <dt!($type)>::from(EPSILON);
                        let epsddt = <ddt!($type)>::from(EPSILON);

                        while current <= t_end {
                            let target = trajectory_calculator.calculate(current);

                            let xd = (target.x - before.x).abs();
                            let vd = (target.v - before.v).abs();
                            let ad = (target.a - before.a).abs();

                            prop_assert!(
                                xd <= v_max * period + epst,
                                "left:{:?}, right:{:?}", xd, v_max * period + epst,
                            );
                            prop_assert!(
                                vd <= a_max * period + epsdt,
                                "left:{:?}, right:{:?}", vd, a_max * period + epsdt,
                            );
                            prop_assert!(
                                ad <= j_max * period + epsddt,
                                "left:{:?}, right:{:?}", ad, j_max * period + epsddt,
                            );

                            before = target;
                            current += period;
                        }
                        prop_assert!((f32::from(before.x) - x_start).abs() <= distance + EPSILON);
                    }
                }
            )*
        }
    }

    straight_calculator_tests! {
        test_straight_calculator_distance: Distance,
        test_straight_calculator_angle: Angle,
    }

    proptest! {
        #[ignore]
        #[test]
        fn test_straight_trajectory(
            x_start in 0.0f32..288.0f32,
            y_start in 0.0f32..288.0f32,
            x_end in 0.0f32..288.0f32,
            y_end in 0.0f32..288.0f32,
            start_speed in 0.0f32..2.0f32,
            end_speed in 0.0f32..2.0f32,
        ) {
            let period = Time::from_seconds(0.001);
            let v_max = Speed::from_meter_per_second(2.0);
            let a_max = Acceleration::from_meter_per_second_squared(1.0);
            let j_max = Jerk::from_meter_per_second_cubed(0.5);
            let generator = StraightTrajectoryGenerator::new(v_max, a_max, j_max, period);
            let mut trajectory = generator.generate(
                Distance::from_meters(x_start),
                Distance::from_meters(y_start),
                Distance::from_meters(x_end),
                Distance::from_meters(y_end),
                Speed::from_meter_per_second(start_speed),
                Speed::from_meter_per_second(end_speed),
            );

            let mut before = trajectory.next().unwrap();
            for target in trajectory {
                let cos = target.theta.x.cos();
                let sin = target.theta.x.sin();

                let xd = ((target.x.x-before.x.x) * cos + (target.y.x-before.y.x) * sin).abs();
                let vd = ((target.x.v-before.x.v) * cos + (target.y.v-before.y.v) * sin).abs();
                let ad = ((target.x.a-before.x.a) * cos + (target.y.a-before.y.a) * sin).abs();

                prop_assert!(xd <= v_max * period + Distance::from_meters(EPSILON));
                prop_assert!(vd <= a_max * period + Speed::from_meter_per_second(EPSILON));
                prop_assert!(ad <= j_max * period + Acceleration::from_meter_per_second_squared(EPSILON));

                before = target;
            }
        }
    }
}
