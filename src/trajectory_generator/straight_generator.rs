use super::trajectory::{AngleTarget, LengthTarget, Target};
use crate::quantities::f32::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Time,
    Velocity,
};
use crate::quantities::{
    dimensionless::{radian, scalar},
    length::meter,
    time::second,
};

macro_rules! impl_calculator_generator {
    (
        $generator_name: ident,
        $overall_name: ident,
        $accel_name: ident,
        $constant_name: ident:
        $t: ty,
        $dt: ty,
        $ddt: ty,
        $dddt: ty,
        $tunit: ty,
        $target: ident,
    ) => {
        pub struct $generator_name {
            v_max: $dt,
            a_max: $ddt,
            j_max: $dddt,
        }

        impl $generator_name {
            const LOOP_COUNT: u8 = 25;

            pub fn new(v_max: $dt, a_max: $ddt, j_max: $dddt) -> Self {
                Self {
                    v_max,
                    a_max,
                    j_max,
                }
            }

            //NOTO: v_start and v_end should not be negative
            pub fn generate(
                &self,
                x_start: $t,
                distance: $t,
                v_start: $dt,
                v_end: $dt,
            ) -> ($overall_name, Time) {
                let (distance, sign) = if distance.get::<$tunit>() < 0.0 {
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
                let (const_calculator, dt2) =
                    Self::generate_constant(x_start + dist1, dist2, v_max);
                let (decel_calculator, dt3) =
                    self.generate_acceleration(x_start + distance - dist3, v_max, v_end);

                let t1 = dt1;
                let t2 = t1 + dt2;
                let t3 = t2 + dt3;

                (
                    $overall_name {
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
                x_start: $t,
                v_start: $dt,
                v_end: $dt,
            ) -> ($accel_name, Time) {
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
                    $accel_name {
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

            fn calculate_acceleration_distance(&self, v_start: $dt, v_end: $dt) -> $t {
                let tc = self.a_max / self.j_max;
                let vd = self.a_max * tc;
                let t = if (v_end - v_start).abs() >= vd {
                    (v_end - v_start).abs() / self.a_max + tc
                } else {
                    2.0 * ((v_end - v_start).abs() / self.j_max).sqrt()
                };
                (v_start + v_end) * t / 2.0
            }

            fn generate_constant(x_start: $t, distance: $t, v: $dt) -> ($constant_name, Time) {
                let t_end = if (distance / v).get::<second>() < 0.0 {
                    Default::default()
                } else {
                    distance / v
                };
                ($constant_name { t_end, x_start, v }, t_end)
            }

            //TODO: devise faster algorithm
            pub fn calculate_reachable_speed(&self, v_start: $dt, v_end: $dt, distance: $t) -> $dt {
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
        struct $accel_name {
            x_start: $t,
            v_start: $dt,
            x_end: $t,
            v_end: $dt,
            a_m: $ddt,
            j_m: $dddt,
            t1: Time,
            t2: Time,
            t3: Time,
        }

        impl $accel_name {
            fn calculate(&self, t: Time) -> $target {
                if t.get::<second>() < 0.0 {
                    $target {
                        x: self.x_start,
                        v: self.v_start,
                        a: Default::default(),
                        j: Default::default(),
                    }
                } else if t <= self.t1 {
                    $target {
                        j: self.j_m,
                        a: self.j_m * t,
                        v: self.v_start + self.j_m * t * t / 2.0,
                        x: self.x_start + self.v_start * t + self.j_m * t * t * t / 6.0,
                    }
                } else if t <= self.t2 {
                    let dt1 = t - self.t1;
                    $target {
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
                    $target {
                        j: -self.j_m,
                        a: self.j_m * dt3,
                        v: self.v_end - self.j_m * dt3 * dt3 / 2.0,
                        x: self.x_end - self.v_end * dt3 + self.j_m * dt3 * dt3 * dt3 / 6.0,
                    }
                } else {
                    $target {
                        x: self.x_end,
                        v: self.v_end,
                        a: Default::default(),
                        j: Default::default(),
                    }
                }
            }
        }

        #[derive(Clone)]
        struct $constant_name {
            t_end: Time,
            x_start: $t,
            v: $dt,
        }

        impl $constant_name {
            fn calculate(&self, t: Time) -> $target {
                if t > self.t_end || t.get::<second>() < 0.0 {
                    $target {
                        x: self.x_start + self.v * self.t_end,
                        v: self.v,
                        a: Default::default(),
                        j: Default::default(),
                    }
                } else {
                    $target {
                        j: Default::default(),
                        a: Default::default(),
                        v: self.v,
                        x: self.x_start + self.v * t,
                    }
                }
            }
        }

        #[derive(Clone)]
        pub struct $overall_name {
            x_start: $t,
            v_start: $dt,
            t1: Time,
            t2: Time,
            t3: Time,
            distance: $t,
            v_end: $dt,
            accel_calculator: $accel_name,
            const_calculator: $constant_name,
            decel_calculator: $accel_name,
            sign: f32,
        }

        impl $overall_name {
            pub fn calculate(&self, t: Time) -> $target {
                let target = if t.get::<second>() < 0.0 {
                    $target {
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
                    $target {
                        x: self.x_start + self.distance,
                        v: self.v_end,
                        a: Default::default(),
                        j: Default::default(),
                    }
                };
                $target {
                    x: (target.x - self.x_start) * self.sign + self.x_start,
                    v: target.v * self.sign,
                    a: target.a * self.sign,
                    j: target.j * self.sign,
                }
            }
        }
    };
}

impl_calculator_generator!(
    LengthStraightCalculatorGenerator,
    LengthOverallCalculator,
    LengthAccelerationCalculator,
    LengthConstantCalculator: Length,
    Velocity,
    Acceleration,
    Jerk,
    meter,
    LengthTarget,
);

impl_calculator_generator!(
    AngleStraightCalculatorGenerator,
    AngleOverallCalculator,
    AnglerAccelerationCalculator,
    AnglerConstantCalculator: Angle,
    AngularVelocity,
    AngularAcceleration,
    AngularJerk,
    radian,
    AngleTarget,
);

pub struct StraightTrajectoryGenerator {
    function_generator: LengthStraightCalculatorGenerator,
    period: Time,
}

impl StraightTrajectoryGenerator {
    pub fn new(v_max: Velocity, a_max: Acceleration, j_max: Jerk, period: Time) -> Self {
        Self {
            function_generator: LengthStraightCalculatorGenerator::new(v_max, a_max, j_max),
            period,
        }
    }
}

impl StraightTrajectoryGenerator {
    fn calculate_distance(x_dist: Length, y_dist: Length) -> Length {
        let x_dist_raw = x_dist.get::<meter>();
        let y_dist_raw = y_dist.get::<meter>();

        Length::new::<meter>(libm::sqrtf(
            x_dist_raw * x_dist_raw + y_dist_raw * y_dist_raw,
        ))
    }

    //NOTO: v_start and v_end should not be negative
    pub fn generate(
        &self,
        x_start: Length,
        y_start: Length,
        x_end: Length,
        y_end: Length,
        v_start: Velocity,
        v_end: Velocity,
    ) -> StraightTrajectory {
        let x_dist = x_end - x_start;
        let y_dist = y_end - y_start;

        let dist = Self::calculate_distance(x_dist, y_dist);

        let (trajectory_fn, t_end) =
            self.function_generator
                .generate(Default::default(), dist, v_start, v_end);

        let x_ratio = (x_dist / dist).get::<scalar>();
        let y_ratio = (y_dist / dist).get::<scalar>();

        let theta =
            Angle::new::<radian>(libm::atan2f(y_dist.get::<meter>(), x_dist.get::<meter>()));

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
    trajectory_calculator: LengthOverallCalculator,
    t: Time,
    t_end: Time,
    period: Time,
    x_ratio: f32,
    y_ratio: f32,
    x_start: Length,
    y_start: Length,
    theta: Angle,
}

impl StraightTrajectory {
    fn new(
        trajectory_calculator: LengthOverallCalculator,
        t_end: Time,
        period: Time,
        x_ratio: f32,
        y_ratio: f32,
        x_start: Length,
        y_start: Length,
        theta: Angle,
    ) -> Self {
        Self {
            trajectory_calculator,
            t: Time::new::<second>(0.0),
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
            x: LengthTarget {
                x: self.x_start + target.x * self.x_ratio,
                v: target.v * self.x_ratio,
                a: target.a * self.x_ratio,
                j: target.j * self.x_ratio,
            },
            y: LengthTarget {
                x: self.y_start + target.x * self.y_ratio,
                v: target.v * self.y_ratio,
                a: target.a * self.y_ratio,
                j: target.j * self.y_ratio,
            },
            theta: AngleTarget {
                x: self.theta,
                ..Default::default()
            },
        })
    }
}

#[cfg(feature = "expensive_tests")]
#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantities::{
        acceleration::meter_per_second_squared, cubed_frequency::radian_per_second_cubed,
        dimensionless::radian, frequency::radian_per_second, jerk::meter_per_second_cubed,
        length::meter, squared_frequency::radian_per_second_squared, time::second,
        velocity::meter_per_second,
    };

    const EPSILON: f32 = 1e-2; //low accuracy...

    use proptest::prelude::*;

    macro_rules! straight_calculator_tests {
        ($(
            $test_name: ident,
            $generator_name: ident: $t: ty,
            $dt: ty,
            $ddt: ty,
            $dddt: ty,
            $tunit: ty,
            $dtunit: ty,
            $ddtunit: ty,
            $dddtunit: ty,
        )*) => {
            $(
                proptest!{
                    #[test]
                    fn $test_name(
                        a_max in 0.5f32..500.0f32,
                        j_max in 0.5f32..1000.0f32,
                        x_start in 0.0f32..288.0f32,
                        distance in 0.01f32..288.0f32,
                        period in 0.001f32..0.01f32,
                        (v_max, v_start, v_end) in (0.5f32..100.0f32)
                            .prop_flat_map(|v_max| (Just(v_max), 0.0..v_max, 0.0..v_max)),
                    ) {
                        let v_max = <$dt>::new::<$dtunit>(v_max);
                        let a_max = <$ddt>::new::<$ddtunit>(a_max);
                        let j_max = <$dddt>::new::<$dddtunit>(j_max);
                        let period = Time::new::<second>(period);
                        let generator = $generator_name::new(v_max, a_max, j_max);
                        let (trajectory_calculator, t_end) = generator.generate(
                            <$t>::new::<$tunit>(x_start),
                            <$t>::new::<$tunit>(distance),
                            <$dt>::new::<$dtunit>(v_start),
                            <$dt>::new::<$dtunit>(v_end),
                        );
                        let mut current = Time::new::<second>(0.0);
                        let mut before = trajectory_calculator.calculate(current);

                        let epst = <$t>::new::<$tunit>(EPSILON);
                        let epsdt = <$dt>::new::<$dtunit>(EPSILON);
                        let epsddt = <$ddt>::new::<$ddtunit>(EPSILON);

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
                        prop_assert!((before.x.get::<$tunit>() - x_start).abs() <= distance + EPSILON);
                    }
                }
            )*
        }
    }

    straight_calculator_tests! {
        test_length_straight_calculator,
        LengthStraightCalculatorGenerator: Length,
        Velocity,
        Acceleration,
        Jerk,
        meter,
        meter_per_second,
        meter_per_second_squared,
        meter_per_second_cubed,
        test_angle_straight_calculator,
        AngleStraightCalculatorGenerator: Angle,
        AngularVelocity,
        AngularAcceleration,
        AngularJerk,
        radian,
        radian_per_second,
        radian_per_second_squared,
        radian_per_second_cubed,
    }

    proptest! {
        #[test]
        fn test_straight_trajectory(
            x_start in 0.0f32..288.0f32,
            y_start in 0.0f32..288.0f32,
            x_end in 0.0f32..288.0f32,
            y_end in 0.0f32..288.0f32,
            (v_max, v_start, v_end) in (0.5f32..10.0f32)
                .prop_flat_map(|v_max| (Just(v_max), 0.0..v_max, 0.0..v_max)),
            a_max in 0.5f32..10.0f32,
            j_max in 0.5f32..10.0f32,
            period in 0.001f32..0.01f32,
        ) {
            let period = Time::new::<second>(period);
            let v_max = Velocity::new::<meter_per_second>(v_max);
            let a_max = Acceleration::new::<meter_per_second_squared>(a_max);
            let j_max = Jerk::new::<meter_per_second_cubed>(j_max);
            let generator = StraightTrajectoryGenerator::new(v_max, a_max, j_max, period);
            let mut trajectory = generator.generate(
                Length::new::<meter>(x_start),
                Length::new::<meter>(y_start),
                Length::new::<meter>(x_end),
                Length::new::<meter>(y_end),
                Velocity::new::<meter_per_second>(v_start),
                Velocity::new::<meter_per_second>(v_end),
            );

            let mut before = trajectory.next().unwrap();
            for target in trajectory {
                let cos = target.theta.x.cos();
                let sin = target.theta.x.sin();

                let xd = ((target.x.x-before.x.x) * cos + (target.y.x-before.y.x) * sin).abs();
                let vd = ((target.x.v-before.x.v) * cos + (target.y.v-before.y.v) * sin).abs();
                let ad = ((target.x.a-before.x.a) * cos + (target.y.a-before.y.a) * sin).abs();

                prop_assert!(xd <= v_max * period + Length::new::<meter>(EPSILON));
                prop_assert!(vd <= a_max * period + Velocity::new::<meter_per_second>(EPSILON));
                prop_assert!(ad <= j_max * period + Acceleration::new::<meter_per_second_squared>(EPSILON));

                before = target;
            }
        }
    }
}
