use core::marker::PhantomData;

use uom::si::{
    angle::radian,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Time,
        Velocity,
    },
    length::meter,
    time::second,
};

use super::trajectory::{AngleTarget, LengthTarget, MoveTarget, Target};
use crate::traits::Math;

macro_rules! impl_calculator_generator {
    ($mod_name: ident, $t: ty, $dt: ty, $ddt: ty, $dddt: ty, $tunit: ty, $target: ident) => {
        mod $mod_name {
            use super::*;

            pub struct StraightCalculatorGenerator<M> {
                v_max: $dt,
                a_max: $ddt,
                j_max: $dddt,
                _phantom: PhantomData<fn() -> M>,
            }

            impl<M> StraightCalculatorGenerator<M> {
                const LOOP_COUNT: u8 = 25;

                pub fn new(v_max: $dt, a_max: $ddt, j_max: $dddt) -> Self {
                    Self {
                        v_max,
                        a_max,
                        j_max,
                        _phantom: PhantomData,
                    }
                }
            }

            impl<M> StraightCalculatorGenerator<M>
            where
                M: Math,
            {
                #[inline]
                #[allow(unused)]
                pub fn generate(
                    &self,
                    x_start: $t,
                    distance: $t,
                    v_start: $dt,
                    v_end: $dt,
                ) -> (OverallCalculator, Time) {
                    let (calculator, t_end, _) =
                        self.generate_with_terminal_velocity(x_start, distance, v_start, v_end);
                    (calculator, t_end)
                }

                //NOTO: v_start and v_end should not be negative
                pub fn generate_with_terminal_velocity(
                    &self,
                    x_start: $t,
                    distance: $t,
                    v_start: $dt,
                    v_end: $dt,
                ) -> (OverallCalculator, Time, $dt) {
                    let (distance, sign) = if distance.get::<$tunit>() < 0.0 {
                        (-distance, -1.0f32)
                    } else {
                        (distance, 1.0f32)
                    };
                    let v_reach = self.calculate_reachable_velocity(v_start, v_end, distance);
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

                    let (accel_calculator, dt1) =
                        self.generate_acceleration(x_start, v_start, v_max);
                    let (const_calculator, dt2) =
                        Self::generate_constant(x_start + dist1, dist2, v_max);
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
                        v_reach,
                    )
                }

                fn generate_acceleration(
                    &self,
                    x_start: $t,
                    v_start: $dt,
                    v_end: $dt,
                ) -> (AccelerationCalculator, Time) {
                    let tc = self.a_max / self.j_max;
                    let vd = <$dt>::from(self.a_max * tc);
                    let (t1, t2, t3) = if (v_end - v_start).abs() >= vd {
                        let tm = (v_end - v_start).abs() / self.a_max - tc;
                        (tc, tc + tm, 2.0 * tc + tm)
                    } else {
                        let td = M::sqrt((v_end - v_start).abs() / self.j_max);
                        (td, td, 2.0 * td)
                    };

                    let (a_m, j_m) = if v_start <= v_end {
                        (self.a_max, self.j_max)
                    } else {
                        (-self.a_max, -self.j_max)
                    };

                    let distance = <$t>::from((v_start + v_end) * t3) / 2.0;
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

                fn calculate_acceleration_distance(&self, v_start: $dt, v_end: $dt) -> $t {
                    let tc = self.a_max / self.j_max;
                    let vd = <$dt>::from(self.a_max * tc);
                    let t = if (v_end - v_start).abs() >= vd {
                        (v_end - v_start).abs() / self.a_max + tc
                    } else {
                        2.0 * M::sqrt((v_end - v_start).abs() / self.j_max)
                    };
                    <$t>::from((v_start + v_end) * t / 2.0)
                }

                pub fn generate_constant(
                    x_start: $t,
                    distance: $t,
                    v: $dt,
                ) -> (ConstantCalculator, Time) {
                    let t_end = if (distance / v).get::<second>() < 0.0 {
                        Default::default()
                    } else {
                        distance / v
                    };
                    (ConstantCalculator { t_end, x_start, v }, t_end)
                }

                //TODO: devise faster algorithm
                pub fn calculate_reachable_velocity(
                    &self,
                    v_start: $dt,
                    v_end: $dt,
                    distance: $t,
                ) -> $dt {
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
            struct AccelerationCalculator {
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

            impl AccelerationCalculator {
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
                            a: <$ddt>::from(self.j_m * t),
                            v: self.v_start + <$dt>::from(self.j_m * t * t / 2.0),
                            x: self.x_start
                                + <$t>::from(self.v_start * t + self.j_m * t * t * t / 6.0),
                        }
                    } else if t <= self.t2 {
                        let dt1 = t - self.t1;
                        $target {
                            j: Default::default(),
                            a: self.a_m,
                            v: self.v_start
                                + <$dt>::from(self.a_m * self.t1 / 2.0 + self.a_m * dt1),
                            x: self.x_start
                                + <$t>::from(
                                    self.v_start * self.t1
                                        + self.j_m * self.t1 * self.t1 * self.t1 / 6.0
                                        + (self.v_start + <$dt>::from(self.a_m * self.t1) / 2.0)
                                            * dt1
                                        + self.a_m * dt1 * dt1 / 2.0,
                                ),
                        }
                    } else if t <= self.t3 {
                        let dt3 = self.t3 - t;
                        $target {
                            j: -self.j_m,
                            a: <$ddt>::from(self.j_m * dt3),
                            v: self.v_end - <$dt>::from(self.j_m * dt3 * dt3 / 2.0),
                            x: self.x_end - <$t>::from(self.v_end * dt3)
                                + <$t>::from(self.j_m * dt3 * dt3 * dt3 / 6.0),
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
            pub struct ConstantCalculator {
                t_end: Time,
                x_start: $t,
                v: $dt,
            }

            impl ConstantCalculator {
                pub fn calculate(&self, t: Time) -> $target {
                    if t > self.t_end || t.get::<second>() < 0.0 {
                        $target {
                            x: self.x_start + <$t>::from(self.v * self.t_end),
                            v: self.v,
                            a: Default::default(),
                            j: Default::default(),
                        }
                    } else {
                        $target {
                            j: Default::default(),
                            a: Default::default(),
                            v: self.v,
                            x: self.x_start + <$t>::from(self.v * t),
                        }
                    }
                }
            }

            #[derive(Clone)]
            pub struct OverallCalculator {
                x_start: $t,
                v_start: $dt,
                t1: Time,
                t2: Time,
                t3: Time,
                distance: $t,
                v_end: $dt,
                accel_calculator: AccelerationCalculator,
                const_calculator: ConstantCalculator,
                decel_calculator: AccelerationCalculator,
                sign: f32,
            }

            impl OverallCalculator {
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
        }
    };
}

pub use length_calculator::{
    ConstantCalculator as LengthConstantCalculator, OverallCalculator as LengthOverallCalculator,
    StraightCalculatorGenerator as LengthStraightCalculatorGenerator,
};
impl_calculator_generator!(
    length_calculator,
    Length,
    Velocity,
    Acceleration,
    Jerk,
    meter,
    LengthTarget
);

pub use angle_calculator::{
    OverallCalculator as AngleOverallCalculator,
    StraightCalculatorGenerator as AngleStraightCalculatorGenerator,
};
impl_calculator_generator!(
    angle_calculator,
    Angle,
    AngularVelocity,
    AngularAcceleration,
    AngularJerk,
    radian,
    AngleTarget
);

pub struct StraightTrajectoryGenerator<M> {
    function_generator: LengthStraightCalculatorGenerator<M>,
    period: Time,
}

impl<M> StraightTrajectoryGenerator<M> {
    pub fn new(v_max: Velocity, a_max: Acceleration, j_max: Jerk, period: Time) -> Self {
        Self {
            function_generator: LengthStraightCalculatorGenerator::new(v_max, a_max, j_max),
            period,
        }
    }
}

impl<M> StraightTrajectoryGenerator<M>
where
    M: Math,
{
    //NOTO: v_start and v_end should not be negative
    #[inline]
    pub fn generate(
        &self,
        distance: Length,
        v_start: Velocity,
        v_end: Velocity,
    ) -> StraightTrajectory {
        let (trajectory, _) = self.generate_with_terminal_velocity(distance, v_start, v_end);
        trajectory
    }

    pub fn generate_with_terminal_velocity(
        &self,
        distance: Length,
        v_start: Velocity,
        v_end: Velocity,
    ) -> (StraightTrajectory, Velocity) {
        let (trajectory_fn, t_end, terminal_velocity) = self
            .function_generator
            .generate_with_terminal_velocity(Default::default(), distance, v_start, v_end);

        (
            StraightTrajectory::new(
                StraightTrajectoryCalculator::Accel(trajectory_fn),
                t_end,
                self.period,
            ),
            terminal_velocity,
        )
    }

    pub fn generate_constant(distance: Length, v: Velocity, period: Time) -> StraightTrajectory {
        let (trajectory_fn, t_end) = LengthStraightCalculatorGenerator::<M>::generate_constant(
            Default::default(),
            distance,
            v,
        );

        StraightTrajectory::new(
            StraightTrajectoryCalculator::Constant(trajectory_fn),
            t_end,
            period,
        )
    }

    #[allow(unused)]
    pub fn reachable_velocity(
        &self,
        distance: Length,
        v_start: Velocity,
        v_end: Velocity,
    ) -> Velocity {
        self.function_generator
            .calculate_reachable_velocity(v_start, v_end, distance)
    }
}

#[derive(Clone)]
pub enum StraightTrajectoryCalculator {
    Accel(LengthOverallCalculator),
    Constant(LengthConstantCalculator),
}

#[derive(Clone)]
pub struct StraightTrajectory {
    trajectory_calculator: StraightTrajectoryCalculator,
    t: Time,
    t_end: Time,
    period: Time,
}

impl StraightTrajectory {
    fn new(trajectory_calculator: StraightTrajectoryCalculator, t_end: Time, period: Time) -> Self {
        Self {
            trajectory_calculator,
            t: Default::default(),
            t_end,
            period,
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
        let target = match &self.trajectory_calculator {
            StraightTrajectoryCalculator::Accel(calculator) => calculator.calculate(t),
            StraightTrajectoryCalculator::Constant(calculator) => calculator.calculate(t),
        };
        Some(Target::Moving(MoveTarget {
            x: target,
            ..Default::default()
        }))
    }

    fn advance_by(&mut self, n: usize) -> Result<(), usize> {
        self.t += self.period * n as f32;
        if self.t < self.t_end {
            Ok(())
        } else {
            Err(n - ((self.t - self.t_end) / self.period).get::<uom::si::ratio::ratio>() as usize)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::math::MathFake;
    use uom::si::{
        acceleration::meter_per_second_squared, angle::radian,
        angular_acceleration::radian_per_second_squared, angular_jerk::radian_per_second_cubed,
        angular_velocity::radian_per_second, jerk::meter_per_second_cubed, length::meter,
        time::second, velocity::meter_per_second,
    };

    const EPSILON: f32 = 1e-2; //low accuracy...

    use proptest::prelude::*;

    macro_rules! define_straight_calculator_test {
        (
            $test_name: ident,
            $generator_name: ident: (
                $t: ty,
                $dt: ty,
                $ddt: ty,
                $dddt: ty,
                $tunit: ty,
                $dtunit: ty,
                $ddtunit: ty,
                $dddtunit: ty,
            ),
        ) => {
            proptest! {
                #[ignore]
                #[test]
                fn $test_name(
                    a_max in 0.5f32..1.00f32,
                    j_max in 0.5f32..10.0f32,
                    x_start in 0.0f32..2.88f32,
                    distance in 0.01f32..2.88f32,
                    period in 0.001f32..0.01f32,
                    (v_max, v_start, v_end) in (0.5f32..1.0f32)
                        .prop_flat_map(|v_max| (Just(v_max), 0.0..v_max, 0.0..v_max)),
                ) {
                    let v_max = <$dt>::new::<$dtunit>(v_max);
                    let a_max = <$ddt>::new::<$ddtunit>(a_max);
                    let j_max = <$dddt>::new::<$dddtunit>(j_max);
                    let period = Time::new::<second>(period);
                    let generator = $generator_name::<MathFake>::new(v_max, a_max, j_max);
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

                        let dx_max = <$t>::from(v_max * period) + epst;
                        let dv_max = <$dt>::from(a_max * period) + epsdt;
                        let da_max = <$ddt>::from(j_max * period) + epsddt;

                        prop_assert!(
                            xd <= dx_max,
                            "left:{:?}, right:{:?}", xd, dx_max,
                        );
                        prop_assert!(
                            vd <= dv_max,
                            "left:{:?}, right:{:?}", vd, dv_max,
                        );
                        prop_assert!(
                            ad <= da_max,
                            "left:{:?}, right:{:?}", ad, da_max,
                        );

                        before = target;
                        current += period;
                    }
                    prop_assert!((before.x.get::<$tunit>() - x_start).abs() <= distance + EPSILON);
                }
            }
        };
    }

    define_straight_calculator_test!(
        test_length_straight_calculator,
        LengthStraightCalculatorGenerator:
            (
                Length,
                Velocity,
                Acceleration,
                Jerk,
                meter,
                meter_per_second,
                meter_per_second_squared,
                meter_per_second_cubed,
            ),
    );
    define_straight_calculator_test!(
        test_angle_straight_calculator,
        AngleStraightCalculatorGenerator:
            (
                Angle,
                AngularVelocity,
                AngularAcceleration,
                AngularJerk,
                radian,
                radian_per_second,
                radian_per_second_squared,
                radian_per_second_cubed,
            ),
    );

    proptest! {
        #[ignore]
        #[test]
        fn test_straight_trajectory(
            distance in 0.0f32..400.0f32,
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
            let generator = StraightTrajectoryGenerator::<MathFake>::new(v_max, a_max, j_max, period);
            let mut trajectory = generator.generate(
                Length::new::<meter>(distance),
                Velocity::new::<meter_per_second>(v_start),
                Velocity::new::<meter_per_second>(v_end),
            );

            let mut before = trajectory.next().unwrap().moving().unwrap();
            for target in trajectory {
                let target = target.moving().unwrap();
                let cos = target.theta.x.get::<radian>().cos();
                let sin = target.theta.x.get::<radian>().sin();

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
