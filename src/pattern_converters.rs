//! Implementations for [PatternConverter](crate::mazes::PatternConverter).

use heapless::Vec;
use serde::{Deserialize, Serialize};

use crate::mazes::PatternConverter;
use crate::types::data::Pattern;
use crate::{impl_deconstruct_with_default, Construct};
use crate::{impl_with_builder, Deconstruct};

impl_with_builder! {
    /// A builder for [LinearPatternConverter].
    LinearPatternConverterBuilder:
    {
        /// An implementation of [PatternConverter](PatternConverter).
        ///
        /// This calculates cost of straight pattern by multiplication with a given multiplier.
        #[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
        pub struct LinearPatternConverter<Cost> {
            search90: Cost,
            fast_run45: Cost,
            fast_run90: Cost,
            fast_run135: Cost,
            fast_run180: Cost,
            fast_run_diagonal90: Cost,
            spin_back: Cost,
            straight_multiplier: Cost,
            straight_diagonal_multiplier: Cost,
        }
    }
}

impl Default for LinearPatternConverter<u16> {
    fn default() -> Self {
        Self {
            search90: 8,
            fast_run45: 12,
            fast_run90: 15,
            fast_run135: 20,
            fast_run180: 25,
            fast_run_diagonal90: 15,
            spin_back: 15,
            straight_multiplier: 10,
            straight_diagonal_multiplier: 7,
        }
    }
}

impl<Config, State, Resource, Cost> Construct<Config, State, Resource>
    for LinearPatternConverter<Cost>
where
    Cost: Clone,
    Config: AsRef<LinearPatternConverter<Cost>>,
{
    fn construct<'a>(config: &'a Config, _state: &'a State, _resource: &'a mut Resource) -> Self {
        let converter = config.as_ref();
        converter.clone()
    }
}

impl_deconstruct_with_default!(LinearPatternConverter<Cost>);

impl<Cost> PatternConverter<Pattern> for LinearPatternConverter<Cost>
where
    u8: Into<Cost>,
    Cost: core::ops::Mul<Output = Cost> + Copy,
{
    type Cost = Cost;

    fn convert(&self, pattern: &Pattern) -> Self::Cost {
        use Pattern::*;

        match pattern {
            &Straight(x) => x.into() * self.straight_multiplier,
            &StraightDiagonal(x) => x.into() * self.straight_diagonal_multiplier,
            Search90 => self.search90,
            FastRun45 => self.fast_run45,
            FastRun90 => self.fast_run90,
            FastRun135 => self.fast_run135,
            FastRun180 => self.fast_run180,
            FastRunDiagonal90 => self.fast_run_diagonal90,
            SpinBack => self.spin_back,
        }
    }
}

impl_with_builder! {
    /// A builder for [DefaultPatternConverter].
    DefaultPatternConverterBuilder:
    {
        /// An implementation of [PatternConverter].
        ///
        /// The costs of straight and straight diagonal can be determined for each length.
        #[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
        pub struct DefaultPatternConverter<Cost; const N: usize> {
            search90: Cost,
            fast_run45: Cost,
            fast_run90: Cost,
            fast_run135: Cost,
            fast_run180: Cost,
            fast_run_diagonal90: Cost,
            spin_back: Cost,
            straight_array: Vec<Cost, N>,
            straight_diagonal_array: Vec<Cost, N>,
        }
    }
}

impl<Cost, const N: usize> PatternConverter<Pattern> for DefaultPatternConverter<Cost, N>
where
    u8: Into<Cost>,
    Cost: core::ops::Mul<Output = Cost> + Copy,
{
    type Cost = Cost;

    fn convert(&self, pattern: &Pattern) -> Self::Cost {
        use Pattern::*;

        match pattern {
            &Straight(x) => self.straight_array[x as usize - 1],
            &StraightDiagonal(x) => self.straight_diagonal_array[x as usize - 1],
            Search90 => self.search90,
            FastRun45 => self.fast_run45,
            FastRun90 => self.fast_run90,
            FastRun135 => self.fast_run135,
            FastRun180 => self.fast_run180,
            FastRunDiagonal90 => self.fast_run_diagonal90,
            SpinBack => self.spin_back,
        }
    }
}

impl<Config, State, Resource, Cost, const N: usize> Construct<Config, State, Resource>
    for DefaultPatternConverter<Cost, N>
where
    Cost: Clone,
    Config: AsRef<DefaultPatternConverter<Cost, N>>,
{
    fn construct<'a>(config: &'a Config, _state: &'a State, _resource: &'a mut Resource) -> Self {
        let converter = config.as_ref();
        converter.clone()
    }
}

impl<State, Resource, Cost, const N: usize> Deconstruct<State, Resource>
    for DefaultPatternConverter<Cost, N>
where
    State: Default,
    Resource: Default,
{
    fn deconstruct(self) -> (State, Resource) {
        (Default::default(), Default::default())
    }
}

impl<const N: usize> Default for DefaultPatternConverter<u16, N> {
    fn default() -> Self {
        use core::array::IntoIter;

        Self {
            search90: 8,
            fast_run45: 12,
            fast_run90: 15,
            fast_run135: 20,
            fast_run180: 25,
            fast_run_diagonal90: 15,
            spin_back: 15,
            straight_array: IntoIter::new([
                10, 16, 20, 24, 27, 29, 31, 33, 35, 37, 39, 42, 44, 46, 49, 51, 53, 55, 58, 60, 62,
                65, 67, 69, 72, 74, 76, 78, 81, 83, 85,
            ])
            .take(N)
            .collect(),
            straight_diagonal_array: IntoIter::new([
                7, 12, 15, 18, 21, 23, 25, 26, 28, 29, 31, 32, 33, 34, 36, 37, 39, 40, 42, 43, 45,
                47, 48, 50, 51, 53, 54, 56, 57, 59, 60,
            ])
            .take(N)
            .collect(),
        }
    }
}
