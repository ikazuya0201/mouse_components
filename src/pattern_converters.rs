//! Implementations for [PatternConverter](crate::mazes::PatternConverter).

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::impl_with_builder;
use crate::mazes::PatternConverter;
use crate::types::data::Pattern;

impl_with_builder! {
    LinearPatternConverterBuilder:
    {
        /// An implementation of [PatternConverter](PatternConverter).
        ///
        /// This calculates cost of straight pattern by multiplication with a given multiplier.
        #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
        #[derive(Clone, PartialEq, Eq, Debug)]
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
