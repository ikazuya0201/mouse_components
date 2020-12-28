#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Probability(f32);

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct OutOfBoundError {
    val: f32,
}

impl Probability {
    pub fn new(val: f32) -> Result<Self, OutOfBoundError> {
        if val.is_nan() || val.is_sign_negative() || val > 1.0 {
            Err(OutOfBoundError { val })
        } else {
            Ok(Probability(val))
        }
    }

    pub fn one() -> Self {
        Probability(1.0)
    }

    pub fn zero() -> Self {
        Probability(0.0)
    }

    pub fn reverse(&self) -> Self {
        Probability(1.0 - self.0)
    }
}

impl Default for Probability {
    fn default() -> Self {
        Self::zero()
    }
}

impl core::convert::TryFrom<f32> for Probability {
    type Error = OutOfBoundError;

    fn try_from(value: f32) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

impl core::ops::Mul<Probability> for f32 {
    type Output = f32;

    fn mul(self, rhs: Probability) -> Self::Output {
        self * rhs.0
    }
}

impl core::ops::Mul<f32> for Probability {
    type Output = f32;

    fn mul(self, rhs: f32) -> Self::Output {
        self.0 * rhs
    }
}
