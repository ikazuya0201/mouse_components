use core::marker::PhantomData;

use typenum::{consts::*, operator_aliases::PartialQuot, type_operators::PartialDiv, Integer};
use uom::marker::Div;
use uom::si::{angle::radian, f32::Angle, Dimension, Quantity, ISQ, SI};

pub trait Math {
    fn sinf(val: f32) -> f32;
    fn cosf(val: f32) -> f32;

    fn sincosf(val: f32) -> (f32, f32) {
        (Self::sinf(val), Self::cosf(val))
    }

    fn sin(angle: Angle) -> f32 {
        Self::sinf(angle.get::<radian>())
    }

    fn cos(angle: Angle) -> f32 {
        Self::cosf(angle.get::<radian>())
    }

    fn sincos(angle: Angle) -> (f32, f32) {
        Self::sincosf(angle.get::<radian>())
    }
}

/// Calculates remainder of x divided by y.
///
/// The result is in the range of [0, y).
pub fn rem_euclidf(x: f32, y: f32) -> f32 {
    let rem = libm::remainderf(x, y);
    if rem.is_sign_negative() {
        y + rem
    } else {
        rem
    }
}

pub fn sqrt<D>(
    val: Quantity<D, SI<f32>, f32>,
) -> Quantity<
    ISQ<
        PartialQuot<D::L, P2>,
        PartialQuot<D::M, P2>,
        PartialQuot<D::T, P2>,
        PartialQuot<D::I, P2>,
        PartialQuot<D::Th, P2>,
        PartialQuot<D::N, P2>,
        PartialQuot<D::J, P2>,
    >,
    SI<f32>,
    f32,
>
where
    D: Dimension + ?Sized,
    D::L: PartialDiv<P2>,
    <D::L as PartialDiv<P2>>::Output: Integer,
    D::M: PartialDiv<P2>,
    <D::M as PartialDiv<P2>>::Output: Integer,
    D::T: PartialDiv<P2>,
    <D::T as PartialDiv<P2>>::Output: Integer,
    D::I: PartialDiv<P2>,
    <D::I as PartialDiv<P2>>::Output: Integer,
    D::Th: PartialDiv<P2>,
    <D::Th as PartialDiv<P2>>::Output: Integer,
    D::N: PartialDiv<P2>,
    <D::N as PartialDiv<P2>>::Output: Integer,
    D::J: PartialDiv<P2>,
    <D::J as PartialDiv<P2>>::Output: Integer,
    D::Kind: Div,
{
    Quantity {
        dimension: PhantomData,
        units: PhantomData,
        value: libm::sqrtf(val.value),
    }
}

pub struct LibmMath;

impl Math for LibmMath {
    fn sinf(val: f32) -> f32 {
        libm::sinf(val)
    }

    fn cosf(val: f32) -> f32 {
        libm::cosf(val)
    }
}

#[cfg(test)]
pub use dummy::MathFake;

#[cfg(test)]
mod dummy {
    use super::*;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct MathFake;

    impl Math for MathFake {
        fn sinf(val: f32) -> f32 {
            val.sin()
        }

        fn cosf(val: f32) -> f32 {
            val.cos()
        }
    }
}
