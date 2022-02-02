use core::marker::PhantomData;

#[allow(unused_imports)]
use micromath::F32Ext;
use typenum::{consts::*, operator_aliases::PartialQuot, type_operators::PartialDiv, Integer};
use uom::marker::Div;
use uom::si::{Dimension, Quantity, ISQ, SI};

#[allow(clippy::type_complexity)]
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
        value: val.value.sqrt(),
    }
}
