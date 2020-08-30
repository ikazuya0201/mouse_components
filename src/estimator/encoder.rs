use uom::si::f32::Length;

pub trait Encoder {
    type Error;

    fn get_relative_distance(&mut self) -> nb::Result<Length, Self::Error>;
}
