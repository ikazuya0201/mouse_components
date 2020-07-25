use quantities::Distance;

pub trait Encoder {
    type Error;

    fn get_relative_distance(&mut self) -> nb::Result<Distance, Self::Error>;
}
