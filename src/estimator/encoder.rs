use quantities::Distance;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderError;

pub trait Encoder {
    fn get_relative_distance(&mut self) -> nb::Result<Distance, EncoderError>;
}
