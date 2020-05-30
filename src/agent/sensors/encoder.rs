use quantities::Angle;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderError;

pub trait Encoder {
    fn get_absolute_angle(&mut self) -> nb::Result<Angle, EncoderError>;
}
