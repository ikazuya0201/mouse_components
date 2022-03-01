use embedded_hal::Qei;
use uom::si::{angle::revolution, f32::Angle};

pub struct MA702GQ<Q>
where
    Q: Qei,
{
    qei: Q,
    before_count: u16,
}

impl<Q> MA702GQ<Q>
where
    Q: Qei,
{
    const RESOLUTION_PER_ROTATION: f32 = 1024.0;

    pub fn new(qei: Q) -> Self {
        Self {
            qei,
            before_count: 0,
        }
    }

    #[inline]
    fn get_lower_bits(&self, val: u32) -> u16 {
        (val & core::u16::MAX as u32) as u16
    }
}

impl<Q> MA702GQ<Q>
where
    Q: Qei,
    Q::Count: Into<u32>,
{
    pub fn angle(&mut self) -> nb::Result<Angle, core::convert::Infallible> {
        let after_count = self.get_lower_bits(self.qei.count().into());
        let relative_count = if after_count > self.before_count {
            (after_count - self.before_count) as i16
        } else {
            -((self.before_count - after_count) as i16)
        };
        self.before_count = after_count;
        Ok(Angle::new::<revolution>(
            relative_count as f32 / Self::RESOLUTION_PER_ROTATION,
        ))
    }
}
