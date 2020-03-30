use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

pub struct AtomicEnum<T>(AtomicU8, PhantomData<fn() -> T>);

impl<T> AtomicEnum<T>
where
    T: num::FromPrimitive + num::ToPrimitive,
{
    pub fn new(val: T) -> Self {
        Self(AtomicU8::new(val.to_u8().unwrap()), PhantomData)
    }

    pub fn load(&self, order: Ordering) -> T {
        let raw = self.0.load(order);
        T::from_u8(raw).unwrap()
    }

    pub fn store(&self, val: T, order: Ordering) {
        self.0.store(val.to_u8().unwrap(), order)
    }
}
