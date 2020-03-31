use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

pub struct AtomicEnum<T>(AtomicU8, PhantomData<fn() -> T>);

impl<T> AtomicEnum<T>
where
    T: Into<u8> + From<u8>,
{
    pub fn new(val: T) -> Self {
        Self(AtomicU8::new(val.into()), PhantomData)
    }

    pub fn load(&self, order: Ordering) -> T {
        let raw = self.0.load(order);
        T::from(raw)
    }

    pub fn store(&self, val: T, order: Ordering) {
        self.0.store(val.into(), order)
    }
}
