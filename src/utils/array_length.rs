use generic_array::ArrayLength as GArrayLength;
use heapless::ArrayLength as HArrayLength;

pub trait ArrayLength<T>: GArrayLength<T> + HArrayLength<T> {}

impl<T, U> ArrayLength<U> for T where T: GArrayLength<U> + HArrayLength<U> {}
