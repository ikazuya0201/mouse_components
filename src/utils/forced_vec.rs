use heapless::{ArrayLength, Vec};

#[derive(Clone, Debug)]
pub(crate) struct ForcedVec<T, N>(Vec<T, N>)
where
    N: ArrayLength<T>;

impl<T, N> ForcedVec<T, N>
where
    N: ArrayLength<T>,
{
    pub fn new() -> Self {
        ForcedVec(Vec::new())
    }

    pub fn push(&mut self, item: T) {
        self.0
            .push(item)
            .unwrap_or_else(|_| unreachable!("Should never exceed the max length"));
    }

    pub fn extend_from_slice(&mut self, other: &[T])
    where
        T: Clone,
    {
        self.0
            .extend_from_slice(other)
            .unwrap_or_else(|_| unreachable!("Should never exceed the max length"));
    }
}

impl<T, N> core::ops::Deref for ForcedVec<T, N>
where
    N: ArrayLength<T>,
{
    type Target = Vec<T, N>;

    fn deref(&self) -> &Self::Target {
        use core::borrow::Borrow;
        self.0.borrow()
    }
}

impl<T, N> core::ops::DerefMut for ForcedVec<T, N>
where
    N: ArrayLength<T>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        use core::borrow::BorrowMut;
        self.0.borrow_mut()
    }
}

impl<T, N> Into<Vec<T, N>> for ForcedVec<T, N>
where
    N: ArrayLength<T>,
{
    fn into(self) -> Vec<T, N> {
        self.0
    }
}
