use heapless::Vec;

#[derive(Clone, Debug)]
pub(crate) struct ForcedVec<T, const N: usize>(Vec<T, N>);

impl<T, const N: usize> ForcedVec<T, N> {
    pub fn new() -> Self {
        ForcedVec(Vec::new())
    }

    pub fn push(&mut self, item: T) {
        self.0
            .push(item)
            .unwrap_or_else(|_| unreachable!("Should never exceed the max length"));
    }
}

impl<T, const N: usize> core::ops::Deref for ForcedVec<T, N> {
    type Target = Vec<T, N>;

    fn deref(&self) -> &Self::Target {
        use core::borrow::Borrow;
        self.0.borrow()
    }
}

impl<T, const N: usize> core::ops::DerefMut for ForcedVec<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        use core::borrow::BorrowMut;
        self.0.borrow_mut()
    }
}

impl<T, const N: usize> Into<Vec<T, N>> for ForcedVec<T, N> {
    fn into(self) -> Vec<T, N> {
        self.0
    }
}
