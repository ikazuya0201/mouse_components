#[derive(PartialEq, PartialOrd, Copy, Clone, Debug)]
pub struct Total<T>(pub T);

impl<T: PartialEq> Eq for Total<T> {}

#[allow(clippy::derive_ord_xor_partial_ord)]
impl<T: PartialOrd> Ord for Total<T> {
    fn cmp(&self, other: &Total<T>) -> core::cmp::Ordering {
        self.0.partial_cmp(&other.0).unwrap()
    }
}
