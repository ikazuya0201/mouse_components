#[derive(PartialEq, PartialOrd, Copy, Clone)]
pub struct Total<T>(pub T);

impl<T: PartialEq> Eq for Total<T> {}

impl<T: PartialOrd> Ord for Total<T> {
    fn cmp(&self, other: &Total<T>) -> core::cmp::Ordering {
        self.0.partial_cmp(&other.0).unwrap()
    }
}
