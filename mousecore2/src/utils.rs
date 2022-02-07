#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct SideData<T> {
    pub left: T,
    pub right: T,
}
