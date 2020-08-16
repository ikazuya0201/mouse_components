pub fn repeat_n<T>(val: T, count: usize) -> impl Iterator<Item = T>
where
    T: Clone,
{
    core::iter::repeat(val).take(count)
}

