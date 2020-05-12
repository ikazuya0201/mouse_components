pub trait Counter {
    fn reset(&self);
    fn count(&self) -> u8;
}
