#[derive(Clone, Copy, Debug)]
pub enum Pattern {
    Straight(u16),
    StraightDiagonal(u16),
    Search90,
    FastRun45,
    FastRun90,
    FastRun135,
    FastRun180,
    FastRunDiagonal90,
    Back,
}
