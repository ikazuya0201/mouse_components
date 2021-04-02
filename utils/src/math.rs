use components::utils::math::Math;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MathFake;

impl Math for MathFake {
    fn sinf(val: f32) -> f32 {
        val.sin()
    }

    fn cosf(val: f32) -> f32 {
        val.cos()
    }
}
