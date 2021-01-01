use components::utils::math::Math;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MathFake;

impl Math for MathFake {
    fn sqrtf(val: f32) -> f32 {
        val.sqrt()
    }

    fn sinf(val: f32) -> f32 {
        val.sin()
    }

    fn cosf(val: f32) -> f32 {
        val.cos()
    }

    fn expf(val: f32) -> f32 {
        val.exp()
    }

    fn atan2f(x: f32, y: f32) -> f32 {
        x.atan2(y)
    }

    fn rem_euclidf(lhs: f32, rhs: f32) -> f32 {
        lhs.rem_euclid(rhs)
    }
}
