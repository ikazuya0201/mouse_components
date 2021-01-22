use core::marker::PhantomData;

use heapless::Vec;
use uom::si::f32::Length;

use crate::data_types::{Obstacle, Pose};
use crate::maze::WallManager;
use crate::robot::WallDetector as IWallDetector;
use crate::utils::{forced_vec::ForcedVec, math::Math as IMath, probability::Probability};

#[derive(Debug, Clone)]
pub struct CorrectInfo {
    pub obstacle: Obstacle,
    pub diff_from_expected: Length,
}

pub struct WallInfo<Wall> {
    pub wall: Wall,
    pub existing_distance: Length,
    pub not_existing_distance: Length,
}

pub trait PoseConverter<Pose> {
    type Error;
    type Wall;

    fn convert(&self, pose: &Pose) -> Result<WallInfo<Self::Wall>, Self::Error>;
}

pub trait ObstacleDetector<State> {
    type Obstacle;
    type Obstacles: IntoIterator<Item = Self::Obstacle>;

    fn detect(&mut self, state: &State) -> Self::Obstacles;
}

pub struct WallDetector<'a, Manager, Detector, Converter, Math> {
    manager: &'a Manager,
    detector: Detector,
    converter: Converter,
    _math: PhantomData<fn() -> Math>,
}

impl<'a, Manager, Detector, Converter, Math> WallDetector<'a, Manager, Detector, Converter, Math> {
    pub fn new(manager: &'a Manager, detector: Detector, converter: Converter) -> Self {
        Self {
            manager,
            detector,
            converter,
            _math: PhantomData,
        }
    }
}

type ObstacleSizeUpperBound = typenum::consts::U6;

impl<'a, Manager, Detector, Converter, Math, State> IWallDetector<State>
    for WallDetector<'a, Manager, Detector, Converter, Math>
where
    Manager: WallManager<Converter::Wall>,
    Detector: ObstacleDetector<State, Obstacle = Obstacle>,
    Converter: PoseConverter<Pose>,
    Math: IMath,
{
    type Info = CorrectInfo;
    type Infos = Vec<CorrectInfo, ObstacleSizeUpperBound>;

    fn detect_and_update(&mut self, state: &State) -> Self::Infos {
        use uom::si::ratio::ratio;

        let obstacles = self.detector.detect(state);

        let mut infos = ForcedVec::new();
        for obstacle in obstacles {
            if let Ok(wall_info) = self.converter.convert(&obstacle.source) {
                if let Ok(existence) = self.manager.try_existence_probability(&wall_info.wall) {
                    if existence.is_zero() {
                        continue;
                    } else if existence.is_one() {
                        let diff_from_expected =
                            obstacle.distance.mean - wall_info.existing_distance;
                        infos.push(CorrectInfo {
                            obstacle,
                            diff_from_expected,
                        });
                        continue;
                    }

                    let exist_val = {
                        let tmp = ((wall_info.existing_distance - obstacle.distance.mean)
                            / obstacle.distance.standard_deviation)
                            .get::<ratio>();
                        -tmp * tmp / 2.0
                    };
                    let not_exist_val = {
                        let tmp = ((wall_info.not_existing_distance - obstacle.distance.mean)
                            / obstacle.distance.standard_deviation)
                            .get::<ratio>();
                        -tmp * tmp / 2.0
                    };

                    debug_assert!(!exist_val.is_nan());
                    debug_assert!(!not_exist_val.is_nan());

                    let min = if exist_val < not_exist_val {
                        exist_val
                    } else {
                        not_exist_val
                    };
                    let exist_val = Math::expf(exist_val - min) * existence;
                    let not_exist_val = Math::expf(not_exist_val - min) * existence.reverse();

                    let existence = if exist_val.is_infinite() {
                        Probability::one()
                    } else if not_exist_val.is_infinite() {
                        Probability::zero()
                    } else {
                        Probability::new(exist_val / (exist_val + not_exist_val)).unwrap_or_else(
                            |err| unreachable!("Should never be out of bound: {:?}", err),
                        )
                    };
                    let _ = self.manager.try_update(&wall_info.wall, &existence);
                    //ignore if cannot update
                }
            }
        }
        infos.into()
    }
}
