use core::sync::atomic::{AtomicUsize, Ordering};

use spin::Mutex;

use crate::agents::TrackingTrajectoryManager;
use crate::operators::TrackingInitializer;
use crate::trajectory_managers::CommandConverter;

pub trait InitialTrajectoryGenerator<Command> {
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;
    type Trajectories: core::ops::DerefMut<Target = [Self::Trajectory]>;

    fn generate<Commands: IntoIterator<Item = Command>>(
        &self,
        commands: Commands,
    ) -> Self::Trajectories;
}

pub struct TrajectoryManager<Command, Generator, Converter>
where
    Generator: InitialTrajectoryGenerator<Converter::Output>,
    Converter: CommandConverter<Command>,
{
    generator: Generator,
    converter: Converter,
    trajectories: Mutex<Generator::Trajectories>,
    iter: AtomicUsize,
}

impl<Command, Generator, Converter> TrajectoryManager<Command, Generator, Converter>
where
    Generator: InitialTrajectoryGenerator<Converter::Output>,
    Converter: CommandConverter<Command>,
    Generator::Trajectories: Default,
{
    pub fn new(generator: Generator, converter: Converter) -> Self {
        Self {
            generator,
            converter,
            trajectories: Mutex::new(Default::default()),
            iter: AtomicUsize::new(0),
        }
    }
}

impl<Command, Generator, Converter> TrackingInitializer<Command>
    for TrajectoryManager<Command, Generator, Converter>
where
    Generator: InitialTrajectoryGenerator<Converter::Output>,
    Converter: CommandConverter<Command>,
{
    type Error = core::convert::Infallible;

    fn initialize<Commands: IntoIterator<Item = Command>>(
        &self,
        commands: Commands,
    ) -> Result<(), Self::Error> {
        let commands = commands.into_iter().map(|com| self.converter.convert(&com));
        *self.trajectories.lock() = self.generator.generate(commands);
        Ok(())
    }
}

impl<Command, Generator, Converter> TrackingTrajectoryManager
    for TrajectoryManager<Command, Generator, Converter>
where
    Generator: InitialTrajectoryGenerator<Converter::Output>,
    Converter: CommandConverter<Command>,
{
    type Target = Generator::Target;

    fn next(&self) -> Option<Self::Target> {
        let mut trajectories = self.trajectories.lock();
        while !trajectories.is_empty() {
            if let Some(target) = trajectories[self.iter.load(Ordering::Acquire)].next() {
                return Some(target);
            } else {
                self.iter.fetch_add(1, Ordering::AcqRel);
            }
        }
        None
    }
}
