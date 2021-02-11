use heapless::{spsc::Queue, ArrayLength};
use spin::Mutex;

use crate::agents::TrackingTrajectoryManager;
use crate::operators::TrackingInitializer;
use crate::trajectory_managers::CommandConverter;

pub trait InitialTrajectoryGenerator<Command> {
    type MaxLength;
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;
    type Trajectories: IntoIterator<Item = Self::Trajectory>;

    fn generate<Commands: IntoIterator<Item = Command>>(
        &self,
        commands: Commands,
    ) -> Self::Trajectories;
}

pub struct TrajectoryManager<Command, Generator, Converter>
where
    Generator: InitialTrajectoryGenerator<Converter::Output>,
    Converter: CommandConverter<Command>,
    Generator::MaxLength: ArrayLength<Generator::Trajectory>,
{
    generator: Generator,
    converter: Converter,
    trajectories: Mutex<Queue<Generator::Trajectory, Generator::MaxLength>>,
}

impl<Command, Generator, Converter> TrajectoryManager<Command, Generator, Converter>
where
    Generator: InitialTrajectoryGenerator<Converter::Output>,
    Converter: CommandConverter<Command>,
    Generator::MaxLength: ArrayLength<Generator::Trajectory>,
{
    pub fn new(generator: Generator, converter: Converter) -> Self {
        Self {
            generator,
            converter,
            trajectories: Mutex::new(Queue::new()),
        }
    }
}

impl<Command, Generator, Converter> TrackingInitializer<Command>
    for TrajectoryManager<Command, Generator, Converter>
where
    Generator: InitialTrajectoryGenerator<Converter::Output>,
    Converter: CommandConverter<Command>,
    Generator::MaxLength: ArrayLength<Generator::Trajectory>,
{
    type Error = core::convert::Infallible;

    fn initialize<Commands: IntoIterator<Item = Command>>(
        &self,
        commands: Commands,
    ) -> Result<(), Self::Error> {
        let commands = commands.into_iter().map(|com| self.converter.convert(&com));
        let mut trajectories = self.trajectories.lock();
        for trajectory in self.generator.generate(commands) {
            trajectories.enqueue(trajectory).unwrap_or_else(|_| {
                unreachable!("Should never exceed the length of trajectory queue")
            });
        }
        Ok(())
    }
}

impl<Command, Generator, Converter> TrackingTrajectoryManager
    for TrajectoryManager<Command, Generator, Converter>
where
    Generator: InitialTrajectoryGenerator<Converter::Output>,
    Converter: CommandConverter<Command>,
    Generator::MaxLength: ArrayLength<Generator::Trajectory>,
{
    type Target = Generator::Target;

    fn next(&self) -> Option<Self::Target> {
        let mut trajectories = self.trajectories.lock();
        while !trajectories.is_empty() {
            let trajectory = trajectories
                .iter_mut()
                .next()
                .unwrap_or_else(|| unreachable!());
            if let Some(target) = trajectory.next() {
                return Some(target);
            } else {
                trajectories.dequeue();
            }
        }
        None
    }
}
