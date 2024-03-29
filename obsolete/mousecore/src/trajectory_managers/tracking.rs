use heapless::spsc::Queue;
use spin::Mutex;

use crate::agents::TrackingTrajectoryManager;
use crate::trajectory_managers::CommandConverter;
use crate::Construct;

/// A trait that generates trajectory for search.
pub trait TrackingTrajectoryGenerator<Command> {
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;

    fn generate(&self, command: &Command) -> Self::Trajectory;
}

const QUEUE_LENGTH: usize = 4;

/// An implementation of [TrackingTrajectoryManager](crate::agents::TrackingTrajectoryManager) required
/// by [TrackingAgent](crate::agents::TrackingAgent).
#[derive(Debug)]
pub struct TrajectoryManager<Generator, Converter, Target, Trajectory> {
    generator: Generator,
    converter: Converter,
    trajectories: Mutex<Queue<Trajectory, QUEUE_LENGTH>>,
    last_target: Mutex<Option<Target>>,
}

impl<Generator, Converter, Target, Trajectory>
    TrajectoryManager<Generator, Converter, Target, Trajectory>
{
    pub fn new(generator: Generator, converter: Converter) -> Self {
        Self {
            generator,
            converter,
            trajectories: Mutex::new(Queue::new()),
            last_target: Mutex::new(None),
        }
    }
}

impl<Generator, Converter, Target, Trajectory, Config, State, Resource>
    Construct<Config, State, Resource>
    for TrajectoryManager<Generator, Converter, Target, Trajectory>
where
    Generator: Construct<Config, State, Resource>,
    Converter: Construct<Config, State, Resource>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let generator = Generator::construct(config, state, resource);
        let converter = Converter::construct(config, state, resource);
        Self::new(generator, converter)
    }
}

impl_deconstruct_with_default!(TrajectoryManager<Generator, Converter, Target, Trajectory>);

/// Error on [TrajectoryManager](TrajectoryManager).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TrajectoryManagerError {
    FullQueue,
}

impl<Generator, Converter, Command> TrackingTrajectoryManager<Command>
    for TrajectoryManager<Generator, Converter, Generator::Target, Generator::Trajectory>
where
    Generator: TrackingTrajectoryGenerator<Converter::Output>,
    Generator::Target: Clone,
    Converter: CommandConverter<Command>,
{
    type Error = TrajectoryManagerError;
    type Target = Generator::Target;

    fn set_command(&self, command: &Command) -> Result<(), Self::Error> {
        if self.is_full().unwrap_or(true) {
            return Err(TrajectoryManagerError::FullQueue);
        }

        //blocking
        let mut trajectories = self.trajectories.lock();

        let command = self.converter.convert(command);
        let trajectory = self.generator.generate(&command);
        trajectories
            .enqueue(trajectory)
            .unwrap_or_else(|_| unreachable!("Should never exceed the length of queue."));
        Ok(())
    }

    fn next(&self) -> Self::Target {
        let target = if let Some(mut trajectories) = self.trajectories.try_lock() {
            loop {
                if let Some(trajectory) = trajectories.iter_mut().next() {
                    if let Some(target) = trajectory.next() {
                        break Some(target);
                    }
                } else {
                    break None;
                }
                trajectories.dequeue();
            }
        } else {
            None
        };

        if let Some(target) = target {
            *self.last_target.lock() = Some(target.clone());
            target
        } else if let Some(target) = self.last_target.lock().clone() {
            target
        } else {
            unimplemented!()
        }
    }

    fn is_empty(&self) -> Option<bool> {
        Some(self.trajectories.try_lock()?.is_empty())
    }

    fn is_full(&self) -> Option<bool> {
        // actual queue length is QUEUE_LENGTH-1
        Some(self.trajectories.try_lock()?.len() + 1 == QUEUE_LENGTH)
    }
}
