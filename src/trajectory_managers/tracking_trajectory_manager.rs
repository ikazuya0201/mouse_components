use heapless::{consts::*, spsc::Queue};
use spin::Mutex;

use crate::agents::TrackingTrajectoryManager;
use crate::trajectory_managers::CommandConverter;

/// A trait that generates trajectory for search.
pub trait TrackingTrajectoryGenerator<Command> {
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;

    fn generate(&self, command: &Command) -> Self::Trajectory;
}

type QueueLength = U3;

/// An implementation of [TrackingTrajectoryManager](crate::agents::TrackingTrajectoryManager) required
/// by [TrackingAgent](crate::agents::TrackingAgent).
#[derive(Debug)]
pub struct TrajectoryManager<Generator, Converter, Target, Trajectory> {
    generator: Generator,
    converter: Converter,
    trajectories: Mutex<Queue<Trajectory, QueueLength>>,
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
        use typenum::Unsigned;

        //blocking
        let mut trajectories = self.trajectories.lock();

        if trajectories.len() == QueueLength::USIZE {
            Err(TrajectoryManagerError::FullQueue)
        } else {
            let command = self.converter.convert(command);
            let trajectory = self.generator.generate(&command);
            trajectories
                .enqueue(trajectory)
                .unwrap_or_else(|_| unreachable!("Should never exceed the length of queue."));
            Ok(())
        }
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
        use typenum::Unsigned;

        Some(self.trajectories.try_lock()?.len() == QueueLength::USIZE)
    }
}
