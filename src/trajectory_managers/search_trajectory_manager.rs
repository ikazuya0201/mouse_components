use core::sync::atomic::{AtomicUsize, Ordering};

use heapless::spsc::Queue;
use spin::Mutex;

use crate::agents::TrackingTrajectoryManager;
use crate::trajectory_managers::CommandConverter;
use crate::{impl_deconstruct_with_default, Construct};

/// A trait that generates trajectory for search.
pub trait SearchTrajectoryGenerator<Command> {
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;

    fn generate_search(&self, command: &Command) -> Self::Trajectory;
    fn generate_emergency(&self, target: &Self::Target) -> Self::Trajectory;
}

const QUEUE_LENGTH: usize = 4;

/// An implementation of [TrackingTrajectoryManager](crate::agents::TrackingTrajectoryManager) required
/// by [TrackingAgent](crate::agents::TrackingAgent).
#[derive(Debug)]
pub struct TrajectoryManager<Generator, Converter, Target, Trajectory> {
    generator: Generator,
    converter: Converter,
    trajectories: Mutex<Queue<Trajectory, QUEUE_LENGTH>>,
    emergency_trajectory: Mutex<Option<Trajectory>>,
    emergency_counter: AtomicUsize,
    last_target: Mutex<Target>,
}

impl<Generator, Converter, Target, Trajectory>
    TrajectoryManager<Generator, Converter, Target, Trajectory>
{
    pub fn new(generator: Generator, converter: Converter) -> Self
    where
        Target: Default,
    {
        Self {
            generator,
            converter,
            trajectories: Mutex::new(Queue::new()),
            emergency_trajectory: Mutex::new(None),
            emergency_counter: AtomicUsize::new(0),
            last_target: Mutex::new(Target::default()),
        }
    }
}

impl<Generator, Converter, Target, Trajectory, Config, State, Resource>
    Construct<Config, State, Resource>
    for TrajectoryManager<Generator, Converter, Target, Trajectory>
where
    Generator: Construct<Config, State, Resource>,
    Converter: Construct<Config, State, Resource>,
    Target: Default,
{
    fn construct(config: &Config, state: &State, resource: Resource) -> (Self, Resource) {
        let (generator, resource) = Generator::construct(config, state, resource);
        let (converter, resource) = Converter::construct(config, state, resource);
        (Self::new(generator, converter), resource)
    }
}

impl_deconstruct_with_default!(TrajectoryManager<Generator,Converter, Target, Trajectory>);

/// Error on [TrajectoryManager](TrajectoryManager).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TrajectoryManagerError {
    FullQueue,
}

impl<Generator, Converter, Command> TrackingTrajectoryManager<Command>
    for TrajectoryManager<Generator, Converter, Generator::Target, Generator::Trajectory>
where
    Generator: SearchTrajectoryGenerator<Converter::Output>,
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
        let trajectory = self.generator.generate_search(&command);
        trajectories
            .enqueue(trajectory)
            .unwrap_or_else(|_| unreachable!("Should never exceed the length of queue."));
        Ok(())
    }

    fn next(&self) -> Self::Target {
        let get_or_init_emergency = || {
            let mut trajectory = self.emergency_trajectory.lock();
            if trajectory.is_none() {
                trajectory.replace(self.generator.generate_emergency(&*self.last_target.lock()));
                self.emergency_counter.store(0, Ordering::Release);
            }
            self.emergency_counter.fetch_add(1, Ordering::AcqRel);
            trajectory.as_mut().expect("Should never be None.").next()
        };

        let (target, is_emergency) = {
            if let Some(mut trajectories) = self.trajectories.try_lock() {
                loop {
                    if let Some(trajectory) = trajectories.iter_mut().next() {
                        if let Some(target) =
                            trajectory.nth(self.emergency_counter.load(Ordering::Acquire))
                        {
                            break (Some(target), false);
                        }
                    } else {
                        break (get_or_init_emergency(), true);
                    }
                    trajectories.dequeue();
                }
            } else {
                (get_or_init_emergency(), true)
            }
        };

        if !is_emergency {
            *self.emergency_trajectory.lock() = None;
            self.emergency_counter.store(0, Ordering::Release);
        }

        if let Some(target) = target {
            *self.last_target.lock() = target.clone();
            target
        } else {
            self.last_target.lock().clone()
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
