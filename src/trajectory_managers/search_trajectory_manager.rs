use core::sync::atomic::{AtomicUsize, Ordering};

use heapless::{consts::*, spsc::Queue};
use spin::Mutex;

use crate::agents::SearchTrajectoryManager;
use crate::trajectory_managers::CommandConverter;

pub trait SearchTrajectoryGenerator<Command> {
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;

    fn generate_search(&self, command: &Command) -> Self::Trajectory;
    fn generate_emergency(&self, target: &Self::Target) -> Self::Trajectory;
}

type QueueLength = U3;

pub struct TrajectoryManager<Generator, Converter, Target, Trajectory> {
    generator: Generator,
    converter: Converter,
    trajectories: Mutex<Queue<Trajectory, QueueLength>>,
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

#[derive(Debug)]
pub enum TrajectoryManagerError {
    MutexLocked,
    FullQueue,
}

impl<Generator, Converter, Command> SearchTrajectoryManager<Command>
    for TrajectoryManager<Generator, Converter, Generator::Target, Generator::Trajectory>
where
    Generator: SearchTrajectoryGenerator<Converter::Output>,
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
            let trajectory = self.generator.generate_search(&command);
            trajectories
                .enqueue(trajectory)
                .unwrap_or_else(|_| unreachable!("Should never exceed the length of queue."));
            Ok(())
        }
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

    fn is_empty(&self) -> Result<bool, Self::Error> {
        Ok(self
            .trajectories
            .try_lock()
            .ok_or(TrajectoryManagerError::MutexLocked)?
            .is_empty())
    }

    fn is_full(&self) -> Result<bool, Self::Error> {
        use typenum::Unsigned;

        Ok(self
            .trajectories
            .try_lock()
            .ok_or(TrajectoryManagerError::MutexLocked)?
            .len()
            == QueueLength::USIZE)
    }
}
