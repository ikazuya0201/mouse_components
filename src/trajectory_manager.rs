use core::cell::{Cell, RefCell};

use heapless::{consts::*, spsc::Queue};
use spin::Mutex;

use crate::agents::simple_search_agent::TrajectoryManager as ITrajectoryManager;
use crate::agents::SearchTrajectoryGenerator;
use crate::operators::CommandConverter;

type QueueLength = U3;

pub struct TrajectoryManager<Generator, Converter, Target, Trajectory> {
    generator: Generator,
    converter: Converter,
    trajectories: Mutex<Queue<Trajectory, QueueLength>>,
    emergency_trajectory: RefCell<Option<Trajectory>>,
    emergency_counter: Cell<usize>,
    last_target: RefCell<Target>,
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
            emergency_trajectory: RefCell::new(None),
            emergency_counter: Cell::new(0),
            last_target: RefCell::new(Target::default()),
        }
    }
}

#[derive(Debug)]
pub enum TrajectoryManagerError {
    MutexLocked,
    FullQueue,
}

impl<Generator, Converter, Command> ITrajectoryManager<Command>
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
            let mut trajectory = self.emergency_trajectory.borrow_mut();
            if trajectory.is_none() {
                trajectory.replace(
                    self.generator
                        .generate_emergency(&*self.last_target.borrow()),
                );
                self.emergency_counter.set(0);
            }
            self.emergency_counter.set(self.emergency_counter.get() + 1);
            trajectory.as_mut().expect("Should never be None.").next()
        };

        let target = {
            if let Some(mut trajectories) = self.trajectories.try_lock() {
                loop {
                    if let Some(trajectory) = trajectories.iter_mut().next() {
                        if let Some(target) = trajectory.nth(self.emergency_counter.get()) {
                            break Some(target);
                        }
                    } else {
                        break get_or_init_emergency();
                    }
                    trajectories.dequeue();
                }
            } else {
                get_or_init_emergency()
            }
        };
        if let Some(target) = target {
            self.last_target.replace(target.clone());
            self.emergency_trajectory.replace(None);
            self.emergency_counter.set(0);
            target
        } else {
            self.last_target.borrow().clone()
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
