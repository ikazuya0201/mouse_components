use core::marker::PhantomData;

use spin::Mutex;

use crate::agents::ReturnSetupTrajectoryManager as IReturnSetupTrajectoryManager;
use crate::operators::ReturnSetupCommandConsumer;

/// A trait that generates trajectory for setting up return to start.
pub trait ReturnSetupTrajectoryGenerator<Command> {
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;

    fn generate(&self, command: &Command) -> Self::Trajectory;
}

/// An implementation of
/// [ReturnSetupTrajectoryManager](crate::agents::ReturnSetupTrajectoryManager) and
/// [ReturnSetupCommandConsumer](crate::operators::ReturnSetupCommandConsumer) required by
/// [ReturnSetupAgent](crate::agents::ReturnSetupAgent).
pub struct ReturnSetupTrajectoryManager<Generator, Target, Trajectory> {
    generator: Generator,
    trajectory: Mutex<Option<Trajectory>>,
    _target: PhantomData<fn() -> Target>,
}

impl<Generator, Target, Trajectory> ReturnSetupTrajectoryManager<Generator, Target, Trajectory> {
    pub fn new(generator: Generator) -> Self {
        Self {
            generator,
            trajectory: Mutex::new(None),
            _target: PhantomData,
        }
    }
}

impl<Generator, Target, Trajectory> IReturnSetupTrajectoryManager
    for ReturnSetupTrajectoryManager<Generator, Target, Trajectory>
where
    Trajectory: Iterator<Item = Target>,
{
    type Target = Target;

    fn next(&self) -> Option<Self::Target> {
        let mut trajectory = self.trajectory.lock();
        if let Some(trajectory) = trajectory.as_mut() {
            if let Some(target) = trajectory.next() {
                return Some(target);
            }
        }
        None
    }
}

impl<Generator, Command> ReturnSetupCommandConsumer<Command>
    for ReturnSetupTrajectoryManager<Generator, Generator::Target, Generator::Trajectory>
where
    Generator: ReturnSetupTrajectoryGenerator<Command>,
{
    type Error = core::convert::Infallible;

    fn set_command(&self, command: &Command) -> Result<(), Self::Error> {
        self.trajectory
            .lock()
            .replace(self.generator.generate(command));
        Ok(())
    }
}
