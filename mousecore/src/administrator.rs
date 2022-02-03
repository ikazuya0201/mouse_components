//! Definition of [Administrator](Administrator) and its dependencies.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

/// Error on [Operator](Operator).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperatorError<T> {
    Incompleted,
    Other(T),
}

/// A trait that consumes and initializes operators.
pub trait OperatorStore<Mode, Operator> {
    type Error;

    /// Exchanges an operator by a given mode.
    fn exchange(&self, operator: Operator, mode: Mode) -> Result<Operator, Self::Error>;
}

/// A trait that has a successor.
pub trait Successor {
    /// Returns the successor of the value.
    fn successor(&self) -> Self;
}

/// A trait that operates a given process.
pub trait Operator {
    type Error;

    /// Executes periodic tasks for control.
    /// The execution should never be blocked.
    fn tick(&self) -> Result<(), Self::Error>;

    /// Executes non-periodic tasks like solving maze.
    /// The execution can be blocked.
    fn run(&self) -> Result<(), OperatorError<Self::Error>>;

    /// Stops all actuators.
    fn stop(&self);
}

/// A trait that is used for selecting a mode.
pub trait Selector<Mode> {
    /// Resets the state of selector.
    fn reset(&self);

    /// Gets the currently selected mode.
    fn mode(&self) -> Mode;

    /// Returns where the selector is enabled or not.
    fn is_enabled(&self) -> bool;
}

/// A trait that manages interrupts.
pub trait InterruptManager {
    /// Turns on periodic interrupt.
    fn turn_on(&self);

    /// Turns off periodic interrupt.
    fn turn_off(&self);
}

// TODO: Write test.
/// A type that manages modes and orchestrates operators.
pub struct Administrator<Mode, Selector, Operator, Store, Manager> {
    is_select: AtomicBool,
    selector: Selector,
    operator: RefCell<Option<Operator>>,
    store: Store,
    mode: RefCell<Mode>,
    manager: Manager,
}

/// Error on [Administrator].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AdministratorError<OperatorError, StoreError> {
    Operator(OperatorError),
    OperatorStore(StoreError),
}

impl<Mode, SelectorType, OperatorType, Store, Manager>
    Administrator<Mode, SelectorType, OperatorType, Store, Manager>
where
    Mode: Successor + Copy,
    SelectorType: Selector<Mode>,
    OperatorType: Operator,
    Store: OperatorStore<Mode, OperatorType>,
    Manager: InterruptManager,
{
    pub fn new(
        selector: SelectorType,
        operator: OperatorType,
        store: Store,
        mode: Mode,
        manager: Manager,
    ) -> Self {
        Self {
            is_select: AtomicBool::new(false),
            selector,
            operator: RefCell::new(Some(operator)),
            store,
            mode: RefCell::new(mode),
            manager,
        }
    }

    pub fn start(&self) {
        self.manager.turn_on();
    }

    pub fn run(&self) -> Result<(), AdministratorError<OperatorType::Error, Store::Error>> {
        if self.is_select.load(Ordering::Relaxed) {
            self.mode_select()
                .map_err(AdministratorError::OperatorStore)
        } else {
            let operator = self.operator.borrow();
            if let Err(err) = operator.as_ref().unwrap_or_else(|| todo!()).run() {
                match err {
                    OperatorError::Incompleted => Ok(()),
                    OperatorError::Other(err) => Err(AdministratorError::Operator(err)),
                }
            } else {
                self.manager.turn_off();
                core::mem::drop(operator);
                let mut mode = self.mode.borrow_mut();
                *mode = mode.successor();
                self.exchange_operator(*mode)
                    .map_err(AdministratorError::OperatorStore)?;
                self.manager.turn_on();
                Ok(())
            }
        }
    }

    fn exchange_operator(&self, mode: Mode) -> Result<(), Store::Error> {
        let operator = {
            let operator = self
                .operator
                .borrow_mut()
                .take()
                .expect("Should never be None");
            operator.stop();
            self.store.exchange(operator, mode)?
        };
        *self.operator.borrow_mut() = Some(operator);
        Ok(())
    }

    fn mode_select(&self) -> Result<(), Store::Error> {
        self.manager.turn_off();
        //waiting for switch off
        while self.selector.is_enabled() {}

        self.selector.reset();

        //waiting for switch on
        while !self.selector.is_enabled() {}
        //waiting for switch off
        while self.selector.is_enabled() {}

        let mode = self.selector.mode();
        *self.mode.borrow_mut() = mode;
        self.exchange_operator(mode)?;

        self.is_select.store(false, Ordering::Relaxed);
        self.manager.turn_on();
        Ok(())
    }
}

impl<Mode, SelectorType, OperatorType, Store, Manager>
    Administrator<Mode, SelectorType, OperatorType, Store, Manager>
where
    SelectorType: Selector<Mode>,
    OperatorType: Operator,
{
    //called by periodic interrupt
    pub fn tick(&self) -> Result<(), OperatorType::Error> {
        if self.is_select.load(Ordering::Relaxed) {
            return Ok(());
        } else if let Some(operator) = self.operator.borrow().as_ref() {
            operator.tick()?
        }
        if self.selector.is_enabled() {
            self.is_select.store(true, Ordering::Relaxed);
        }
        Ok(())
    }
}
