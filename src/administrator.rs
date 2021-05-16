//! Definition of [Administrator](Administrator) and its dependencies.

use core::sync::atomic::{AtomicBool, Ordering};

use spin::Mutex;

/// Error on [Operator](Operator).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperatorError<T> {
    Incompleted,
    Other(T),
}

/// A trait that consumes and initializes operators.
pub trait OperatorStore<Mode, Operator> {
    /// Exchanges an operator by a given mode.
    fn exchange(&self, operator: Operator, mode: Mode) -> Operator;
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
    operator: Mutex<Option<Operator>>,
    store: Store,
    mode: Mutex<Mode>,
    manager: Manager,
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
            operator: Mutex::new(Some(operator)),
            store,
            mode: Mutex::new(mode),
            manager,
        }
    }

    pub fn run(&self) -> Result<(), OperatorType::Error> {
        if self.is_select.load(Ordering::Relaxed) {
            self.mode_select();
            Ok(())
        } else if let Err(err) = self
            .operator
            .lock()
            .as_ref()
            .unwrap_or_else(|| todo!())
            .run()
        {
            match err {
                OperatorError::Incompleted => Ok(()),
                OperatorError::Other(err) => Err(err),
            }
        } else {
            self.manager.turn_off();
            let mut mode = self.mode.lock();
            *mode = mode.successor();
            self.exchange_operator(*mode);
            self.manager.turn_on();
            Ok(())
        }
    }

    fn exchange_operator(&self, mode: Mode) {
        let operator = self.operator.lock().take().expect("Should never be None");
        let operator = self.store.exchange(operator, mode);
        *self.operator.lock() = Some(operator);
    }

    fn mode_select(&self) {
        self.manager.turn_off();
        //waiting for switch off
        while self.selector.is_enabled() {}

        self.selector.reset();

        //waiting for switch on
        while !self.selector.is_enabled() {}
        //waiting for switch off
        while self.selector.is_enabled() {}

        let mode = self.selector.mode();
        *self.mode.lock() = mode;
        self.exchange_operator(mode);

        self.is_select.store(false, Ordering::Relaxed);
        self.manager.turn_on();
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
        } else if let Some(operator) = self.operator.try_lock() {
            operator.as_ref().unwrap_or_else(|| todo!()).tick()?
        }
        if self.selector.is_enabled() {
            self.is_select.store(true, Ordering::Relaxed);
        }
        Ok(())
    }
}
