use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use spin::Mutex;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IncompletedError;

pub trait OperatorStore<Mode, Operator> {
    fn exchange(&self, operator: Operator, mode: Mode) -> Operator;
    fn next(&self, operator: Operator) -> Operator;
}

pub trait Operator {
    type Error;

    fn tick(&self) -> Result<(), Self::Error>;
    fn run(&self) -> Result<(), Result<IncompletedError, Self::Error>>;
}

pub trait Selector<Mode> {
    fn reset(&self);
    fn mode(&self) -> Mode;
    fn is_enabled(&self) -> bool;
}

pub struct Administrator<Mode, ISelector, Operator, Store> {
    is_select: AtomicBool,
    selector: ISelector,
    operator: Mutex<Option<Operator>>,
    store: Store,
    _mode: PhantomData<fn() -> Mode>,
}

impl<Mode, ISelector, IOperator, Store> Administrator<Mode, ISelector, IOperator, Store>
where
    ISelector: Selector<Mode>,
    IOperator: Operator,
    Store: OperatorStore<Mode, IOperator>,
{
    pub fn new(selector: ISelector, operator: IOperator, store: Store) -> Self {
        Self {
            is_select: AtomicBool::new(false),
            selector,
            operator: Mutex::new(Some(operator)),
            store,
            _mode: PhantomData,
        }
    }

    //called by periodic interrupt
    pub fn tick(&self) {
        if self.is_select.load(Ordering::Relaxed) {
            return;
        } else if let Some(operator) = self.operator.try_lock() {
            operator
                .as_ref()
                .unwrap_or_else(|| todo!())
                .tick()
                .unwrap_or_else(|_| todo!());
        }
        if self.selector.is_enabled() {
            self.is_select.store(true, Ordering::Relaxed);
        }
    }

    pub fn run(&self) {
        loop {
            if self.is_select.load(Ordering::Relaxed) {
                self.mode_select();
            } else if self
                .operator
                .lock()
                .as_ref()
                .unwrap_or_else(|| todo!())
                .run()
                .is_ok()
            {
                let operator = self.operator.lock().take().expect("Should never be None");
                let operator = self.store.next(operator);
                *self.operator.lock() = Some(operator);
            }
        }
    }

    fn exchange_operator(&self, mode: Mode) {
        let operator = self.operator.lock().take().expect("Should never be None");
        let operator = self.store.exchange(operator, mode);
        *self.operator.lock() = Some(operator);
    }

    fn mode_select(&self) {
        //waiting for switch off
        while self.selector.is_enabled() {}

        self.selector.reset();

        //waiting for switch on
        while !self.selector.is_enabled() {}
        //waiting for switch off
        while self.selector.is_enabled() {}

        let mode = self.selector.mode();
        self.exchange_operator(mode);

        self.is_select.store(false, Ordering::Relaxed);
    }
}
