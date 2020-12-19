use alloc::rc::Rc;
use core::marker::PhantomData;
use core::sync::atomic::Ordering;

#[derive(Debug, Clone, Copy)]
pub struct NotFinishError;

pub trait Operator {
    type Mode;

    fn init(&self);
    fn tick(&self);
    fn run(&self) -> Result<Self::Mode, NotFinishError>;
}

pub trait OperatorStore<Mode> {
    fn get_operator(&self, mode: Mode) -> Rc<dyn Operator<Mode = Mode>>;
}

pub trait Atomic<T> {
    fn load(&self, order: Ordering) -> T;
    fn store(&self, val: T, order: Ordering);
}

pub enum SelectMode<Mode> {
    Select,
    Other(Mode),
}

pub trait Selector<Mode> {
    fn reset(&self);
    fn mode(&self) -> Mode;
    fn is_enabled(&self) -> bool;
}

pub struct Administrator<Mode, AtomicMode, ISelector, Store> {
    mode: AtomicMode,
    selector: ISelector,
    store: Store,
    _mode: PhantomData<fn() -> Mode>,
}

impl<Mode, AtomicMode, ISelector, Store> Administrator<Mode, AtomicMode, ISelector, Store>
where
    Mode: Default + From<u8>,
    AtomicMode: Atomic<SelectMode<Mode>>,
    ISelector: Selector<Mode>,
    Store: OperatorStore<Mode>,
{
    pub fn new(mode: AtomicMode, selector: ISelector, store: Store) -> Self {
        Self {
            mode,
            selector,
            store,
            _mode: PhantomData,
        }
    }

    //called by periodic interrupt
    pub fn tick(&self) {
        match self.mode.load(Ordering::Relaxed) {
            SelectMode::Select => return,
            SelectMode::Other(mode) => self.store.get_operator(mode).tick(),
        }
        if self.selector.is_enabled() {
            self.mode.store(SelectMode::Select, Ordering::Relaxed);
        }
    }

    pub fn run(&self) {
        loop {
            if let Ok(mode) = match self.mode.load(Ordering::Relaxed) {
                SelectMode::Select => self.mode_select(),
                SelectMode::Other(mode) => self.store.get_operator(mode).run(),
            } {
                self.mode.store(SelectMode::Other(mode), Ordering::Relaxed);
            }
        }
    }

    fn mode_select(&self) -> Result<Mode, NotFinishError> {
        //waiting for switch off
        while self.selector.is_enabled() {}

        self.selector.reset();

        //waiting for switch on
        while !self.selector.is_enabled() {}
        //waiting for switch off
        while self.selector.is_enabled() {}

        Ok(self.selector.mode())
    }
}
