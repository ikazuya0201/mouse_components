use core::cell::UnsafeCell;
use core::fmt;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, Ordering::*};

#[derive(Debug)]
pub struct Mutex<T> {
    locked: AtomicBool,
    inner: UnsafeCell<T>,
}

#[derive(Debug, Clone, Copy)]
pub struct MutexError;

pub struct MutexGuard<'a, T>
where
    T: 'a,
{
    mutex: &'a Mutex<T>,
}

impl<T> Mutex<T> {
    pub fn new(data: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            inner: UnsafeCell::new(data),
        }
    }

    pub fn try_lock<'a>(&'a self) -> Result<MutexGuard<'a, T>, MutexError> {
        if self.locked.swap(true, Acquire) {
            Err(MutexError)
        } else {
            Ok(MutexGuard { mutex: self })
        }
    }

    //spin lock
    pub fn lock<'a>(&'a self) -> MutexGuard<'a, T> {
        loop {
            if let Ok(m) = self.try_lock() {
                break m;
            }
        }
    }
}

unsafe impl<T> Send for Mutex<T> {}

impl<T> Drop for Mutex<T> {
    fn drop(&mut self) {
        unsafe { self.inner.get().drop_in_place() }
    }
}

impl<'a, T> Deref for MutexGuard<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { &*self.mutex.inner.get() }
    }
}

impl<'a, T> DerefMut for MutexGuard<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { &mut *self.mutex.inner.get() }
    }
}

impl<'a, T> fmt::Debug for MutexGuard<'a, T>
where
    T: fmt::Debug,
{
    fn fmt(&self, fmtr: &mut fmt::Formatter) -> fmt::Result {
        write!(fmtr, "{:?}", &**self)
    }
}

impl<'a, T> Drop for MutexGuard<'a, T> {
    fn drop(&mut self) {
        self.mutex.locked.swap(false, Release);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mutex_try_lock() {
        let mutex = Mutex::new(10usize);
        {
            let mut value = mutex.try_lock().unwrap();
            assert_eq!(*value, 10);
            assert!(mutex.try_lock().is_err());
            *value = 20;
        }
        let value = mutex.try_lock().unwrap();
        assert_eq!(*value, 20);
    }
}
