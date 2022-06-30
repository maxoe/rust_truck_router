//! # Example
//!
//! ```
//! use rust_truck_router::timestamped_vector::*;
//!
//! let mut v : TimestampedVector<u32, usize> = TimestampedVector::with_size(10);
//!
//! for i in 0..10 {
//!     *v.get_mut(i) = 5;
//! }
//!
//! for i in 0..10 {
//!     assert_eq!(*v.get(i), 5);
//! }
//!
//! for i in 0..10 {
//!     v.set(i, 6)
//! }
//!
//! for i in 0..10 {
//!     assert_eq!(*v.get(i), 6);
//! }
//!
//! v.reset();
//!
//! for i in 0..10 {
//!     assert!(!v.is_set(i));
//! }
//!
//! for i in 0..10 {
//!     assert_ne!(*v.get(i), 6);
//! }
//!
//! ```
//!
//!

use num::{traits::ops::overflowing::OverflowingAdd, Integer, Unsigned};

use crate::types::DefaultReset;

#[derive(Clone, Debug)]
pub struct TimestampedVector<T, U = usize> {
    // u16 for more frequent resets to save memory
    pub run: Vec<U>,
    run_count: U,
    pub vector: Vec<T>,
    default: T,
}

impl<T: DefaultReset, U: Unsigned + Integer + OverflowingAdd + Clone + Copy> TimestampedVector<T, U> {
    pub fn new() -> Self {
        Self {
            run: Vec::new(),
            run_count: U::one(),
            vector: Vec::new(),
            default: T::DEFAULT,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            run: Vec::with_capacity(capacity),
            run_count: U::one(),
            vector: Vec::with_capacity(capacity),
            default: T::DEFAULT,
        }
    }

    pub fn with_size(size: usize) -> Self {
        Self {
            run: vec![U::zero(); size],
            run_count: U::one(),
            vector: vec![T::DEFAULT; size],
            default: T::DEFAULT,
        }
    }
    pub fn from_vec(v: Vec<T>) -> Self {
        Self {
            run: vec![U::zero(); v.len()],
            run_count: U::zero(),
            vector: v,
            default: T::DEFAULT,
        }
    }

    pub fn reset(&mut self) {
        let (new_count, overflowed) = self.run_count.overflowing_add(&U::one());

        if overflowed {
            self.clean();
        }
        self.run_count = new_count;
    }

    pub fn clean(&mut self) {
        for e in &mut self.vector {
            e.reset();
        }
    }

    pub fn get(&self, i: usize) -> &T {
        if self.run[i] == self.run_count {
            &self.vector[i]
        } else {
            &self.default
        }
    }

    pub fn get_mut(&mut self, i: usize) -> &mut T {
        if self.run[i] != self.run_count {
            self.vector[i].reset();
            self.run[i] = self.run_count;
        }

        &mut self.vector[i]
    }

    pub fn set(&mut self, i: usize, e: T) {
        self.vector[i] = e;
        self.run[i] = self.run_count;
    }

    pub fn len(&self) -> usize {
        self.vector.len()
    }

    pub fn is_empty(&self) -> bool {
        self.vector.is_empty()
    }

    pub fn is_set(&self, i: usize) -> bool {
        self.run[i] == self.run_count
    }
}
