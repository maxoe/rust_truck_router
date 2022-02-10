use std::slice;

use crate::types::DefaultReset;

#[derive(Clone, Debug)]
pub struct TimestampedVector<T> {
    run: Vec<usize>,
    run_count: usize,
    vector: Vec<T>,
    default: T,
}

impl<T: DefaultReset> TimestampedVector<T> {
    pub fn new() -> Self {
        Self {
            run: Vec::new(),
            run_count: 1,
            vector: Vec::new(),
            default: T::DEFAULT,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            run: Vec::with_capacity(capacity),
            run_count: 1,
            vector: Vec::with_capacity(capacity),
            default: T::DEFAULT,
        }
    }

    pub fn with_size(size: usize) -> Self {
        Self {
            run: vec![0; size],
            run_count: 1,
            vector: vec![T::DEFAULT; size],
            default: T::DEFAULT,
        }
    }
    pub fn from_vec(v: Vec<T>) -> Self {
        Self {
            run: vec![0; v.len()],
            run_count: 0,
            vector: v,
            default: T::DEFAULT,
        }
    }

    pub fn reset(&mut self) {
        let (new_count, overflowed) = self.run_count.overflowing_add(1);

        if overflowed {
            for e in &mut self.vector {
                e.reset();
            }
        }
        self.run_count = new_count;
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
    pub fn data(&self) -> &[T] {
        &self.vector
    }

    pub fn iter(&self) -> slice::Iter<T> {
        self.vector.iter()
    }

    pub fn is_set(&self, i: usize) -> bool {
        self.run[i] == self.run_count
    }
}
