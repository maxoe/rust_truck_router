//! This module contains a few utilities to measure how long executing algorithms takes.
//! It utilizes the `time` crate.

use time_crate as time;

/// This function will measure how long it takes to execute the given lambda,
/// print the time and return the result of the lambda.
pub fn report_time<Out, F: FnOnce() -> Out>(name: &str, f: F) -> Out {
    let start = time::now();
    println!("starting {}", name);
    let res = f();
    println!("done {} - took: {}", name, (time::now() - start));
    res
}

/// This function will measure how long it takes to execute the given lambda
/// and return a tuple of the result of the lambda and a duration object.
pub fn measure<Out, F: FnOnce() -> Out>(f: F) -> (Out, time::Duration) {
    let start = time::now();
    let res = f();
    (res, time::now() - start)
}

/// A struct to repeatedly measure the time passed since the timer was started
#[derive(Debug)]
pub struct Timer {
    start: time::Tm
}

impl Default for Timer {
    fn default() -> Self {
        Self::new()
    }
}

impl Timer {
    /// Create and start a new `Timer`
    pub fn new() -> Timer {
        Timer { start: time::now() }
    }

    /// Reset the `Timer`
    pub fn restart(&mut self) {
        self.start = time::now();
    }

    /// Print the passed time in ms since the timer was started
    pub fn report_passed_ms(&self) {
        println!("{}ms", (time::now() - self.start).num_milliseconds());
    }

    /// Return the number of ms passed since the timer was started as a `i64`
    pub fn get_passed_ms(&self) -> i64 {
        (time::now() - self.start).num_milliseconds()
    }
}
