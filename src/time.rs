use time_crate as time;

#[allow(dead_code)]
pub fn report_time<Out, F: FnOnce() -> Out>(name: &str, f: F) -> Out {
    let start = time::now();
    println!("starting {}", name);
    let res = f();
    println!("done {} - took: {}", name, (time::now() - start));
    res
}

#[allow(dead_code)]
pub fn measure<Out, F: FnOnce() -> Out>(f: F) -> (Out, time::Duration) {
    let start = time::now();
    let res = f();
    (res, time::now() - start)
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct Timer {
    start: time::Tm
}

impl Timer {
    #[allow(dead_code)]
    pub fn new() -> Timer {
        Timer { start: time::now() }
    }

    #[allow(dead_code)]
    pub fn restart(&mut self) {
        self.start = time::now();
    }

    #[allow(dead_code)]
    pub fn report_passed_ms(&self) {
        println!("{}ms", (time::now() - self.start).num_milliseconds());
    }

    #[allow(dead_code)]
    pub fn get_passed_ms(&self) -> i64 {
        (time::now() - self.start).num_milliseconds()
    }
}
