use rand::Rng;
use rust_truck_router::{
    algo::{ch::*, core_ch::CoreContractionHierarchy, csp_2_core_ch_chpot::CSP2AstarCoreCHQuery},
    experiments::measurement::{MeasurementResult, EXPERIMENTS_N},
    io::*,
    types::*,
};
use std::{
    env,
    error::Error,
    fs::{self, File},
    io::{stdout, LineWriter, Write},
    path::Path,
    time::{Duration, Instant},
};

fn main() -> Result<(), Box<dyn Error>> {
    let n = EXPERIMENTS_N;
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let parent_dir_path = Path::new(arg);

    #[derive(Debug, Clone)]
    struct LocalMeasurementResult {
        pub speed_cap_kmh: Weight,
        pub time_ms: Duration,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "speed_cap_kmh,time_ms";

        fn get_header() -> String {
            format!("{}", Self::OWN_HEADER)
        }
        fn as_csv(&self) -> String {
            format!("{},{}", self.speed_cap_kmh, self.time_ms.as_secs_f64() * 1000.0)
        }
    }

    let subdir_prefix = "speed_cap_";
    let mut stat_logs = Vec::with_capacity(n);

    for path_res in fs::read_dir(parent_dir_path)? {
        let subdir = path_res?.file_name().to_str().unwrap().to_owned();

        if !subdir[..subdir_prefix.len()].eq(subdir_prefix) {
            continue;
        }

        let speed_cap_kmh = subdir[subdir_prefix.len()..].parse::<Weight>().unwrap();
        let path = parent_dir_path.join(subdir);

        let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
        let head = Vec::<NodeId>::load_from(path.join("head"))?;
        let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;

        let graph = OwnedGraph::new(first_out, head, travel_time);
        let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

        let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
        ch.check();
        core_ch.check();

        let mut core_ch_chpot_query = CSP2AstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
        core_ch_chpot_query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

        for i in 0..n {
            print!("Progress {}/{} - Speed Cap {}km/h\r", i, n, speed_cap_kmh);
            stdout().flush()?;

            let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
            let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

            let start = Instant::now();
            core_ch_chpot_query.init_new_s(s);
            core_ch_chpot_query.init_new_t(t);
            let _core_ch_chpot_dist = core_ch_chpot_query.run_query();
            let core_ch_chpot_time = start.elapsed();
            stat_logs.push(LocalMeasurementResult {
                speed_cap_kmh,
                time_ms: core_ch_chpot_time,
            });
        }

        println!("Progress {}/{} - Speed Cap {}km/h\r", n, n, speed_cap_kmh);
    }

    let file = File::create("thesis_speed_cap-csp_2-".to_owned() + parent_dir_path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
