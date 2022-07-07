use rand::Rng;
use rust_truck_router::{
    algo::{ch::*, core_ch::CoreContractionHierarchy, csp_core_ch_chpot::CSPAstarCoreCHQuery},
    experiments::measurement::{MeasurementResult, EXPERIMENTS_N},
    io::*,
    types::*,
};
use std::{
    env,
    error::Error,
    fs::File,
    io::{stdout, LineWriter, Write},
    path::Path,
    rc::Rc,
    time::{Duration, Instant},
};

fn main() -> Result<(), Box<dyn Error>> {
    let n = EXPERIMENTS_N;
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let parking_rc = Rc::new(is_parking_node);

    let graph = OwnedGraph::new(first_out, head, travel_time);
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

    let rk_logfile = std::fs::read_to_string(path.join("core_experiment.log"))?;
    let mut run_list = Vec::new();

    for line in rk_logfile.lines().skip(1) {
        let values: Vec<_> = line.split(',').collect();
        run_list.push((
            values[0].parse::<f64>().unwrap(),
            values[1].parse::<usize>().unwrap(),
            values[2].parse::<f64>().unwrap(),
            path.join(values[3].to_owned()),
        ));
    }

    #[derive(Debug, Clone)]
    struct LocalMeasurementResult {
        pub rel_core_size: f64,
        pub abs_core_size: usize,
        pub construction_time_ms: f64,
        pub time_ms: Duration,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "rel_core_size,abs_core_size,construction_time_ms,time_ms";

        fn get_header() -> String {
            format!("{}", Self::OWN_HEADER)
        }
        fn as_csv(&self) -> String {
            format!(
                "{},{},{},{}",
                self.rel_core_size,
                self.abs_core_size,
                self.construction_time_ms,
                self.time_ms.as_secs_f64() * 1000.0
            )
        }
    }
    let mut stat_logs = Vec::with_capacity(n);

    for (rel_core_size, abs_core_size, construction_time_ms, core_ch_dir) in run_list {
        let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(core_ch_dir)?;
        ch.check();
        core_ch.check();

        let mut core_ch_chpot_query = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
        core_ch_chpot_query.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);
        core_ch_chpot_query.set_custom_reset_nodes(parking_rc.clone());

        for i in 0..n {
            print!("Progress {}/{} - Core Size {}%\r", i, n, rel_core_size * 100.0);
            stdout().flush()?;

            let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
            let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

            let start = Instant::now();
            core_ch_chpot_query.init_new_s(s);
            core_ch_chpot_query.init_new_t(t);
            let _core_ch_chpot_dist = core_ch_chpot_query.run_query();
            let core_ch_chpot_time = start.elapsed();
            stat_logs.push(LocalMeasurementResult {
                rel_core_size,
                abs_core_size,
                construction_time_ms,
                time_ms: core_ch_chpot_time,
            });
        }

        print!("Progress {}/{} - Core Size {}%\r", n, n, rel_core_size * 100.0);
    }

    let file = File::create("thesis_core_sizes-csp-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
