use std::{
    env,
    error::Error,
    fs::File,
    io::{stdout, LineWriter, Write},
    path::Path,
    time::{Duration, Instant},
};

use rand::Rng;
use rust_truck_router::{
    algo::{ch::ContractionHierarchy, core_ch::CoreContractionHierarchy, csp_2_core_ch::CSP2CoreCHQuery, csp_2_core_ch_chpot::CSP2AstarCoreCHQuery},
    experiments::measurement::{MeasurementResult, EXPERIMENTS_BASE_N},
    types::{Graph, NodeId, OwnedGraph, Weight, EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME},
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);

    let graph = OwnedGraph::load_from_routingkit_dir(path)?;
    // let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    // let bw_graph = OwnedGraph::reverse(graph.borrow());
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;

    let mut core_ch_query = CSP2CoreCHQuery::new(core_ch.borrow());
    core_ch_query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let mut core_ch_chpot_query = CSP2AstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    core_ch_chpot_query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let n = EXPERIMENTS_BASE_N;

    #[derive(Debug, Clone)]
    struct LocalMeasurementResult {
        pub algo: String,
        pub time: Duration,
        pub path_distance: Option<Weight>,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "algo,time_ms,path_distance";

        fn get_header() -> String {
            format!("{}", Self::OWN_HEADER)
        }
        fn as_csv(&self) -> String {
            format!(
                "{},{},{}",
                self.algo,
                self.time.as_secs_f64() * 1000.0,
                self.path_distance.map_or("NaN".to_owned(), |v| v.to_string())
            )
        }
    }

    let mut stat_logs = Vec::with_capacity(n);

    for _i in 0..n {
        let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
        let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

        print!("\rProgress {}/{} from {} to {} - Core CH        ", _i, n, s, t);
        stdout().flush()?;

        let start = Instant::now();
        core_ch_query.init_new_s(s);
        core_ch_query.init_new_t(t);
        let core_ch_dist = core_ch_query.run_query();
        let core_ch_time = start.elapsed();

        print!("\rProgress {}/{} from {} to {} - A* Core CH\t\t\t\t\t\t\t", _i, n, s, t);
        stdout().flush()?;

        let start = Instant::now();
        core_ch_chpot_query.init_new_s(s);
        core_ch_chpot_query.init_new_t(t);
        let core_ch_chpot_dist = core_ch_chpot_query.run_query();
        let core_ch_chpot_time = start.elapsed();

        stat_logs.push(LocalMeasurementResult {
            algo: String::from("core_ch"),
            time: core_ch_time,
            path_distance: core_ch_dist,
        });

        stat_logs.push(LocalMeasurementResult {
            algo: String::from("core_ch_chpot"),
            time: core_ch_chpot_time,
            path_distance: core_ch_chpot_dist,
        });
    }

    println!("\rProgress {}/{}", n, n);

    let file = File::create("thesis_avg_fast-csp_2-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
