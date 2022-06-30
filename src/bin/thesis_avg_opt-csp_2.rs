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
    algo::{
        ch::ContractionHierarchy, core_ch::CoreContractionHierarchy, csp_2_core_ch_chpot::CSP2AstarCoreCHQuery,
        csp_2_core_ch_chpot_no_bw::CSP2AstarCoreCHQueryNoBw, csp_2_core_ch_chpot_no_bw_no_prune::CSP2AstarCoreCHQueryNoBwNoPrune,
        csp_2_core_ch_chpot_no_prune::CSP2AstarCoreCHQueryNoPrune,
    },
    experiments::measurement::{MeasurementResult, EXPERIMENTS_N},
    types::{Graph, NodeId, OwnedGraph, EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME},
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);

    let graph = OwnedGraph::load_from_routingkit_dir(path)?;
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;

    let mut query = CSP2AstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let mut query_no_bw = CSP2AstarCoreCHQueryNoBw::new(core_ch.borrow(), ch.borrow());
    query_no_bw.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let mut query_no_pr = CSP2AstarCoreCHQueryNoPrune::new(core_ch.borrow(), ch.borrow());
    query_no_pr.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let mut query_no_bw_no_pr = CSP2AstarCoreCHQueryNoBwNoPrune::new(core_ch.borrow(), ch.borrow());
    query_no_bw_no_pr.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let n = EXPERIMENTS_N;

    #[derive(Debug, Clone)]
    struct LocalMeasurementResult {
        pub algo: String,
        pub time: Duration,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "algo,time_ms";

        fn get_header() -> String {
            format!("{}", Self::OWN_HEADER)
        }
        fn as_csv(&self) -> String {
            format!("{},{}", self.algo, self.time.as_secs_f64() * 1000.0)
        }
    }

    let mut stat_logs = Vec::with_capacity(n);

    for _i in 0..n {
        let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
        let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

        print!("\rProgress {}/{} from {} to {} - A* Core CH FULL\t\t\t\t\t\t\t", _i, n, s, t);
        stdout().flush()?;

        let start = Instant::now();
        query.init_new_s(s);
        query.init_new_t(t);
        let _core_ch_chpot_dist = query.run_query();
        let core_ch_chpot_time = start.elapsed();

        print!("\rProgress {}/{} from {} to {} - A* Core CH NO BW NO PRUNE\t\t\t\t\t\t\t", _i, n, s, t);
        stdout().flush()?;

        let start = Instant::now();
        query_no_bw_no_pr.init_new_s(s);
        query_no_bw_no_pr.init_new_t(t);
        let _no_bw_no_prune_dist = query_no_bw_no_pr.run_query();
        let no_bw_no_prune_time = start.elapsed();

        print!("\rProgress {}/{} from {} to {} - A* Core CH NO BW\t\t\t\t\t\t\t", _i, n, s, t);
        stdout().flush()?;

        let start = Instant::now();
        query_no_bw.init_new_s(s);
        query_no_bw.init_new_t(t);
        let _no_bw_dist = query_no_bw.run_query();
        let no_bw_time = start.elapsed();

        print!("\rProgress {}/{} from {} to {} - A* Core CH NO PRUNE\t\t\t\t\t\t\t", _i, n, s, t);
        stdout().flush()?;

        let start = Instant::now();
        query_no_pr.init_new_s(s);
        query_no_pr.init_new_t(t);
        let _no_prune_dist = query_no_pr.run_query();
        let no_prune_time = start.elapsed();

        stat_logs.push(LocalMeasurementResult {
            algo: String::from("no_bw"),
            time: no_bw_time,
        });

        stat_logs.push(LocalMeasurementResult {
            algo: String::from("no_bw_no_prune"),
            time: no_bw_no_prune_time,
        });

        stat_logs.push(LocalMeasurementResult {
            algo: String::from("no_prune"),
            time: no_prune_time,
        });

        stat_logs.push(LocalMeasurementResult {
            algo: String::from("core_ch_chpot"),
            time: core_ch_chpot_time,
        });
    }

    println!("\rProgress {}/{}", n, n);

    let file = File::create("thesis_avg_opt-csp_2-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
