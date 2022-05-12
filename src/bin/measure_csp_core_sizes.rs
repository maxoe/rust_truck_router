use rand::Rng;
use rust_truck_router::{
    algo::{ch::*, core_ch::CoreContractionHierarchy, csp_core_ch_chpot::CSPAstarCoreCHQuery},
    experiments::measurement::MeasurementResult,
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
    let n = 100;
    let degree_limits = vec![100u32, 200, 300, 400, 500];
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let parking_rc = Rc::new(is_parking_node);

    let graph = OwnedGraph::new(first_out, head, travel_time);
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

    #[derive(Debug, Clone)]
    struct LocalMeasurementResult {
        pub degree_limit: u32,
        pub dist: Option<Weight>,
        pub time_ms: Duration,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "degree_limit,path_distance,time_ms";

        fn get_header() -> String {
            format!("{}", Self::OWN_HEADER)
        }
        fn as_csv(&self) -> String {
            format!(
                "{},{},{}",
                self.degree_limit,
                self.dist.map_or("NaN".to_owned(), |d| { d.to_string() }),
                self.time_ms.as_secs_f64() * 1000.0
            )
        }
    }
    let mut stat_logs = Vec::with_capacity(n);

    for current_degree_limit in degree_limits {
        let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch_".to_owned() + &current_degree_limit.to_string()))?;
        ch.check();
        core_ch.check();

        let mut core_ch_chpot_query = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
        core_ch_chpot_query.set_restriction(16_200_000, 2_700_000);
        core_ch_chpot_query.set_custom_reset_nodes(parking_rc.clone());

        for _i in 0..n {
            print!("Progress {}/{}\r", _i, n);
            stdout().flush()?;

            let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
            let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

            let start = Instant::now();
            core_ch_chpot_query.init_new_s(s);
            core_ch_chpot_query.init_new_t(t);
            let core_ch_chpot_dist = core_ch_chpot_query.run_query();
            let core_ch_chpot_time = start.elapsed();

            // assert_eq!(astar_dist, core_ch_dist);
            // assert_eq!(core_ch_dist, core_ch_chpot_dist);

            stat_logs.push(LocalMeasurementResult {
                degree_limit: current_degree_limit,
                dist: core_ch_chpot_dist,
                time_ms: core_ch_chpot_time,
            });
        }
        println!("Progress {}/{}", n, n);
    }

    let file = File::create("measure_csp_core_sizes-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }
    Ok(())
}
