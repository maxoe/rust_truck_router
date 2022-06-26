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
        ch::ContractionHierarchy,
        ch_potential::CHPotential,
        core_ch::CoreContractionHierarchy,
        csp_2::{TwoRestrictionDijkstra, TwoRestrictionDijkstraData},
        csp_2_bidir_chpot::CSP2BidirAstarCHPotQuery,
        csp_2_core_ch::CSP2CoreCHQuery,
        csp_2_core_ch_chpot::CSP2AstarCoreCHQuery,
    },
    experiments::measurement::{CSP2MeasurementResult, CSPMeasurementResult, MeasurementResult, LONG_QUERY_TIMEOUT_SECS},
    io::load_routingkit_bitvector,
    types::{Graph, NodeId, OwnedGraph, EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME},
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);

    let graph = OwnedGraph::load_from_routingkit_dir(path)?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let bw_graph = OwnedGraph::reverse(graph.borrow());
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;

    let mut astar_state = TwoRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    astar_state.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);
    let astar = TwoRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    let mut bidir_astar_query = CSP2BidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());
    bidir_astar_query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let mut core_ch_query = CSP2CoreCHQuery::new(core_ch.borrow());
    core_ch_query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let mut core_ch_chpot_query = CSP2AstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    core_ch_chpot_query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let mut n = 100;

    #[derive(Debug, Clone)]
    struct LocalMeasurementResult {
        pub algo: String,
        standard: CSP2MeasurementResult,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "algo";

        fn get_header() -> String {
            format!("{},{}", Self::OWN_HEADER, CSP2MeasurementResult::get_header())
        }
        fn as_csv(&self) -> String {
            format!("{},{}", self.algo, self.standard.as_csv())
        }
    }

    let mut stat_logs = Vec::with_capacity(n);

    let mut timeouts = 0;

    for _i in 0..n {
        let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
        let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

        print!("Progress {}/{} from {} to {} - A*             \r", _i, n, s, t);
        stdout().flush()?;

        let start = Instant::now();
        astar_state.init_new_s(s);

        let res = tokio::time::timeout(Duration::from_secs(LONG_QUERY_TIMEOUT_SECS), async { astar.dist_query(&mut astar_state, t) }).await;

        if res.is_err() {
            println!("A* timed out, skipping query                  ");
            timeouts += 1;
            n += 1;
            continue;
        }

        let astar_dist = res.unwrap();
        let astar_time = start.elapsed();

        print!("Progress {}/{} from {} to {} - Bidir A*       \r", _i, n, s, t);
        stdout().flush()?;

        let start = Instant::now();
        bidir_astar_query.init_new_s(s);
        bidir_astar_query.init_new_t(t);

        let res = tokio::time::timeout(Duration::from_secs(LONG_QUERY_TIMEOUT_SECS), async {
            bidir_astar_query.run_query();
        })
        .await;

        if res.is_err() {
            println!("Bidirectional A* timed out, skipping query    ");
            timeouts += 1;
            n += 1;
            continue;
        }

        let _bidir_astar_dist = res.unwrap();
        let bidir_astar_time = start.elapsed();

        print!("Progress {}/{} from {} to {} - Core CH        \r", _i, n, s, t);
        stdout().flush()?;

        let start = Instant::now();
        core_ch_query.init_new_s(s);
        core_ch_query.init_new_t(t);
        let core_ch_dist = core_ch_query.run_query();
        let core_ch_time = start.elapsed();

        print!("Progress {}/{} from {} to {} - A* Core CH\t\t\t\t\t\t\t\r", _i, n, s, t);
        stdout().flush()?;

        let start = Instant::now();
        core_ch_chpot_query.init_new_s(s);
        core_ch_chpot_query.init_new_t(t);
        let core_ch_chpot_dist = core_ch_chpot_query.run_query();
        let core_ch_chpot_time = start.elapsed();

        if astar_dist.is_some() {
            let path = astar_state.current_best_path_to(t, true).unwrap();
            let number_flagged_nodes = astar.flagged_nodes_on_path(&path);
            let number_pauses = astar.reset_nodes_on_path(&path);

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("astar_chpot"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: astar_state.num_queue_pushes,
                        num_settled: astar_state.num_settled,
                        num_labels_propagated: astar_state.num_labels_propagated,
                        num_labels_reset: astar_state.num_labels_reset,
                        num_nodes_searched: astar_state.get_number_of_visited_nodes(),
                        time: astar_time,
                        path_distance: astar_dist,
                        path_number_nodes: Some(path.0.len()),
                        path_number_flagged_nodes: Some(number_flagged_nodes.len()),
                    },
                    path_number_short_pauses: Some(number_pauses.0.len()),
                    path_number_long_pauses: Some(number_pauses.1.len()),
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("astar_bidir_chpot"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: astar_state.num_queue_pushes,
                        num_settled: astar_state.num_settled,
                        num_labels_propagated: astar_state.num_labels_propagated,
                        num_labels_reset: astar_state.num_labels_reset,
                        num_nodes_searched: astar_state.get_number_of_visited_nodes(),
                        time: bidir_astar_time,
                        path_distance: astar_dist,
                        path_number_nodes: Some(path.0.len()),
                        path_number_flagged_nodes: Some(number_flagged_nodes.len()),
                    },
                    path_number_short_pauses: Some(number_pauses.0.len()),
                    path_number_long_pauses: Some(number_pauses.1.len()),
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: astar_state.num_queue_pushes,
                        num_settled: astar_state.num_settled,
                        num_labels_propagated: astar_state.num_labels_propagated,
                        num_labels_reset: astar_state.num_labels_reset,
                        num_nodes_searched: astar_state.get_number_of_visited_nodes(),
                        time: core_ch_time,
                        path_distance: core_ch_dist,
                        path_number_nodes: Some(path.0.len()),
                        path_number_flagged_nodes: Some(number_flagged_nodes.len()),
                    },
                    path_number_short_pauses: Some(number_pauses.0.len()),
                    path_number_long_pauses: Some(number_pauses.1.len()),
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch_chpot"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: astar_state.num_queue_pushes,
                        num_settled: astar_state.num_settled,
                        num_labels_propagated: astar_state.num_labels_propagated,
                        num_labels_reset: astar_state.num_labels_reset,
                        num_nodes_searched: astar_state.get_number_of_visited_nodes(),
                        time: core_ch_chpot_time,
                        path_distance: core_ch_chpot_dist,
                        path_number_nodes: Some(path.0.len()),
                        path_number_flagged_nodes: Some(number_flagged_nodes.len()),
                    },
                    path_number_short_pauses: Some(number_pauses.0.len()),
                    path_number_long_pauses: Some(number_pauses.1.len()),
                },
            });
        } else {
            stat_logs.push(LocalMeasurementResult {
                algo: String::from("astar_chpot"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: astar_state.num_queue_pushes,
                        num_settled: astar_state.num_settled,
                        num_labels_propagated: astar_state.num_labels_propagated,
                        num_labels_reset: astar_state.num_labels_reset,
                        num_nodes_searched: astar_state.get_number_of_visited_nodes(),
                        time: astar_time,
                        path_distance: None,
                        path_number_nodes: None,
                        path_number_flagged_nodes: None,
                    },
                    path_number_short_pauses: None,
                    path_number_long_pauses: None,
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("astar_bidir_chpot"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: astar_state.num_queue_pushes,
                        num_settled: astar_state.num_settled,
                        num_labels_propagated: astar_state.num_labels_propagated,
                        num_labels_reset: astar_state.num_labels_reset,
                        num_nodes_searched: astar_state.get_number_of_visited_nodes(),
                        time: bidir_astar_time,
                        path_distance: None,
                        path_number_nodes: None,
                        path_number_flagged_nodes: None,
                    },
                    path_number_short_pauses: None,
                    path_number_long_pauses: None,
                },
            });
            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: astar_state.num_queue_pushes,
                        num_settled: astar_state.num_settled,
                        num_labels_propagated: astar_state.num_labels_propagated,
                        num_labels_reset: astar_state.num_labels_reset,
                        num_nodes_searched: astar_state.get_number_of_visited_nodes(),
                        time: core_ch_time,
                        path_distance: None,
                        path_number_nodes: None,
                        path_number_flagged_nodes: None,
                    },
                    path_number_short_pauses: None,
                    path_number_long_pauses: None,
                },
            });
            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch_chpot"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: astar_state.num_queue_pushes,
                        num_settled: astar_state.num_settled,
                        num_labels_propagated: astar_state.num_labels_propagated,
                        num_labels_reset: astar_state.num_labels_reset,
                        num_nodes_searched: astar_state.get_number_of_visited_nodes(),
                        time: core_ch_chpot_time,
                        path_distance: None,
                        path_number_nodes: None,
                        path_number_flagged_nodes: None,
                    },
                    path_number_short_pauses: None,
                    path_number_long_pauses: None,
                },
            });
        }
    }
    println!("Progress {}/{}", n, n);
    println!("Timeouts: {}", timeouts);

    let file = File::create("thesis_avg_all-csp_2-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
