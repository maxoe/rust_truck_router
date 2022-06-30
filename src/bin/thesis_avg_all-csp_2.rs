use std::{
    env,
    error::Error,
    fs::File,
    io::{stdout, LineWriter, Write},
    path::Path,
    time::Instant,
};

use rand::Rng;
use rust_truck_router::{
    algo::{
        ch::ContractionHierarchy,
        ch_potential::CHPotential,
        core_ch::CoreContractionHierarchy,
        csp_2::{TwoRestrictionDijkstra, TwoRestrictionDijkstraData},
        csp_2_bidir::CSP2BidirQuery,
        csp_2_bidir_chpot::CSP2BidirAstarCHPotQuery,
        csp_2_core_ch::CSP2CoreCHQuery,
        csp_2_core_ch_chpot::CSP2AstarCoreCHQuery,
    },
    experiments::measurement::{CSP2MeasurementResult, CSPMeasurementResult, MeasurementResult, EXPERIMENTS_N},
    io::load_routingkit_bitvector,
    types::{Graph, NodeId, OwnedGraph, EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME},
};

fn main() -> Result<(), Box<dyn Error>> {
    let path_arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(path_arg);
    // let mut algos: Vec<String> = env::args().skip(2).collect();

    // if algos.is_empty() {
    //     algos = vec![
    //         "dijkstra".to_owned(),
    //         "astar_chpot".to_owned(),
    //         "dijkstra_bidir".to_owned(),
    //         "astar_bidir_chpot".to_owned(),
    //         "core_ch".to_owned(),
    //         "core_ch_chpot".to_owned(),
    //     ];
    // }
    let n = EXPERIMENTS_N;

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

    let mut queries = Vec::with_capacity(n);
    let mut has_path = Vec::with_capacity(n);
    let mut stat_logs = Vec::with_capacity(n);
    let num_nodes;
    let num_edges;

    {
        let graph = OwnedGraph::load_from_routingkit_dir(path)?;
        num_nodes = graph.num_nodes();
        num_edges = graph.num_arcs();

        for _ in 0..n {
            queries.push((
                rand::thread_rng().gen_range(0..num_nodes as NodeId),
                rand::thread_rng().gen_range(0..num_nodes as NodeId),
            ));
        }
    }
    {
        let graph = OwnedGraph::load_from_routingkit_dir(path)?;
        let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
        let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

        let mut astar_state = TwoRestrictionDijkstraData::new_with_potential(num_nodes, CHPotential::from_ch(ch.borrow()));
        astar_state.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);
        let astar = TwoRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

        for (i, &(s, t)) in queries.iter().enumerate() {
            print!("\rProgress {}/{} from {} to {} - A*             ", i, n, s, t);
            stdout().flush()?;

            let start = Instant::now();
            astar_state.init_new_s(s);
            let astar_dist = astar.dist_query(&mut astar_state, t);
            let astar_time = start.elapsed();

            if astar_dist.is_some() {
                has_path.push(true);
                let path = astar_state.current_best_path_to(t, true).unwrap();
                let number_flagged_nodes = astar.flagged_nodes_on_path(&path);
                let (number_short_pauses, number_long_pauses) = astar.reset_nodes_on_path(&path);

                stat_logs.push(LocalMeasurementResult {
                    algo: String::from("astar_chpot"),
                    standard: CSP2MeasurementResult {
                        standard: CSPMeasurementResult {
                            graph_num_nodes: num_nodes,
                            graph_num_edges: num_edges,
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
                        path_number_short_pauses: Some(number_short_pauses.len()),
                        path_number_long_pauses: Some(number_long_pauses.len()),
                    },
                });
            } else {
                has_path.push(false);

                stat_logs.push(LocalMeasurementResult {
                    algo: String::from("astar_chpot"),
                    standard: CSP2MeasurementResult {
                        standard: CSPMeasurementResult {
                            graph_num_nodes: num_nodes,
                            graph_num_edges: num_edges,
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
            }

            astar_state.clean();
        }
    }

    {
        let graph = OwnedGraph::load_from_routingkit_dir(path)?;
        let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
        let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
        let bw_graph = OwnedGraph::reverse(graph.borrow());
        let mut bidir_astar_query = CSP2BidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());
        bidir_astar_query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

        for (i, &(s, t)) in queries.iter().enumerate() {
            print!("\rProgress {}/{} from {} to {} - Bidir A*       ", i, n, s, t);
            stdout().flush()?;

            let start = Instant::now();
            bidir_astar_query.init_new_s(s);
            bidir_astar_query.init_new_t(t);
            let _bidir_astar_dist = bidir_astar_query.run_query();
            let bidir_astar_time = start.elapsed();

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("astar_bidir_chpot"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: num_nodes,
                        graph_num_edges: num_edges,
                        num_queue_pushes: stat_logs[i].standard.standard.num_queue_pushes,
                        num_settled: stat_logs[i].standard.standard.num_settled,
                        num_labels_propagated: stat_logs[i].standard.standard.num_labels_propagated,
                        num_labels_reset: stat_logs[i].standard.standard.num_labels_reset,
                        num_nodes_searched: stat_logs[i].standard.standard.num_nodes_searched,
                        time: bidir_astar_time,
                        path_distance: stat_logs[i].standard.standard.path_distance,
                        path_number_nodes: stat_logs[i].standard.standard.path_number_nodes,
                        path_number_flagged_nodes: stat_logs[i].standard.standard.path_number_flagged_nodes,
                    },
                    path_number_short_pauses: stat_logs[i].standard.path_number_short_pauses,
                    path_number_long_pauses: stat_logs[i].standard.path_number_long_pauses,
                },
            });

            bidir_astar_query.clean();
        }
    }

    {
        let graph = OwnedGraph::load_from_routingkit_dir(path)?;
        let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
        let mut dijkstra_state = TwoRestrictionDijkstraData::new(graph.num_nodes());
        dijkstra_state.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);
        let dijkstra = TwoRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

        for (i, &(s, t)) in queries.iter().enumerate() {
            print!("\rProgress {}/{} from {} to {} - Dijkstra       ", i, n, s, t);
            stdout().flush()?;

            let start = Instant::now();
            dijkstra_state.init_new_s(s);
            let _dijkstra_dist = dijkstra.dist_query(&mut dijkstra_state, t);
            let dijkstra_time = start.elapsed();

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("dijkstra"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: num_nodes,
                        graph_num_edges: num_edges,
                        num_queue_pushes: stat_logs[i].standard.standard.num_queue_pushes,
                        num_settled: stat_logs[i].standard.standard.num_settled,
                        num_labels_propagated: stat_logs[i].standard.standard.num_labels_propagated,
                        num_labels_reset: stat_logs[i].standard.standard.num_labels_reset,
                        num_nodes_searched: stat_logs[i].standard.standard.num_nodes_searched,
                        time: dijkstra_time,
                        path_distance: stat_logs[i].standard.standard.path_distance,
                        path_number_nodes: stat_logs[i].standard.standard.path_number_nodes,
                        path_number_flagged_nodes: stat_logs[i].standard.standard.path_number_flagged_nodes,
                    },
                    path_number_short_pauses: stat_logs[i].standard.path_number_short_pauses,
                    path_number_long_pauses: stat_logs[i].standard.path_number_long_pauses,
                },
            });

            dijkstra_state.clean();
        }
    }

    {
        let graph = OwnedGraph::load_from_routingkit_dir(path)?;
        let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
        let bw_graph = OwnedGraph::reverse(graph.borrow());
        let mut bidir_dijkstra_query = CSP2BidirQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node);
        bidir_dijkstra_query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

        for (i, &(s, t)) in queries.iter().enumerate() {
            print!("\rProgress {}/{} from {} to {} - Bidir. Dijkstra", i, n, s, t);
            stdout().flush()?;

            let start = Instant::now();
            bidir_dijkstra_query.init_new_s(s);
            bidir_dijkstra_query.init_new_t(t);
            let _bidir_dijkstra_dist = bidir_dijkstra_query.run_query();
            let bidir_dijkstra_time = start.elapsed();

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("dijkstra_bidir"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: num_nodes,
                        graph_num_edges: num_edges,
                        num_queue_pushes: stat_logs[i].standard.standard.num_queue_pushes,
                        num_settled: stat_logs[i].standard.standard.num_settled,
                        num_labels_propagated: stat_logs[i].standard.standard.num_labels_propagated,
                        num_labels_reset: stat_logs[i].standard.standard.num_labels_reset,
                        num_nodes_searched: stat_logs[i].standard.standard.num_nodes_searched,
                        time: bidir_dijkstra_time,
                        path_distance: stat_logs[i].standard.standard.path_distance,
                        path_number_nodes: stat_logs[i].standard.standard.path_number_nodes,
                        path_number_flagged_nodes: stat_logs[i].standard.standard.path_number_flagged_nodes,
                    },
                    path_number_short_pauses: stat_logs[i].standard.path_number_short_pauses,
                    path_number_long_pauses: stat_logs[i].standard.path_number_long_pauses,
                },
            });

            bidir_dijkstra_query.clean();
        }
    }

    {
        let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
        let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;

        let mut core_ch_query = CSP2CoreCHQuery::new(core_ch.borrow());
        core_ch_query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

        let mut core_ch_chpot_query = CSP2AstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
        core_ch_chpot_query.set_restriction(EU_LONG_DRIVING_TIME, EU_LONG_PAUSE_TIME, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

        for (i, &(s, t)) in queries.iter().enumerate() {
            print!("\rProgress {}/{} from {} to {} - Core CH        ", i, n, s, t);
            stdout().flush()?;

            let start = Instant::now();
            core_ch_query.init_new_s(s);
            core_ch_query.init_new_t(t);
            let _core_ch_dist = core_ch_query.run_query();
            let core_ch_time = start.elapsed();

            print!("\rProgress {}/{} from {} to {} - A* Core CH\t\t\t\t\t\t\t", i, n, s, t);
            stdout().flush()?;

            let start = Instant::now();
            core_ch_chpot_query.init_new_s(s);
            core_ch_chpot_query.init_new_t(t);
            let _core_ch_chpot_dist = core_ch_chpot_query.run_query();
            let core_ch_chpot_time = start.elapsed();

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: num_nodes,
                        graph_num_edges: num_edges,
                        num_queue_pushes: stat_logs[i].standard.standard.num_queue_pushes,
                        num_settled: stat_logs[i].standard.standard.num_settled,
                        num_labels_propagated: stat_logs[i].standard.standard.num_labels_propagated,
                        num_labels_reset: stat_logs[i].standard.standard.num_labels_reset,
                        num_nodes_searched: stat_logs[i].standard.standard.num_nodes_searched,
                        time: core_ch_time,
                        path_distance: stat_logs[i].standard.standard.path_distance,
                        path_number_nodes: stat_logs[i].standard.standard.path_number_nodes,
                        path_number_flagged_nodes: stat_logs[i].standard.standard.path_number_flagged_nodes,
                    },
                    path_number_short_pauses: stat_logs[i].standard.path_number_short_pauses,
                    path_number_long_pauses: stat_logs[i].standard.path_number_long_pauses,
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch_chpot"),
                standard: CSP2MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: num_nodes,
                        graph_num_edges: num_edges,
                        num_queue_pushes: stat_logs[i].standard.standard.num_queue_pushes,
                        num_settled: stat_logs[i].standard.standard.num_settled,
                        num_labels_propagated: stat_logs[i].standard.standard.num_labels_propagated,
                        num_labels_reset: stat_logs[i].standard.standard.num_labels_reset,
                        num_nodes_searched: stat_logs[i].standard.standard.num_nodes_searched,
                        time: core_ch_chpot_time,
                        path_distance: stat_logs[i].standard.standard.path_distance,
                        path_number_nodes: stat_logs[i].standard.standard.path_number_nodes,
                        path_number_flagged_nodes: stat_logs[i].standard.standard.path_number_flagged_nodes,
                    },
                    path_number_short_pauses: stat_logs[i].standard.path_number_short_pauses,
                    path_number_long_pauses: stat_logs[i].standard.path_number_long_pauses,
                },
            });
        }
    }

    println!("\rProgress {}/{}", n, n);

    let file = File::create("thesis_avg_mid-csp-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
