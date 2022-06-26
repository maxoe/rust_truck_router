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
        csp::{OneRestrictionDijkstra, OneRestrictionDijkstraData},
        csp_bidir::CSPBidirQuery,
        csp_bidir_chpot::CSPBidirAstarCHPotQuery,
        csp_core_ch::CSPCoreCHQuery,
        csp_core_ch_chpot::CSPAstarCoreCHQuery,
    },
    experiments::measurement::{CSP1MeasurementResult, CSPMeasurementResult, MeasurementResult},
    io::load_routingkit_bitvector,
    types::{Graph, NodeId, OwnedGraph, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME},
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);

    let graph = OwnedGraph::load_from_routingkit_dir(path)?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let bw_graph = OwnedGraph::reverse(graph.borrow());
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;

    let mut dijkstra_state = OneRestrictionDijkstraData::new(graph.num_nodes());
    dijkstra_state.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);
    let dijkstra = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    let mut astar_state = OneRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    astar_state.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);
    let astar = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    let mut bidir_dijkstra_query = CSPBidirQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node);
    bidir_dijkstra_query.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let mut bidir_astar_query = CSPBidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());
    bidir_astar_query.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let mut core_ch_query = CSPCoreCHQuery::new(core_ch.borrow());
    core_ch_query.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let mut core_ch_chpot_query = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    core_ch_chpot_query.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

    let n = 1_000;

    #[derive(Debug, Clone)]
    struct LocalMeasurementResult {
        pub algo: String,
        standard: CSP1MeasurementResult,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "algo";

        fn get_header() -> String {
            format!("{},{}", Self::OWN_HEADER, CSP1MeasurementResult::get_header())
        }
        fn as_csv(&self) -> String {
            format!("{},{}", self.algo, self.standard.as_csv())
        }
    }

    let mut stat_logs = Vec::with_capacity(n);

    for _i in 0..n {
        print!("Progress {}/{}\r", _i, n);
        stdout().flush()?;

        let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
        let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

        let start = Instant::now();
        dijkstra_state.init_new_s(s);
        let dijkstra_dist = dijkstra.dist_query(&mut dijkstra_state, t);
        let dijkstra_time = start.elapsed();

        let start = Instant::now();
        astar_state.init_new_s(s);
        let astar_dist = astar.dist_query(&mut astar_state, t);
        let astar_time = start.elapsed();

        let start = Instant::now();
        bidir_dijkstra_query.init_new_s(s);
        bidir_dijkstra_query.init_new_t(t);
        bidir_dijkstra_query.run_query();
        let bidir_dijkstra_time = start.elapsed();

        let start = Instant::now();
        bidir_astar_query.init_new_s(s);
        bidir_astar_query.init_new_t(t);
        bidir_astar_query.run_query();
        let bidir_astar_time = start.elapsed();

        let start = Instant::now();
        core_ch_query.init_new_s(s);
        core_ch_query.init_new_t(t);
        let core_ch_dist = core_ch_query.run_query();
        let core_ch_time = start.elapsed();

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
                algo: String::from("dijkstra"),
                standard: CSP1MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: dijkstra_state.num_queue_pushes,
                        num_settled: dijkstra_state.num_settled,
                        num_labels_propagated: dijkstra_state.num_labels_propagated,
                        num_labels_reset: dijkstra_state.num_labels_reset,
                        num_nodes_searched: dijkstra_state.get_number_of_visited_nodes(),
                        time: dijkstra_time,
                        path_distance: dijkstra_dist,
                        path_number_nodes: Some(path.0.len()),
                        path_number_flagged_nodes: Some(number_flagged_nodes.len()),
                    },
                    path_number_pauses: Some(number_pauses.len()),
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("astar_chpot"),
                standard: CSP1MeasurementResult {
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
                    path_number_pauses: Some(number_pauses.len()),
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("dijkstra_bidir_chpot"),
                standard: CSP1MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: dijkstra_state.num_queue_pushes,
                        num_settled: dijkstra_state.num_settled,
                        num_labels_propagated: dijkstra_state.num_labels_propagated,
                        num_labels_reset: dijkstra_state.num_labels_reset,
                        num_nodes_searched: dijkstra_state.get_number_of_visited_nodes(),
                        time: bidir_dijkstra_time,
                        path_distance: dijkstra_dist,
                        path_number_nodes: Some(path.0.len()),
                        path_number_flagged_nodes: Some(number_flagged_nodes.len()),
                    },
                    path_number_pauses: Some(number_pauses.len()),
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("astar_bidir_chpot"),
                standard: CSP1MeasurementResult {
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
                    path_number_pauses: Some(number_pauses.len()),
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch"),
                standard: CSP1MeasurementResult {
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
                    path_number_pauses: Some(number_pauses.len()),
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch_chpot"),
                standard: CSP1MeasurementResult {
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
                    path_number_pauses: Some(number_pauses.len()),
                },
            });
        } else {
            stat_logs.push(LocalMeasurementResult {
                algo: String::from("dijkstra_chpot"),
                standard: CSP1MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: dijkstra_state.num_queue_pushes,
                        num_settled: dijkstra_state.num_settled,
                        num_labels_propagated: dijkstra_state.num_labels_propagated,
                        num_labels_reset: dijkstra_state.num_labels_reset,
                        num_nodes_searched: dijkstra_state.get_number_of_visited_nodes(),
                        time: dijkstra_time,
                        path_distance: None,
                        path_number_nodes: None,
                        path_number_flagged_nodes: None,
                    },
                    path_number_pauses: None,
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("astar_chpot"),
                standard: CSP1MeasurementResult {
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
                    path_number_pauses: None,
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("dijkstra_bidir_chpot"),
                standard: CSP1MeasurementResult {
                    standard: CSPMeasurementResult {
                        graph_num_nodes: graph.num_nodes(),
                        graph_num_edges: graph.num_arcs(),
                        num_queue_pushes: dijkstra_state.num_queue_pushes,
                        num_settled: dijkstra_state.num_settled,
                        num_labels_propagated: dijkstra_state.num_labels_propagated,
                        num_labels_reset: dijkstra_state.num_labels_reset,
                        num_nodes_searched: dijkstra_state.get_number_of_visited_nodes(),
                        time: bidir_dijkstra_time,
                        path_distance: None,
                        path_number_nodes: None,
                        path_number_flagged_nodes: None,
                    },
                    path_number_pauses: None,
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("astar_bidir_chpot"),
                standard: CSP1MeasurementResult {
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
                    path_number_pauses: None,
                },
            });

            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch"),
                standard: CSP1MeasurementResult {
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
                    path_number_pauses: None,
                },
            });
            stat_logs.push(LocalMeasurementResult {
                algo: String::from("core_ch_chpot"),
                standard: CSP1MeasurementResult {
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
                    path_number_pauses: None,
                },
            });
        }
    }
    println!("Progress {}/{}", n, n);

    let file = File::create("thesis_avg_all-csp-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
