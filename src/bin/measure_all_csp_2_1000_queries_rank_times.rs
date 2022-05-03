use rand::Rng;
use rust_truck_router::{
    algo::{
        ch::*,
        ch_potential::CHPotential,
        core_ch::CoreContractionHierarchy,
        csp_2::{TwoRestrictionDijkstra, TwoRestrictionDijkstraData},
        csp_2_bidir::CSP2BidirAstarCHPotQuery,
        csp_core_ch::CSPCoreCHQuery,
        csp_core_ch_chpot::CSPAstarCoreCHQuery,
        dijkstra::{Dijkstra, DijkstraData},
    },
    experiments::measurement::{CSP2MeasurementResult, CSPMeasurementResult, MeasurementResult},
    io::*,
    types::*,
};
use std::{
    env,
    error::Error,
    fs::File,
    io::{stdout, LineWriter, Write},
    path::Path,
    time::Instant,
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    let graph = OwnedGraph::new(first_out, head, travel_time);
    let bw_graph = OwnedGraph::reverse(graph.borrow());

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    ch.check();
    core_ch.check();

    let mut astar_state = TwoRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    astar_state.set_restriction(32_400_000, 32_400_000, 16_200_000, 2_700_000);
    let astar = TwoRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    let mut bidir_astar_query = CSP2BidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());
    bidir_astar_query.set_restriction(32_400_000, 32_400_000, 16_200_000, 2_700_000);

    let mut core_ch_query = CSPCoreCHQuery::new(core_ch.borrow());
    core_ch_query.set_restriction(16_200_000, 2_700_000);

    let mut core_ch_chpot_query = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
    core_ch_chpot_query.set_restriction(16_200_000, 2_700_000);

    let log_num_nodes = (graph.num_nodes() as f32).log2() as usize;
    let mut dijkstra_state = DijkstraData::new(graph.num_nodes());
    let dijkstra = Dijkstra::new(graph.borrow());

    let n = 100;

    #[derive(Debug, Clone)]
    struct LocalMeasurementResult {
        pub dijkstra_rank_exponent: usize,
        pub algo: String,
        standard: CSP2MeasurementResult,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "dijkstra_rank_exponent,algo";

        fn get_header() -> String {
            format!("{},{}", Self::OWN_HEADER, CSP2MeasurementResult::get_header())
        }
        fn as_csv(&self) -> String {
            format!("{},{},{}", self.dijkstra_rank_exponent, self.algo, self.standard.as_csv())
        }
    }

    let mut stat_logs = Vec::with_capacity(n * log_num_nodes);

    for _i in 0..n {
        print!("Progress {}/{}\r", _i, n);
        stdout().flush()?;

        let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

        astar_state.init_new_s(s);
        dijkstra_state.init_new_s(s);

        let rank_order = dijkstra.ranks_only_exponentials(&mut dijkstra_state);

        for (i, current_t) in rank_order.into_iter().enumerate() {
            let start = Instant::now();
            astar_state.init_new_s(s);
            let astar_dist = astar.dist_query(&mut astar_state, current_t);
            let astar_time = start.elapsed();

            let start = Instant::now();
            bidir_astar_query.init_new_s(s);
            bidir_astar_query.init_new_t(current_t);
            bidir_astar_query.run_query();
            let bidir_astar_time = start.elapsed();

            let start = Instant::now();
            core_ch_query.init_new_s(s);
            core_ch_query.init_new_t(current_t);
            let core_ch_dist = core_ch_query.run_query();
            let core_ch_time = start.elapsed();

            let start = Instant::now();
            core_ch_chpot_query.init_new_s(s);
            core_ch_chpot_query.init_new_t(current_t);
            let core_ch_chpot_dist = core_ch_chpot_query.run_query();
            let core_ch_chpot_time = start.elapsed();

            // assert_eq!(astar_dist, core_ch_dist);
            // assert_eq!(core_ch_dist, core_ch_chpot_dist);

            if astar_dist.is_some() {
                let path = astar_state.current_best_path_to(current_t, true).unwrap();
                let number_flagged_nodes = astar.flagged_nodes_on_path(&path);
                let number_pauses = astar.reset_nodes_on_path(&path);

                stat_logs.push(LocalMeasurementResult {
                    dijkstra_rank_exponent: i,
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
                    dijkstra_rank_exponent: i,
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
                    dijkstra_rank_exponent: i,
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
                    dijkstra_rank_exponent: i,
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
                    dijkstra_rank_exponent: i,
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
                    dijkstra_rank_exponent: i,
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
                    dijkstra_rank_exponent: i,
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
                    dijkstra_rank_exponent: i,
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
    }
    println!("Progress {}/{}", n, n);

    let file = File::create("measure_all_csp_2_1000_queries_rank_times-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
