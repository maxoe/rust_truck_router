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
        csp_core_ch::CSPCoreCHQuery,
        csp_core_ch_chpot::CSPAstarCoreCHQuery,
        dijkstra::{Dijkstra, DijkstraData},
    },
    experiments::measurement::{CSP1MeasurementResult, CSPMeasurementResult, MeasurementResult, EXPERIMENTS_N},
    io::load_routingkit_bitvector,
    types::{Graph, NodeId, OwnedGraph, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME},
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);

    let n = EXPERIMENTS_N;

    #[derive(Debug, Clone)]
    struct LocalMeasurementResult {
        pub algo: String,
        pub dijkstra_rank_exponent: usize,
        standard: CSP1MeasurementResult,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "algo,dijkstra_rank_exponent";

        fn get_header() -> String {
            format!("{},{}", Self::OWN_HEADER, CSP1MeasurementResult::get_header())
        }
        fn as_csv(&self) -> String {
            format!("{},{},{}", self.algo, self.dijkstra_rank_exponent, self.standard.as_csv())
        }
    }

    let mut s_nodes = Vec::with_capacity(n);
    let mut t_nodes = Vec::new();

    let mut has_path = Vec::with_capacity(n);
    let mut stat_logs: Vec<LocalMeasurementResult> = Vec::with_capacity(n);
    let num_nodes;
    let num_edges;
    let num_rank_targets;

    {
        let graph = OwnedGraph::load_from_routingkit_dir(path)?;
        num_nodes = graph.num_nodes();
        num_edges = graph.num_arcs();
        num_rank_targets = (num_nodes as f32).log2() as usize;
        t_nodes.reserve(n * num_rank_targets);

        for i in 0..n {
            print!("\rProgress {}/{} - Rank Targets                     ", i, n);
            stdout().flush()?;
            let s = rand::thread_rng().gen_range(0..num_nodes as NodeId);

            let graph = OwnedGraph::load_from_routingkit_dir(path)?;
            let mut dijkstra_state = DijkstraData::new(graph.num_nodes());
            let dijkstra = Dijkstra::new(graph.borrow());
            dijkstra_state.init_new_s(s);
            let mut ranks_targets = dijkstra.ranks_only_exponentials(&mut dijkstra_state);

            if ranks_targets.len() != num_rank_targets {
                continue;
            }

            s_nodes.push(s);
            t_nodes.append(&mut ranks_targets);
        }
    }

    {
        let graph = OwnedGraph::load_from_routingkit_dir(path)?;
        let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
        let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

        let mut astar_state = OneRestrictionDijkstraData::new_with_potential(num_nodes, CHPotential::from_ch(ch.borrow()));
        astar_state.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);
        let astar = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

        let mut t_node_it = t_nodes.iter();
        for &s in s_nodes.iter() {
            for i in 0..num_rank_targets {
                let t = *t_node_it.next().unwrap();
                print!("\rProgress {}/{} from {} to {} - A*             ", i, num_rank_targets * n, s, t);
                stdout().flush()?;

                let start = Instant::now();
                astar_state.init_new_s(s);
                let astar_dist = astar.dist_query(&mut astar_state, t);
                let astar_time = start.elapsed();

                if astar_dist.is_some() {
                    has_path.push(true);
                    let path = astar_state.current_best_path_to(t, true).unwrap();
                    let number_flagged_nodes = astar.flagged_nodes_on_path(&path);
                    let number_pauses = astar.reset_nodes_on_path(&path);

                    stat_logs.push(LocalMeasurementResult {
                        algo: String::from("astar_chpot"),
                        dijkstra_rank_exponent: i,
                        standard: CSP1MeasurementResult {
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
                            path_number_pauses: Some(number_pauses.len()),
                        },
                    });
                } else {
                    has_path.push(false);

                    stat_logs.push(LocalMeasurementResult {
                        algo: String::from("astar_chpot"),
                        dijkstra_rank_exponent: i,
                        standard: CSP1MeasurementResult {
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
                            path_number_pauses: None,
                        },
                    });
                }

                astar_state.clean();
            }
        }
    }

    {
        let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
        let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;

        let mut core_ch_query = CSPCoreCHQuery::new(core_ch.borrow());
        core_ch_query.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

        let mut core_ch_chpot_query = CSPAstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());
        core_ch_chpot_query.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);

        let mut t_node_it = t_nodes.iter();
        for (j, &s) in s_nodes.iter().enumerate() {
            for i in 0..num_rank_targets {
                let t = *t_node_it.next().unwrap();

                print!("\rProgress {}/{} from {} to {} - A* Core CH\t\t\t\t\t\t\t", i, num_rank_targets * n, s, t);
                stdout().flush()?;

                let start = Instant::now();
                core_ch_chpot_query.init_new_s(s);
                core_ch_chpot_query.init_new_t(t);
                let _core_ch_chpot_dist = core_ch_chpot_query.run_query();
                let core_ch_chpot_time = start.elapsed();

                stat_logs.push(LocalMeasurementResult {
                    algo: String::from("core_ch_chpot"),
                    dijkstra_rank_exponent: i,
                    standard: CSP1MeasurementResult {
                        standard: CSPMeasurementResult {
                            graph_num_nodes: num_nodes,
                            graph_num_edges: num_edges,
                            num_queue_pushes: stat_logs[j * num_rank_targets + i].standard.standard.num_queue_pushes,
                            num_settled: stat_logs[j * num_rank_targets + i].standard.standard.num_settled,
                            num_labels_propagated: stat_logs[j * num_rank_targets + i].standard.standard.num_labels_propagated,
                            num_labels_reset: stat_logs[j * num_rank_targets + i].standard.standard.num_labels_reset,
                            num_nodes_searched: stat_logs[j * num_rank_targets + i].standard.standard.num_nodes_searched,
                            time: core_ch_chpot_time,
                            path_distance: stat_logs[j * num_rank_targets + i].standard.standard.path_distance,
                            path_number_nodes: stat_logs[j * num_rank_targets + i].standard.standard.path_number_nodes,
                            path_number_flagged_nodes: stat_logs[j * num_rank_targets + i].standard.standard.path_number_flagged_nodes,
                        },
                        path_number_pauses: stat_logs[j * num_rank_targets + i].standard.path_number_pauses,
                    },
                });
            }
        }
    }

    println!("\rProgress {}/{}", n, n);

    let file = File::create("thesis_rank_times_all-csp-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
