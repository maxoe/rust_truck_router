use super::{
    astar::Potential,
    ch::BorrowedContractionHierarchy,
    csp_2::{TwoRestrictionDijkstra, TwoRestrictionDijkstraData},
};
use crate::{algo::ch_potential::CHPotential, types::*};
use bit_vec::BitVec;
use num::Integer;
use std::{
    fmt::Write,
    time::{Duration, Instant},
};

pub struct CSP2BidirAstarCHPotQuery<'a> {
    fw_graph: BorrowedGraph<'a>,
    bw_graph: BorrowedGraph<'a>,
    is_reset_node: &'a BitVec,
    fw_state: TwoRestrictionDijkstraData<CHPotential<'a>>,
    bw_state: TwoRestrictionDijkstraData<CHPotential<'a>>,
    restriction_long: DrivingTimeRestriction,
    restriction_short: DrivingTimeRestriction,
    fw_finished: bool,
    bw_finished: bool,
    s: NodeId,
    t: NodeId,
    pub last_dist: Option<Weight>,
    last_middle_node: Option<NodeId>,
    last_time_elapsed: Duration,
}

impl<'a> CSP2BidirAstarCHPotQuery<'a> {
    pub fn new(fw_graph: BorrowedGraph<'a>, bw_graph: BorrowedGraph<'a>, is_reset_node: &'a BitVec, ch: BorrowedContractionHierarchy<'a>) -> Self {
        let node_count = fw_graph.num_nodes();
        CSP2BidirAstarCHPotQuery {
            fw_graph,
            bw_graph,
            is_reset_node,
            fw_state: TwoRestrictionDijkstraData::new_with_potential(node_count, CHPotential::from_ch(ch)),
            bw_state: TwoRestrictionDijkstraData::new_with_potential(node_count, CHPotential::from_ch_backwards(ch)),
            restriction_long: DrivingTimeRestriction {
                pause_time: 0,
                max_driving_time: Weight::infinity(),
            },
            restriction_short: DrivingTimeRestriction {
                pause_time: 0,
                max_driving_time: Weight::infinity(),
            },
            fw_finished: false,
            bw_finished: false,
            s: node_count as NodeId,
            t: node_count as NodeId,
            last_dist: None,
            last_middle_node: None,
            last_time_elapsed: Duration::ZERO,
        }
    }

    pub fn init_new_s(&mut self, s: NodeId) {
        self.s = s;
    }

    pub fn init_new_t(&mut self, t: NodeId) {
        self.t = t;
    }

    pub fn set_restriction(&mut self, max_driving_time_long: Weight, pause_time_long: Weight, max_driving_time_short: Weight, pause_time_short: Weight) {
        self.restriction_short = DrivingTimeRestriction {
            pause_time: pause_time_short,
            max_driving_time: max_driving_time_short,
        };
        self.restriction_long = DrivingTimeRestriction {
            pause_time: pause_time_long,
            max_driving_time: max_driving_time_long,
        };

        self.fw_state
            .set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);
        self.bw_state
            .set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);
    }

    pub fn clear_restriction(&mut self) {
        self.restriction_short = DrivingTimeRestriction {
            pause_time: 0,
            max_driving_time: Weight::infinity(),
        };
        self.restriction_long = DrivingTimeRestriction {
            pause_time: 0,
            max_driving_time: Weight::infinity(),
        };

        self.fw_state.clear_restriction();
        self.bw_state.clear_restriction();
    }
    pub fn reset(&mut self) {
        if self.s != self.fw_graph.num_nodes() as NodeId {
            self.fw_state.init_new_s(self.s);
            self.bw_state.potential.init_new_t(self.s);
        }

        if self.t != self.fw_graph.num_nodes() as NodeId {
            self.bw_state.init_new_s(self.t);
            self.fw_state.potential.init_new_t(self.t);
        }

        self.fw_finished = false;
        self.bw_finished = false;
        self.last_time_elapsed = Duration::ZERO;
        self.last_dist = None;
        self.last_middle_node = None;
    }

    pub fn clean(&mut self) {
        self.fw_state.clean();
        self.bw_state.clean();
        self.reset();
    }

    fn calculate_distance_with_break_at(
        node: NodeId,
        restriction_short: &DrivingTimeRestriction,
        restriction_long: &DrivingTimeRestriction,
        fw_label_dist: &Weight3,
        bw_state: &mut TwoRestrictionDijkstraData<CHPotential>,
    ) -> Weight {
        let v_to_t = bw_state.get_settled_labels_at(node);

        let mut current_bw = v_to_t.rev().map(|r| r.0);

        let mut best_distance = Weight::infinity();
        while let Some(bw_label) = current_bw.next() {
            let total_dist = fw_label_dist.add(bw_label.distance);

            // check if restrictions allows combination of those labels/subpaths
            if total_dist[1] < restriction_short.max_driving_time && total_dist[2] < restriction_long.max_driving_time {
                // subpaths can be connected without additional break
                best_distance = best_distance.min(total_dist[0]);
            }
        }

        best_distance
    }

    pub fn run_query(&mut self) -> Option<Weight> {
        let start = Instant::now();
        if self.s == self.fw_graph.num_nodes() as NodeId || self.t == self.fw_graph.num_nodes() as NodeId {
            return None;
        }

        self.reset();

        let mut tentative_distance = Weight::infinity();

        let mut settled_fw = BitVec::from_elem(self.fw_graph.num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.fw_graph.num_nodes(), false);
        let mut fw_next = true;

        let fw_search = TwoRestrictionDijkstra::new(self.fw_graph, &self.is_reset_node);
        let bw_search = TwoRestrictionDijkstra::new(self.bw_graph, &self.is_reset_node);

        while !self.fw_finished || !self.bw_finished {
            if !self.fw_finished && (self.bw_finished || fw_next) {
                if let Some(State {
                    distance: _dist_from_queue_at_v,
                    node,
                }) = if tentative_distance < Weight::infinity() && self.bw_state.min_key().is_some() {
                    fw_search.settle_next_label_prune_bw_lower_bound(&mut self.fw_state, &mut self.bw_state, tentative_distance, self.t)
                } else {
                    fw_search.settle_next_label(&mut self.fw_state, self.t)
                } {
                    settled_fw.set(node as usize, true);

                    // fw search found t -> done here
                    if node == self.t {
                        tentative_distance = self.fw_state.get_settled_labels_at(node).last().unwrap().0.distance[0]; // dist_from_queue_at_v[0];
                        self.fw_finished = true;
                        self.bw_finished = true;
                        self.last_middle_node = None;

                        break;
                    }

                    if settled_bw.get(node as usize).unwrap() {
                        let tent_dist_at_v = Self::calculate_distance_with_break_at(
                            node,
                            &self.restriction_short,
                            &self.restriction_long,
                            &self.fw_state.get_best_label_at(node).unwrap().distance,
                            &mut self.bw_state,
                        );

                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            self.last_middle_node = Some(node);
                        }
                    }

                    if self.fw_state.min_key().is_none() {
                        self.fw_finished = true;
                        self.bw_finished = true;
                    } else if self.fw_state.min_key().unwrap() >= tentative_distance {
                        self.fw_finished = true;
                    }

                    fw_next = false;
                }
            } else if let Some(State {
                distance: _dist_from_queue_at_v,
                node,
            }) = if tentative_distance < Weight::infinity() && self.fw_state.min_key().is_some() {
                bw_search.settle_next_label_prune_bw_lower_bound(&mut self.bw_state, &mut self.fw_state, tentative_distance, self.s)
            } else {
                bw_search.settle_next_label(&mut self.bw_state, self.s)
            } {
                settled_bw.set(node as usize, true);

                // bw search found s -> done here
                if node == self.s {
                    tentative_distance = self.bw_state.get_settled_labels_at(node).last().unwrap().0.distance[0]; // dist_from_queue_at_v[0];

                    self.fw_finished = true;
                    self.bw_finished = true;
                    self.last_middle_node = None;

                    break;
                }

                if settled_fw.get(node as usize).unwrap() {
                    let tent_dist_at_v = Self::calculate_distance_with_break_at(
                        node,
                        &self.restriction_short,
                        &self.restriction_long,
                        &self.bw_state.get_best_label_at(node).unwrap().distance,
                        &mut self.fw_state,
                    );

                    if tentative_distance > tent_dist_at_v {
                        tentative_distance = tent_dist_at_v;
                        self.last_middle_node = Some(node);
                    }
                }

                if self.bw_state.min_key().is_none() {
                    self.fw_finished = true;
                    self.bw_finished = true;
                } else if self.bw_state.min_key().unwrap() >= tentative_distance {
                    self.bw_finished = true;
                }

                fw_next = true;
            }
        }

        self.last_time_elapsed = start.elapsed();

        if tentative_distance == Weight::infinity() {
            self.last_dist = None;
            return None;
        }

        self.last_dist = Some(tentative_distance);
        self.last_dist
    }

    pub fn timeout_run_query(&mut self, timeout: Duration) -> Result<Option<Weight>, QueryTimeoutError> {
        let start = Instant::now();
        if self.s == self.fw_graph.num_nodes() as NodeId || self.t == self.fw_graph.num_nodes() as NodeId {
            return Ok(None);
        }

        self.reset();

        let mut tentative_distance = Weight::infinity();

        let mut settled_fw = BitVec::from_elem(self.fw_graph.num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.fw_graph.num_nodes(), false);
        let mut fw_next = true;

        let fw_search = TwoRestrictionDijkstra::new(self.fw_graph, &self.is_reset_node);
        let bw_search = TwoRestrictionDijkstra::new(self.bw_graph, &self.is_reset_node);

        while !self.fw_finished || !self.bw_finished {
            if start.elapsed() > timeout {
                return Err(QueryTimeoutError);
            }

            if !self.fw_finished && (self.bw_finished || fw_next) {
                if let Some(State {
                    distance: _dist_from_queue_at_v,
                    node,
                }) = if tentative_distance < Weight::infinity() && self.bw_state.min_key().is_some() {
                    fw_search.settle_next_label_prune_bw_lower_bound(&mut self.fw_state, &mut self.bw_state, tentative_distance, self.t)
                } else {
                    fw_search.settle_next_label(&mut self.fw_state, self.t)
                } {
                    settled_fw.set(node as usize, true);

                    // fw search found t -> done here
                    if node == self.t {
                        tentative_distance = self.fw_state.get_settled_labels_at(node).last().unwrap().0.distance[0]; // dist_from_queue_at_v[0];
                        self.fw_finished = true;
                        self.bw_finished = true;
                        self.last_middle_node = None;

                        break;
                    }

                    if settled_bw.get(node as usize).unwrap() {
                        let tent_dist_at_v = Self::calculate_distance_with_break_at(
                            node,
                            &self.restriction_short,
                            &self.restriction_long,
                            &self.fw_state.get_best_label_at(node).unwrap().distance,
                            &mut self.bw_state,
                        );

                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            self.last_middle_node = Some(node);
                        }
                    }

                    if self.fw_state.min_key().is_none() {
                        self.fw_finished = true;
                        self.bw_finished = true;
                    } else if self.fw_state.min_key().unwrap() >= tentative_distance {
                        self.fw_finished = true;
                    }

                    fw_next = false;
                }
            } else if let Some(State {
                distance: _dist_from_queue_at_v,
                node,
            }) = if tentative_distance < Weight::infinity() && self.fw_state.min_key().is_some() {
                bw_search.settle_next_label_prune_bw_lower_bound(&mut self.bw_state, &mut self.fw_state, tentative_distance, self.s)
            } else {
                bw_search.settle_next_label(&mut self.bw_state, self.s)
            } {
                settled_bw.set(node as usize, true);

                // bw search found s -> done here
                if node == self.s {
                    tentative_distance = self.bw_state.get_settled_labels_at(node).last().unwrap().0.distance[0]; // dist_from_queue_at_v[0];

                    self.fw_finished = true;
                    self.bw_finished = true;
                    self.last_middle_node = None;

                    break;
                }

                if settled_fw.get(node as usize).unwrap() {
                    let tent_dist_at_v = Self::calculate_distance_with_break_at(
                        node,
                        &self.restriction_short,
                        &self.restriction_long,
                        &self.bw_state.get_best_label_at(node).unwrap().distance,
                        &mut self.fw_state,
                    );

                    if tentative_distance > tent_dist_at_v {
                        tentative_distance = tent_dist_at_v;
                        self.last_middle_node = Some(node);
                    }
                }

                if self.bw_state.min_key().is_none() {
                    self.fw_finished = true;
                    self.bw_finished = true;
                } else if self.bw_state.min_key().unwrap() >= tentative_distance {
                    self.bw_finished = true;
                }

                fw_next = true;
            }
        }

        self.last_time_elapsed = start.elapsed();

        if tentative_distance == Weight::infinity() {
            self.last_dist = None;
            return Ok(None);
        }

        self.last_dist = Some(tentative_distance);
        Ok(self.last_dist)
    }

    pub fn summary(&self) -> String {
        let mut s = "\n\nSummary for Bidirectional CSP\n\n".to_string();

        if self.s == self.fw_graph.num_nodes() as NodeId
            || self.t == self.bw_graph.num_nodes() as NodeId
            || self.fw_state.num_settled == 0
            || self.bw_state.num_settled == 0
        {
            writeln!(s, "No query").unwrap();
            return s;
        }

        writeln!(s, "Graph: ").unwrap();
        writeln!(s, "\tnumber of nodes: {}", self.fw_graph.num_nodes()).unwrap();
        writeln!(s, "\tnumber of arcs: {}", self.fw_graph.num_arcs()).unwrap();

        writeln!(s).unwrap();
        writeln!(s, "Restriction: ").unwrap();

        if self.restriction_short.max_driving_time < Weight::infinity() && self.restriction_long.max_driving_time < Weight::infinity() {
            let num_reset_nodes = self.is_reset_node.iter().filter(|b| *b).count();
            writeln!(
                s,
                "\tShort break with\n\t\tmax. driving time: {}\n\t\tpause time: {}\n\t\tnumber of flagged reset nodes: {} ({:.2}%)",
                self.restriction_short.max_driving_time,
                self.restriction_short.pause_time,
                num_reset_nodes,
                100.0 * num_reset_nodes as f32 / self.fw_graph.num_nodes() as f32
            )
            .unwrap();
            writeln!(
                s,
                "\tLong break with\n\t\tmax. driving time: {}\n\t\tpause time: {}\n\t\tnumber of flagged reset nodes: {} ({:.2}%)",
                self.restriction_long.max_driving_time,
                self.restriction_long.pause_time,
                num_reset_nodes,
                100.0 * num_reset_nodes as f32 / self.fw_graph.num_nodes() as f32
            )
            .unwrap();
        } else {
            writeln!(s, "no driving time restriction").unwrap();
        }

        writeln!(s).unwrap();
        writeln!(s, "Query:").unwrap();
        writeln!(s, "\ts: {} t: {}", self.s, self.t).unwrap();
        writeln!(s, "\ttime elapsed: {:.3} ms", self.last_time_elapsed.as_secs_f64() * 1000.0,).unwrap();
        writeln!(
            s,
            "\tnumber of queue pushes (fw/bw): {}/{}",
            self.fw_state.num_queue_pushes, self.bw_state.num_queue_pushes
        )
        .unwrap();
        writeln!(
            s,
            "\tnumber of settled nodes (fw/bw): {}/{}",
            self.fw_state.num_settled, self.bw_state.num_settled
        )
        .unwrap();
        writeln!(
            s,
            "\tnumber of propagated labels (fw/bw): {}/{}",
            self.fw_state.num_labels_propagated, self.bw_state.num_labels_propagated
        )
        .unwrap();
        writeln!(
            s,
            "\tnumber of labels which were reset (fw/bw): {}/{}",
            self.fw_state.num_labels_reset, self.bw_state.num_labels_reset
        )
        .unwrap();

        writeln!(s).unwrap();
        writeln!(s, "Path:").unwrap();

        if self.last_dist.is_some() {
            writeln!(s, "\tdistance: {}", self.last_dist.unwrap()).unwrap();
            writeln!(s, "\tmiddle node: {:?}", self.last_middle_node).unwrap();

            let mut path = if let Some(m) = self.last_middle_node {
                let dist_s_m = self.fw_state.get_tentative_dist_at(m)[0];
                let dist_m_t = self.bw_state.get_tentative_dist_at(m)[0];
                writeln!(s, "\t  - distance s -> m: {}", dist_s_m).unwrap();
                writeln!(s, "\t  - distance m -> t: {}", dist_m_t).unwrap();
                self.fw_state.current_best_path_to(m, true).unwrap()
            } else {
                self.fw_state.current_best_path_to(self.t, true).unwrap_or((vec![], vec![]))
            };

            let mut bw_path = if let Some(m) = self.last_middle_node {
                self.bw_state.current_best_path_to(m, true).unwrap()
            } else {
                self.bw_state.current_best_path_to(self.s, true).unwrap_or((vec![], vec![]))
            };
            bw_path.0.reverse();
            bw_path.1.reverse();
            let bw_length = bw_path.0.len();
            path.0.append(&mut bw_path.0);
            path.1.append(&mut bw_path.1);

            let (node_path, weights) = path;

            writeln!(
                s,
                "\tnumber of nodes (total/fw/bw): {}/{}/{}",
                node_path.len(),
                node_path.len() - bw_length,
                bw_length
            )
            .unwrap();

            let fw_search = TwoRestrictionDijkstra::new(self.fw_graph, &self.is_reset_node);
            let flagged_p = fw_search.flagged_nodes_on_node_path(&node_path);
            writeln!(s, "\t  -thereof number of flagged nodes: {}", flagged_p.len()).unwrap();

            let (reset_p_short, reset_p_long) = fw_search.reset_nodes_on_path(&(node_path, weights));
            writeln!(s, "\t  -thereof number of flagged nodes actually used: {}", reset_p_short.len()).unwrap();
            writeln!(s, "\t  -thereof long breaks: {}", reset_p_long.len()).unwrap();
        } else {
            writeln!(s, "\tno path found").unwrap();
        }

        let mut label_sizes_settled = Vec::with_capacity(self.fw_state.per_node_labels.len());
        let mut label_sizes_unsettled = Vec::with_capacity(self.fw_state.per_node_labels.len());
        let mut label_sizes = Vec::with_capacity(self.fw_state.per_node_labels.len());

        for i in 0..self.fw_state.per_node_labels.len() {
            let label = self.fw_state.per_node_labels.get(i);
            let number_of_unsettled = label.iter().count();
            let number_of_settled = label.popped().count();

            if number_of_settled != 0 {
                label_sizes.push(number_of_unsettled + number_of_settled);
                label_sizes_settled.push(number_of_settled);
                label_sizes_unsettled.push(number_of_unsettled);
            }
        }

        writeln!(s).unwrap();
        writeln!(s, "Label Statistics Forward:").unwrap();

        if label_sizes.is_empty() {
            writeln!(s, "\t  -no labels were propagated").unwrap();
        } else {
            writeln!(
                s,
                "\tnumber of nodes with settled labels: {} ({:.2}%)",
                label_sizes.len(),
                100.0 * label_sizes.len() as f32 / self.fw_graph.num_nodes() as f32
            )
            .unwrap();
            writeln!(s, "\t  -thereof maximum number of labels: {}", label_sizes.iter().max().unwrap()).unwrap();
            writeln!(
                s,
                "\t  -thereof avg number of labels: {:.2}",
                label_sizes.iter().sum::<usize>() as f32 / label_sizes.len() as f32
            )
            .unwrap();
            label_sizes.sort_unstable();
            let mean = if label_sizes.len().is_odd() {
                label_sizes[label_sizes.len() / 2] as f32
            } else {
                (label_sizes[(label_sizes.len() / 2) - 1] as f32 + label_sizes[label_sizes.len() / 2] as f32) / 2.0
            };
            writeln!(s, "\t  -thereof mean number of labels: {:.2}", mean).unwrap();
        }

        let mut label_sizes_settled = Vec::with_capacity(self.bw_state.per_node_labels.len());
        let mut label_sizes_unsettled = Vec::with_capacity(self.bw_state.per_node_labels.len());
        let mut label_sizes = Vec::with_capacity(self.bw_state.per_node_labels.len());

        for i in 0..self.bw_state.per_node_labels.len() {
            let label = self.bw_state.per_node_labels.get(i);
            let number_of_unsettled = label.iter().count();
            let number_of_settled = label.popped().count();

            if number_of_settled != 0 {
                label_sizes.push(number_of_unsettled + number_of_settled);
                label_sizes_settled.push(number_of_settled);
                label_sizes_unsettled.push(number_of_unsettled);
            }
        }

        writeln!(s).unwrap();
        writeln!(s, "Label Statistics Backward:").unwrap();

        if label_sizes.is_empty() {
            writeln!(s, "\t  -no labels were propagated").unwrap();
        } else {
            writeln!(
                s,
                "\tnumber of nodes with settled labels: {} ({:.2}%)",
                label_sizes.len(),
                100.0 * label_sizes.len() as f32 / self.bw_graph.num_nodes() as f32
            )
            .unwrap();
            writeln!(s, "\t  -thereof maximum number of labels: {}", label_sizes.iter().max().unwrap()).unwrap();
            writeln!(
                s,
                "\t  -thereof avg number of labels: {:.2}",
                label_sizes.iter().sum::<usize>() as f32 / label_sizes.len() as f32
            )
            .unwrap();
            label_sizes.sort_unstable();
            let mean = if label_sizes.len().is_odd() {
                label_sizes[label_sizes.len() / 2] as f32
            } else {
                (label_sizes[(label_sizes.len() / 2) - 1] as f32 + label_sizes[label_sizes.len() / 2] as f32) / 2.0
            };
            writeln!(s, "\t  -thereof mean number of labels: {:.2}", mean).unwrap();
        }
        writeln!(s).unwrap();

        s
    }
}
