use bit_vec::BitVec;
use stud_rust_base::{algo::mcd::*, types::OwnedGraph};

#[test]
fn simple() {
    //  0 - 1
    // 	| / |
    //  3 - 2
    let first_out = vec![0, 1, 2, 3, 5];
    let head = vec![1, 2, 3, 0, 1];
    let travel_time = vec![2, 3, 3, 1, 5];
    let s = 2;
    let t = 1;
    let graph_mcd = OwnedGraph::new(first_out, head, travel_time);

    let mut instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    let result = instance_mcd.dist_query(t).unwrap();

    assert_eq!(result, 6);

    instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd.set_restriction(5, 0);
    let result = instance_mcd.dist_query(t);
    assert_eq!(result, None);
}

#[test]
fn shortes_path_breaks_constraint() {
    // 0 -> 1 -> 2p -> 4
    //      | -> 3p -> |
    let first_out = vec![0, 1, 3, 4, 5, 5];
    let head = vec![1, 2, 3, 4, 4];
    let travel_time = vec![1, 4, 3, 2, 4];
    let s = 0;
    let t = 4;
    let graph_mcd = OwnedGraph::new(first_out, head, travel_time);
    let mut instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd
        .set_reset_flags(BitVec::from_fn(5, |i| i == 2 || i == 3).to_bytes())
        .set_restriction(5, 0);
    let result = instance_mcd.dist_query(t).unwrap();
    assert_eq!(result, 8);
    assert_eq!(vec![0, 1, 3, 4], instance_mcd.current_best_node_path_to(t).unwrap());

    instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd
        .set_reset_flags(BitVec::from_fn(5, |i| i == 2 || i == 3).to_bytes())
        .set_restriction(6, 0);
    let result = instance_mcd.dist_query(t).unwrap();
    assert_eq!(result, 7);
    assert_eq!(vec![0, 1, 2, 4], instance_mcd.current_best_node_path_to(t).unwrap());

    instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd
        .set_reset_flags(BitVec::from_fn(5, |i| i == 2 || i == 3).to_bytes())
        .set_restriction(5, 2);
    let result = instance_mcd.dist_query(t).unwrap();
    assert_eq!(result, 10);
    assert_eq!(vec![0, 1, 3, 4], instance_mcd.current_best_node_path_to(t).unwrap());

    instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd
        .set_reset_flags(BitVec::from_fn(5, |i| i == 2 || i == 3).to_bytes())
        .set_restriction(6, 2);
    let result = instance_mcd.dist_query(t).unwrap();
    assert_eq!(result, 9);
    assert_eq!(vec![0, 1, 2, 4], instance_mcd.current_best_node_path_to(t).unwrap());
}

#[test]
fn needs_loop_to_fulfill_constraint() {
    // 0 -> 1 <-> 2(p)
    //         -> 3
    let first_out = vec![0, 1, 3, 4, 4];
    let head = vec![1, 2, 3, 1];
    let travel_time = vec![2, 1, 3, 1];
    let s = 0;
    let t = 3;
    let graph_mcd = OwnedGraph::new(first_out, head, travel_time);
    let mut instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd.set_reset_flags(BitVec::from_fn(4, |i| i == 2).to_bytes()).set_restriction(5, 0);
    let mut result = instance_mcd.dist_query(t).unwrap();
    assert_eq!(result, 7);
    assert_eq!(vec![0, 1, 2, 1, 3], instance_mcd.current_best_node_path_to(t).unwrap());

    instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd.set_reset_flags(BitVec::from_fn(4, |i| i == 2).to_bytes()).set_restriction(7, 0);
    result = instance_mcd.dist_query(t).unwrap();
    assert_eq!(result, 5);
    assert_eq!(vec![0, 1, 3], instance_mcd.current_best_node_path_to(t).unwrap());

    instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd.set_reset_flags(BitVec::from_fn(4, |i| i == 2).to_bytes()).set_restriction(5, 2);
    let mut result = instance_mcd.dist_query(t).unwrap();
    assert_eq!(result, 9);
    assert_eq!(vec![0, 1, 2, 1, 3], instance_mcd.current_best_node_path_to(t).unwrap());

    instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd.set_reset_flags(BitVec::from_fn(4, |i| i == 2).to_bytes()).set_restriction(7, 2);
    result = instance_mcd.dist_query(t).unwrap();
    assert_eq!(result, 5);
    assert_eq!(vec![0, 1, 3], instance_mcd.current_best_node_path_to(t).unwrap());
}

#[test]
fn ignore_parking() {
    // 0 -> 1 -> 2p -> 4
    //      | -> 3p -> |
    let first_out = vec![0, 1, 3, 4, 5, 5];
    let head = vec![1, 2, 3, 4, 4];
    let travel_time = vec![1, 3, 1, 3, 1];
    let s = 0;
    let t = 4;

    let graph_mcd = OwnedGraph::new(first_out, head, travel_time);
    let mut instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd
        .set_reset_flags(BitVec::from_fn(5, |i| i == 2 || i == 3).to_bytes())
        .set_restriction(5, 4);
    let result = instance_mcd.dist_query(t).unwrap();
    assert_eq!(result, 3);
    assert_eq!(vec![0, 1, 3, 4], instance_mcd.current_best_node_path_to(t).unwrap());
}

#[test]
fn no_path_fulfills_constraint() {
    // 0 -> 1 -> 2
    let first_out = vec![0, 1, 2, 2];
    let head = vec![1, 2];
    let travel_time = vec![1, 5];
    let s = 0;
    let t = 2;
    let graph_mcd = OwnedGraph::new(first_out, head, travel_time);
    let mut instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow());
    instance_mcd.init_new_s(s);
    instance_mcd.set_reset_flags(BitVec::from_fn(3, |i| i == 1).to_bytes()).set_restriction(4, 0);

    assert_eq!(instance_mcd.dist_query(t), None);
    assert_eq!(None, instance_mcd.current_best_node_path_to(t));
}
