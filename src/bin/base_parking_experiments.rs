use stud_rust_base::{
    algo::{dijkstra::Dijkstra, mcd::*},
    io::*,
    time::measure,
    types::*,
};

use rand::{Rng, SeedableRng};

use std::{error::Error, io::Write, path::Path, vec};
use std::{fs::File, io::LineWriter};

fn main() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/large/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    let graph = OwnedGraph::new(first_out.clone(), head.clone(), travel_time.clone());
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let graph_mcd = OwnedOneRestrictionGraph::new(first_out, head, travel_time);

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);
    let n = 100;
    let mut instance = Dijkstra::new(graph.borrow());

    let mut results_without_constraint = Vec::with_capacity(n);
    for i in 0..n {
        let s = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        println!("Query #{} from {} to {} without constraints", i, s, t);
        instance.init_new_s(s);
        let mut instance_mcd_with_reset = OneRestrictionDijkstra::new(graph_mcd.borrow(), s);
        instance_mcd_with_reset.set_reset_flags(is_parking_node.to_bytes());
        let mut instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow(), s);

        results_without_constraint.push((
            measure(|| instance.dist_query(t)).1.num_microseconds().unwrap(),
            measure(|| instance_mcd.dist_query(t)).1.num_microseconds().unwrap(),
            measure(|| instance_mcd_with_reset.dist_query(t)).1.num_microseconds().unwrap(),
            instance.num_settled,
        ));
    }

    let file = File::create("../../eval/data/dijkstra_csp_comp.csv")?;
    let mut file = LineWriter::new(file);

    writeln!(file, "dijkstra_time_µs,csp_time_µs,csp_with_reset_time_µs,dijkstra_rank")?;

    for i in 0..results_without_constraint.len() {
        writeln!(
            file,
            "{},{},{},{}",
            results_without_constraint[i].0, results_without_constraint[i].1, results_without_constraint[i].2, results_without_constraint[i].3
        )?;
    }

    gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);
    let mut results_number_parking = vec![Vec::with_capacity(n); 3];
    let mut dijkstra = Dijkstra::new(graph.borrow());

    let mut i = 0;
    while results_number_parking[0].len() != n || results_number_parking[1].len() != n || results_number_parking[2].len() != n {
        let s = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        println!(
            "Query #{} from {} to {} with variable number parking, lengths [0, 1, 2] are [{}, {}, {}]",
            i,
            s,
            t,
            results_number_parking[0].len(),
            results_number_parking[1].len(),
            results_number_parking[2].len()
        );
        let mut instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow(), s);
        instance_mcd.set_reset_flags(is_parking_node.to_bytes()).set_restriction(1_620_000, 360_000);

        let (_, time) = measure(|| instance_mcd.dist_query(t));

        if let Some(parkings) = &instance_mcd.current_best_path_to(t, true) {
            let num_parkings = instance_mcd.reset_nodes_on_path(parkings).len();

            if results_number_parking[num_parkings].len() < n {
                instance.init_new_s(s);
                dijkstra.init_new_s(s);
                dijkstra.dist_query(t);
                results_number_parking[num_parkings].push((time.num_microseconds().unwrap(), dijkstra.num_settled));
            }
        }

        i += 1;
    }

    for i in 0..results_number_parking.len() {
        println!(
            "#{} parking avg: {}",
            i,
            results_number_parking[i].iter().map(|e| e.0).sum::<i64>() / results_number_parking[i].len() as i64
        );
    }

    let file = File::create("../../eval/data/variable_number_parking.csv")?;
    let mut file = LineWriter::new(file);

    writeln!(file, "number_of_parkings,time_µs,dijkstra_rank")?;

    for j in 0..n {
        for i in 0..results_number_parking.len() {
            writeln!(file, "{},{},{}", i, results_number_parking[i][j].0, results_number_parking[i][j].1)?;
        }
    }

    Ok(())
}
