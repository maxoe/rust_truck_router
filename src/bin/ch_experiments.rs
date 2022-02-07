use stud_rust_base::{
    algo::{ch::ContractionHierarchy, dijkstra::Dijkstra},
    io::*,
    time::measure,
    types::*,
};

use rand::{Rng, SeedableRng};

use std::{error::Error, io::Write, path::Path};
use std::{fs::File, io::LineWriter};

fn main() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/large/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;

    let graph = OwnedGraph::new(first_out, head, travel_time);
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let mut ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);
    let n = 100;

    let mut results_without_constraint = Vec::with_capacity(n);
    let mut instance = Dijkstra::new(graph.borrow());

    for _i in 0..n {
        let s = gen.gen_range(0..graph.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph.num_nodes() as NodeId);
        // println!("Query #{} from {} to {}", i, s, t);

        instance.init_new_s(s);
        ch.init_new_s(s);
        ch.init_new_t(t);
        results_without_constraint.push((
            measure(|| instance.dist_query(t)).1.num_microseconds().unwrap(),
            measure(|| ch.run_query()).1.num_microseconds().unwrap(),
            instance.num_settled,
        ));
    }

    let file = File::create("../../eval/data/dijkstra_ch_cmp.csv")?;
    let mut file = LineWriter::new(file);

    writeln!(file, "dijkstra_time_µs,ch_time_µs,dijkstra_rank")?;

    for result in results_without_constraint {
        writeln!(file, "{},{},{}", result.0, result.1, result.2)?;
    }

    Ok(())
}
