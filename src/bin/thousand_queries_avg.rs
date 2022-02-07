use rand::Rng;
use std::{env, error::Error, path::Path};
use stud_rust_base::{
    algo::{
        ch::*,
        ch_potential::CHPotential,
        mcd::{OneRestrictionDijkstra, OwnedOneRestrictionGraph},
    },
    io::*,
    time::report_time,
    types::*,
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);

    let first_out = Vec::<stud_rust_base::types::EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<stud_rust_base::types::NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<stud_rust_base::types::Weight>::load_from(path.join("travel_time"))?;

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();
    let pot = CHPotential::from_ch(ch);

    // let graph = stud_rust_base::types::OwnedGraph::new(first_out, head, travel_time);
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let graph_mcd = OwnedOneRestrictionGraph::new(first_out, head, travel_time);
    let mut instance_mcd = OneRestrictionDijkstra::new_with_potential(graph_mcd.borrow(), pot);
    instance_mcd.set_reset_flags(is_parking_node).set_restriction(16_200_000, 1_950_000);
    // println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    // report_time("1000 random ch one-to-one distance queries", || {
    //     for _i in 0..1000 {
    //         let s = rand::thread_rng().gen_range(0..graph.num_nodes() as stud_rust_base::types::NodeId);
    //         let t = rand::thread_rng().gen_range(0..graph.num_nodes() as stud_rust_base::types::NodeId);
    //         ch.init_new_s(s);
    //         ch.init_new_t(t);
    //         ch.run_query();
    //     }
    // });

    report_time("random 1000 restriction dijkstra one-to-one distance queries", || {
        for _i in 0..1000 {
            if _i % 100 == 0 {
                println!("{}", _i);
            }
            let s = rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as NodeId);
            let t = rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as NodeId);

            instance_mcd.init_new_s(s);
            instance_mcd.reset();
            instance_mcd.dist_query(t);
        }
    });

    report_time(
        "random 1000 restriction dijkstra one-to-one distance queries, always propagate all labels",
        || {
            for _i in 0..1000 {
                if _i % 100 == 0 {
                    println!("{}", _i);
                }
                let s = rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as NodeId);
                let t = rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as NodeId);

                instance_mcd.init_new_s(s);
                instance_mcd.reset();
                instance_mcd.dist_query_propagate_all_labels(t);
            }
        },
    );

    Ok(())
}
