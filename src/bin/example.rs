extern crate stud_rust_base;

use stud_rust_base::{
    types::*,
    io::*,
    time::report_time,
};

use std::{env, path::Path};

fn main() {
    let mut args = env::args();
    args.next();

    let arg = &args.next().expect("No directory arg given");
    let path = Path::new(arg);

    let first_out = Vec::<EdgeId>::load_from(path.join("first_out").to_str().unwrap()).expect("could not read first_out");
    let head = Vec::<NodeId>::load_from(path.join("head").to_str().unwrap()).expect("could not read head");
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time").to_str().unwrap()).expect("could not read travel_time");

    report_time("iterating over arcs of some node", || {
        let node_id = 42;
        for &edge_id in &head[first_out[node_id] as usize .. first_out[node_id + 1] as usize] {
            println!("There is an arc from {} to {} with weight {}", node_id, head[edge_id as usize], travel_time[edge_id as usize]);
        }
    });

    vec![42; 42].write_to(path.join("distances").to_str().unwrap()).expect("could not write distances");
}
