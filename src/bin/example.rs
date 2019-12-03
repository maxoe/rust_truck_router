use stud_rust_base::{io::*, time::report_time, types::*};

use std::{env, error::Error, path::Path};

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args();
    args.next();

    let arg = &args.next().expect("No directory arg given");
    let path = Path::new(arg);

    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;

    report_time("iterating over arcs of some node", || {
        let node_id = 42;
        for edge_id in first_out[node_id]..first_out[node_id + 1] {
            println!(
                "There is an arc from {} to {} with weight {}",
                node_id, head[edge_id as usize], travel_time[edge_id as usize]
            );
        }
    });

    vec![42; 42].write_to(&path.join("distances"))?;

    Ok(())
}
