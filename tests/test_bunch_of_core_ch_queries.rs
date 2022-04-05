use rust_truck_router::{
    algo::{core_ch::CoreContractionHierarchy, mcd::OneRestrictionDijkstra},
    types::OwnedGraph,
};
use std::{error::Error, path::Path};

#[test]
fn load_core_ch() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    ch.check();
    Ok(())
}

#[test]
fn query_does_not_reach_core() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let graph = OwnedGraph::load_from_routingkit_dir(path.clone())?;
    let mut core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();

    let to_test = [(0, 1)];

    for (s, t) in to_test {
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();

        let mut csp_pot = OneRestrictionDijkstra::new(&graph);
        csp_pot.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}

#[test]
fn query_to_core_node() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let graph = OwnedGraph::load_from_routingkit_dir(path.clone())?;
    let mut core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();

    let to_test = [(0, 2), (0, 3)];

    for (s, t) in to_test {
        println!("Testing {} -> {}", s, t);
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();

        let mut csp_pot = OneRestrictionDijkstra::new(&graph);
        csp_pot.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}

#[test]
fn query_through_core() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let graph = OwnedGraph::load_from_routingkit_dir(path.clone())?;
    let mut core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();

    let to_test = [(0, 4), (1, 4)];

    for (s, t) in to_test {
        println!("Testing {} -> {}", s, t);
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();

        let mut csp_pot = OneRestrictionDijkstra::new(&graph);
        csp_pot.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}

#[test]
fn query_from_core_node() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let graph = OwnedGraph::load_from_routingkit_dir(path.clone())?;
    let mut core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();

    let to_test = [(2, 4), (3, 4)];

    for (s, t) in to_test {
        println!("Testing {} -> {}", s, t);
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();

        let mut csp_pot = OneRestrictionDijkstra::new(&graph);
        csp_pot.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}
