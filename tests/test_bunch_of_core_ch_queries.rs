use bit_vec::BitVec;
use rust_truck_router::{
    algo::{
        core_ch::{CoreContractionHierarchy, CoreContractionHierarchyQuery},
        csp::{OneRestrictionDijkstra, OneRestrictionDijkstraData},
    },
    types::{Graph, OwnedGraph},
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
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();
    let mut core_ch = CoreContractionHierarchyQuery::new(core_ch.borrow());

    let to_test = [(0, 1)];

    for (s, t) in to_test {
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();

        let mut csp_pot_state = OneRestrictionDijkstraData::new(graph.num_nodes());
        let is_parking_node = BitVec::from_elem(graph.num_nodes(), false);
        let csp_pot = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);
        csp_pot_state.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(&mut csp_pot_state, t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}

#[test]
fn query_to_core_node() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let graph = OwnedGraph::load_from_routingkit_dir(path.clone())?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();
    let mut core_ch = CoreContractionHierarchyQuery::new(core_ch.borrow());

    let to_test = [(0, 2), (0, 3)];

    for (s, t) in to_test {
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();

        let mut csp_pot_state = OneRestrictionDijkstraData::new(graph.num_nodes());
        let is_parking_node = BitVec::from_elem(graph.num_nodes(), false);
        let csp_pot = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);
        csp_pot_state.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(&mut csp_pot_state, t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}
#[test]
fn query_through_core() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let graph = OwnedGraph::load_from_routingkit_dir(path.clone())?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();
    let mut core_ch = CoreContractionHierarchyQuery::new(core_ch.borrow());

    let to_test = [(0, 4), (1, 4)];

    for (s, t) in to_test {
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();

        let mut csp_pot_state = OneRestrictionDijkstraData::new(graph.num_nodes());
        let is_parking_node = BitVec::from_elem(graph.num_nodes(), false);
        let csp_pot = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);
        csp_pot_state.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(&mut csp_pot_state, t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}

#[test]
fn query_from_core_node() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let graph = OwnedGraph::load_from_routingkit_dir(path.clone())?;
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();
    let mut core_ch = CoreContractionHierarchyQuery::new(core_ch.borrow());

    let to_test = [(2, 4), (3, 4)];

    for (s, t) in to_test {
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();

        let mut csp_pot_state = OneRestrictionDijkstraData::new(graph.num_nodes());
        let is_parking_node = BitVec::from_elem(graph.num_nodes(), false);
        let csp_pot = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);
        csp_pot_state.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(&mut csp_pot_state, t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}
