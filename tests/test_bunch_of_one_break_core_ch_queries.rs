use std::{error::Error, path::Path};

use rust_truck_router::algo::one_break_core_ch::OneBreakCoreContractionHierarchy;

#[test]
fn query_fails_due_to_restriction() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let mut core_ch = OneBreakCoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.add_restriction(4, 0);
    core_ch.check();

    let to_test = [(0, 4, None)];

    for (s, t, result) in to_test {
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();
        assert_eq!(dist, result);
    }

    Ok(())
}

#[test]
fn query_succeeds() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let mut core_ch = OneBreakCoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();

    let to_test = [
        (0, 4, 5, 0, Some(8)),
        (0, 4, 5, 1, Some(9)),
        (0, 4, 6, 0, Some(7)),
        (0, 4, 6, 1, Some(8)),
        (0, 4, 6, 10, Some(17)),
        (0, 4, 8, 10, Some(7)),
    ];

    for (s, t, max_driving_time, pause_time, result) in to_test {
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        core_ch.clear_restrictions();
        core_ch.add_restriction(max_driving_time, pause_time);
        let dist = core_ch.run_query();
        assert_eq!(dist, result);
    }

    Ok(())
}

#[test]
fn query_multiple_restrictions() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/core_instance"));
    let mut core_ch = OneBreakCoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();

    core_ch.init_new_s(0);
    core_ch.init_new_t(4);
    core_ch.add_restriction(5, 1);
    core_ch.add_restriction(6, 3);
    core_ch.add_restriction(8, 4);
    let dist = core_ch.run_query();
    assert_eq!(dist, Some(12));

    core_ch.clear_restrictions();
    core_ch.add_restriction(5, 1);
    core_ch.add_restriction(6, 3);
    core_ch.add_restriction(8, 4);
    let dist = core_ch.run_query();
    assert_eq!(dist, Some(12));

    Ok(())
}
