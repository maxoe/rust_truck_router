use rust_truck_router::{
    algo::{
        ch::*,
        ch_potential::CHPotential,
        csp_2::{TwoRestrictionDijkstra, TwoRestrictionDijkstraData},
        csp_2_bidir::CSP2BidirAstarCHPotQuery,
    },
    io::*,
    types::*,
};

use rand::Rng;
use std::{
    env,
    error::Error,
    fs::File,
    io::{LineWriter, Write},
    path::Path,
    time::Instant,
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    // let graph_mcd = OwnedOneRestrictionGraph::new(first_out, head, travel_time);
    let graph = OwnedGraph::new(first_out, head, travel_time);

    let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
    let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
    // let s = 61540650;
    // let t = 41065895;
    // let max_driving_time_short = 4_000_000;
    // let pause_time_short = 100_000;
    // let max_driving_time_long = 6_000_000;
    // let pause_time_long = 500_000;

    // let is_routing_node = load_routingkit_bitvector(path.join("is_routing_node"))?;
    // path with distance 20517304
    // let s = is_routing_node.to_local(80232745).unwrap(); // osm_id
    // let t = is_routing_node.to_local(824176810).unwrap(); // osm_id

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    // let mut instance_mcd_acc_state = OneRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    // instance_mcd_acc_state
    //     .set_reset_flags(is_parking_node.to_bytes())
    //     .set_restriction(16_200_000, 2_700_000);
    let mut csp2_astar_state = TwoRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    csp2_astar_state.set_restriction(32_400_000, 32_400_000, 16_200_000, 2_700_000);
    // csp2_astar_state.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);
    let bw_graph = OwnedGraph::reverse(graph.borrow());
    let mut csp2_bidir_astar_query = CSP2BidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());
    csp2_bidir_astar_query.set_restriction(32_400_000, 32_400_000, 16_200_000, 2_700_000);
    // csp2_bidir_astar_query.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    csp2_astar_state.init_new_s(s);

    let csp2_astar = TwoRestrictionDijkstra::new(graph.borrow(), &is_parking_node);
    let timer = Instant::now();
    let unidir_dist = csp2_astar.dist_query(&mut csp2_astar_state, t);
    println!("Elapsed: {}ms", timer.elapsed().as_secs_f64() * 1000.0);
    print!("{}", csp2_astar.summary(&csp2_astar_state));

    csp2_bidir_astar_query.init_new_s(s);
    csp2_bidir_astar_query.init_new_t(t);
    let timer = Instant::now();
    let bidir_dist = csp2_bidir_astar_query.run_query();
    println!("Elapsed bidirectional: {}ms", timer.elapsed().as_secs_f64() * 1000.0);
    print!("{}", csp2_bidir_astar_query.summary());

    // let ns = vec![
    //     170777, 170783, 170567, 170576, 95503, 26090, 393675, 383461, 182538, 91045, 95506, 191564, 98306, 191563, 116635, 116634, 171974, 28831, 95300,
    //     399058, 95513, 98406, 399057, 104243, 98329, 197855, 124802, 28830, 182321, 155200, 399063, 399065, 395107, 168646, 28822, 374638, 28823, 395108,
    //     28824, 126669, 98604, 28825, 100741, 275285, 99339, 87662, 161009, 374623, 99337, 28836, 28826, 187114, 111393, 28827, 28828, 99336, 179254, 347639,
    //     347638, 26176, 26177, 391937, 26178, 26179, 26180, 99331, 99330, 33821, 377796, 99329, 157374, 157371, 157375, 26181, 26182, 33079, 26183, 87929,
    //     233374, 300558, 99328, 324212, 27635, 26184, 233401, 33081, 233384, 99327, 33082, 26185, 344087, 392671, 88310, 225687, 225688, 88626, 225695, 327717,
    //     327772, 296344, 233447, 233448, 227215, 233450, 233451, 232427, 296342, 228566, 13839, 228568, 228570, 13852, 13853, 232428, 229769, 311601, 13838,
    //     399280, 13131, 71805, 325481, 229054, 66129, 71314, 66136, 232434, 232435, 13130, 311668, 311982, 368512, 71318, 82577, 82578, 365904, 13129, 32885,
    //     14658, 325486, 365917, 365914, 195228, 80154, 365916, 365910, 278510, 364775, 365961, 365962, 76500, 340361, 133941, 9172, 327091, 40819, 14603,
    //     133944, 9171, 272307, 9170, 96266, 133943, 93186, 9169, 277710, 16008, 133948, 224480, 133949, 133950, 133958, 159914, 9168, 127309, 133947, 286375,
    //     133477, 8826, 343679, 364208, 269047, 139882, 85414, 111314, 111315, 296350, 365595, 91991, 365596, 91999, 16813, 363816, 363817, 365594, 363818,
    //     363819, 363820, 363821, 365598, 363822, 399279, 363823, 363814, 363824, 365567, 365570, 96418, 365569, 367485, 365568, 294248, 8804, 294246, 22590,
    //     84755, 365562, 365573, 8803, 269053, 365579, 168135, 97812, 22589, 213650, 293018, 97811, 365577, 220575, 293975, 265251, 365581, 213648, 365583,
    //     224476, 340314, 224475, 360649, 213652, 26664, 175799, 360648, 180116, 183072, 180117, 207820, 207819, 152362, 230424, 16832, 319831, 16833, 266618,
    //     230425, 16962, 16960, 329739, 230335, 230336, 110, 112, 302496, 302499, 227831, 309911, 227729, 227730, 227728, 333231, 219997, 219996, 227006, 97,
    //     220285, 96, 5410, 226997, 303218, 226964, 226965, 41435, 226959, 347322, 197193, 220289, 220289, 197194, 220291, 93, 108188, 10998, 145943, 230322,
    //     230323, 395940, 281572, 226663, 80, 108187, 106276, 230309, 230310, 165797, 79, 323708, 221393, 108185, 77, 230311, 230314, 230317, 352201, 230318,
    //     230302, 85, 193353, 238954, 185, 47, 230300, 230301, 230293, 230294, 230292, 280929, 222571, 222202, 222205, 230299, 275487, 222609, 5310, 75328,
    //     303229, 262859, 186, 187, 9244, 9243, 290520, 188, 189, 190, 221804, 191, 265, 221805, 357879, 225576, 192, 193, 225579, 194, 195, 266642, 24088,
    //     221547, 360662, 196, 77493, 197, 225580, 198, 195798, 199, 100874, 100873, 50020, 50019, 278675, 200, 354787, 375462, 201, 292965, 292966, 272, 284,
    //     302260, 202, 302263, 302265, 60470, 60471, 225382, 203, 204, 385226, 395411, 205, 302268, 206, 207, 302269, 353182, 353181, 302317, 230261, 266,
    //     302319, 62322, 267, 353180, 68502, 397828, 68501, 48251, 48252, 220320, 208, 270, 21188, 71938, 225390, 220322, 370, 30533, 225140, 342, 294297,
    //     171935, 172121, 395170, 302552, 165602, 165603, 302554, 302555, 228872, 171939, 347, 302551, 114757, 348, 9, 111412, 124850, 349, 243961, 350, 24834,
    //     228885, 328302, 243963, 351, 228888, 50469, 50467, 50468, 243947, 292271, 228892, 292272, 50133, 243443, 292273, 292274, 243444, 292275, 149164,
    //     149165, 292276, 352, 353, 244075, 319717, 319718, 32094, 32095, 319719, 71921, 67324, 67323, 71920, 167695, 228949, 354, 155994, 155995, 355, 228954,
    //     228957, 244589, 320374, 46610, 46611, 244605, 356, 244622, 50464, 292373, 50463, 244854, 292375, 50461, 50462, 191162, 248909, 191149, 228986, 50460,
    //     357, 50458, 358, 359, 220132, 71111, 71109, 50455, 50456, 5693, 5694, 50457, 343242, 343243, 361, 187023, 292222, 356098, 362, 258077, 258080, 50453,
    //     228273, 228274, 292223, 292225, 228262, 261414, 228263, 261416, 228280, 30537, 228287, 228288, 304, 228257, 170255, 4954, 4955, 229423, 286, 284207,
    //     390, 327803, 328186, 30006, 206056, 350133, 391, 392, 29899, 29898, 29897, 288140, 393, 92320, 385, 386, 387, 229422, 29895, 208054, 230815, 32266,
    //     221571, 221567, 207684, 308513, 388, 207693, 211168, 389, 257536, 5788, 191313, 253783, 191314, 191322, 29700, 230452, 29701, 115494, 29702, 115497,
    //     29703, 249317, 115513, 115511, 115515, 115509, 115517, 115507, 115506, 115505, 226781,
    // ];
    // let (ps, ws) = csp2_astar_state.current_best_path_to(t, true).unwrap();
    // for ((&node, p), w) in ns.iter().zip(ps).zip(ws) {
    //     // csp2_astar.dist_query(&mut csp2_astar_state, t);
    //     csp2_bidir_astar_query.init_new_t(p);
    //     let bidir_dist = csp2_bidir_astar_query.run_query().unwrap();

    //     if w[0] != bidir_dist {
    //         println!("at {}: {:?} vs. {}", p, w, bidir_dist)
    //     }
    // }
    assert_eq!(unidir_dist, bidir_dist);

    // instance_mcd_acc.reset();
    // instance_mcd_acc.dist_query_propagate_all_labels(t);
    // print!("{}", instance_mcd_acc.info());

    let latitude = Vec::<f32>::load_from(path.join("latitude"))?;
    let longitude = Vec::<f32>::load_from(path.join("longitude"))?;
    let osm_node_id = Vec::<u64>::load_from(path.join("osm_node_id"))?;
    assert_eq!(latitude.len(), longitude.len());
    assert_eq!(latitude.len(), graph.num_nodes());

    if let Some((p, _d)) = csp2_astar_state.current_best_path_to(t, true) {
        let file = File::create("path_acc.csv")?;
        let mut file = LineWriter::new(file);
        writeln!(file, "latitude,longitude")?;

        for &node_id in &p {
            writeln!(file, "{},{}", latitude[node_id as usize], longitude[node_id as usize])?;
        }

        let file = File::create("path_flagged_acc.csv")?;
        let mut file = LineWriter::new(file);
        writeln!(file, "latitude,longitude,osm_id,is_parking_used")?;
        let used_p = csp2_astar.reset_nodes_on_path(&(p, _d));
        for &node_id in used_p.0.iter() {
            writeln!(
                file,
                "{},{},{},{}",
                latitude[node_id as usize],
                longitude[node_id as usize],
                osm_node_id[node_id as usize],
                used_p.0.contains(&node_id)
            )?;
        }

        // let (reset_p_short, reset_p_long) = instance_mcd_acc.reset_nodes_on_path(&(p, d));
        // let file = File::create("path_flagged_acc.csv")?;
        // let mut file = LineWriter::new(file);
        // writeln!(file, "latitude,longitude,osm_id,is_parking_short_break,is_parking_long_break")?;

        // for node_id in flagged_p {
        //     writeln!(
        //         file,
        //         "{},{},{},{},{}",
        //         latitude[node_id as usize],
        //         longitude[node_id as usize],
        //         osm_node_id[node_id as usize],
        //         reset_p_short.contains(&node_id),
        //         reset_p_long.contains(&node_id)
        //     )?;
        // }
    }

    let file = File::create("labels_acc.csv")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "latitude,longitude,num_labels")?;

    for (i, n) in csp2_astar_state
        .get_per_node_number_of_labels()
        .into_iter()
        .enumerate()
        .filter(|(_, n)| *n != 0)
    {
        writeln!(file, "{},{},{}", latitude[i], longitude[i], n)?;
    }

    Ok(())
}
