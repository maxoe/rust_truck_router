use std::time::Duration;

use crate::types::Weight;

#[derive(Debug, Clone, Copy)]
pub struct CSPMeasurementResult {
    pub num_queue_pushes: u32,
    pub num_settled: u32,
    pub num_labels_propagated: u32,
    pub num_labels_reset: u32,
    pub num_nodes_searched: usize,
    pub time: Duration,
    pub path_distance: Option<Weight>,
    pub path_number_nodes: Option<usize>,
    pub path_number_flagged_nodes: Option<usize>,
    pub path_number_pauses: Option<usize>,
}

pub trait MeasurementResult {
    const OWN_HEADER: &'static str;
    fn get_header() -> String;
    fn as_csv(&self) -> String;
}

impl MeasurementResult for CSPMeasurementResult {
    const OWN_HEADER : &'static str= "num_queue_pushes,num_settled,num_labels_propagated,num_labels_reset,num_nodes_searched,time_ms,path_distance,path_number_nodes,path_number_flagged_nodes,path_number_pauses";

    fn get_header() -> String {
        Self::OWN_HEADER.to_string()
    }

    fn as_csv(&self) -> String {
        if self.path_distance.is_some() {
            format!(
                "{},{},{},{},{},{},{},{},{},{}",
                self.num_queue_pushes,
                self.num_settled,
                self.num_labels_propagated,
                self.num_labels_reset,
                self.num_nodes_searched,
                self.time.as_secs_f64() * 1000.0,
                self.path_distance.unwrap(),
                self.path_number_nodes.unwrap(),
                self.path_number_flagged_nodes.unwrap(),
                self.path_number_pauses.unwrap()
            )
        } else {
            format!(
                "{},{},{},{},{},{},{},{},{},{}",
                self.num_queue_pushes,
                self.num_settled,
                self.num_labels_propagated,
                self.num_labels_reset,
                self.num_nodes_searched,
                self.time.as_secs_f64() * 1000.0,
                "NaN",
                "NaN",
                "NaN",
                "NaN",
            )
        }
    }
}
