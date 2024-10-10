use smallvec::SmallVec;

use crate::gp::program::{Program, ProgramContext, MAX_PROGRAM_NODE_CHILDREN};

use super::{problem::Request, Simulation, VehicleState};

pub struct RoutingContext<'a> {
    pub sim: &'a Simulation<'a>,
    pub vehicle: usize,
    pub request: &'a Request,
}

pub struct SequencingContext<'a> {
    pub sim: &'a Simulation<'a>,
    pub vehicle: usize,
    pub request: &'a Request,
    pub ready_time: f32,
}

fn common_num_internal() -> usize {
    6
}

fn common_internal_num_children(_: usize) -> usize {
    2
}

fn common_internal(idx: usize, child_values: SmallVec<[f32; MAX_PROGRAM_NODE_CHILDREN]>) -> f32 {
    let x = child_values[0];
    let y = child_values[1];
    match idx {
        0 => x + y,
        1 => x - y,
        2 => x * y,
        3 => {
            if y.abs() < 1e-4 {
                1.0
            } else {
                x / y
            }
        }
        4 => x.min(y),
        5 => x.max(y),
        _ => unreachable!(),
    }
}

pub type RoutingProgram<'a> = Program<RoutingContext<'a>>;
pub type SequencingProgram<'a> = Program<SequencingContext<'a>>;

impl<'a> ProgramContext for RoutingContext<'a> {
    fn internal(
        &self,
        idx: usize,
        child_values: SmallVec<[f32; MAX_PROGRAM_NODE_CHILDREN]>,
    ) -> f32 {
        common_internal(idx, child_values)
    }

    fn internal_num_children(index: usize) -> usize {
        common_internal_num_children(index)
    }

    fn num_internals() -> usize {
        common_num_internal()
    }

    fn terminal(&self, idx: usize) -> f32 {
        match idx {
            0 => {
                self.sim.vehicles[self.vehicle].queue.len() as f32
                    / self.sim.problem.requests.len() as f32
            }
            1 => {
                self.sim.vehicles[self.vehicle]
                    .queue
                    .iter()
                    .map(|r| r.0.demand)
                    .sum::<f32>()
                    / self.sim.problem.total_demand()
            }
            2 => {
                (self.sim.vehicles[self.vehicle].busy_until - self.sim.time)
                    / self.sim.problem.depot.close
            }
            3 => {
                self.sim.vehicles[self.vehicle].raw_time_cost(
                    self.sim.problem,
                    self.request,
                    self.sim.time,
                ) / self.sim.problem.depot.close
            }
            4 => {
                let pts = self.sim.vehicles[self.vehicle].queue.iter().map(|r| {
                    self.sim.vehicles[self.vehicle].raw_time_cost(
                        self.sim.problem,
                        r.0,
                        self.sim.time,
                    ) / self.sim.problem.depot.close
                });
                VehicleState::median(pts)
            }
            6 => self.request.demand / self.sim.problem.total_demand(),
            7 | 5 => self.sim.time / self.sim.problem.depot.close,
            8 => self.request.time / self.sim.problem.depot.close,
            9 => self.request.service_time / self.sim.problem.depot.close,
            10 => self.request.open / self.sim.problem.depot.close,
            11 => self.request.close / self.sim.problem.depot.close,
            12 => {
                let (x, y) = self.sim.vehicles[self.vehicle].median_queue_pos();
                let (rx, ry) = (self.request.x, self.request.y);
                ((x - rx) * (x - rx) + (y - ry) * (y - ry)).sqrt()
                    / self.sim.problem.truck_speed
                    / self.sim.problem.depot.close
            }
            _ => unreachable!(),
        }
    }

    fn num_terminals() -> usize {
        13
    }
}

impl<'a> ProgramContext for SequencingContext<'a> {
    fn internal(
        &self,
        idx: usize,
        child_values: SmallVec<[f32; MAX_PROGRAM_NODE_CHILDREN]>,
    ) -> f32 {
        common_internal(idx, child_values)
    }

    fn internal_num_children(index: usize) -> usize {
        common_internal_num_children(index)
    }

    fn num_internals() -> usize {
        common_num_internal()
    }

    fn terminal(&self, idx: usize) -> f32 {
        match idx {
            0 => {
                self.sim.vehicles[self.vehicle].queue.len() as f32
                    / self.sim.problem.requests.len() as f32
            }
            1 => {
                self.sim.vehicles[self.vehicle]
                    .queue
                    .iter()
                    .map(|r| r.0.demand)
                    .sum::<f32>()
                    / self.sim.problem.total_demand()
            }
            2 => {
                (self.sim.vehicles[self.vehicle].busy_until - self.sim.time)
                    / self.sim.problem.depot.close
            }
            3 => {
                self.sim.vehicles[self.vehicle].raw_time_cost(
                    self.sim.problem,
                    self.request,
                    self.sim.time,
                ) / self.sim.problem.depot.close
            }
            4 => {
                let pts = self.sim.vehicles[self.vehicle].queue.iter().map(|r| {
                    self.sim.vehicles[self.vehicle].raw_time_cost(
                        self.sim.problem,
                        r.0,
                        self.sim.time,
                    ) / self.sim.problem.depot.close
                });
                VehicleState::median(pts)
            }
            5 => self.ready_time / self.sim.problem.depot.close,
            6 => self.request.demand / self.sim.problem.total_demand(),
            7 => self.sim.time / self.sim.problem.depot.close,
            8 => self.request.time / self.sim.problem.depot.close,
            9 => self.request.service_time / self.sim.problem.depot.close,
            10 => self.request.open / self.sim.problem.depot.close,
            11 => self.request.close / self.sim.problem.depot.close,
            12 => {
                let (x, y) = self.sim.vehicles[self.vehicle].median_queue_pos();
                let (rx, ry) = (self.request.x, self.request.y);
                ((x - rx) * (x - rx) + (y - ry) * (y - ry)).sqrt()
                    / self.sim.problem.truck_speed
                    / self.sim.problem.depot.close
            }
            _ => unreachable!(),
        }
    }

    fn num_terminals() -> usize {
        return 13;
    }
}
