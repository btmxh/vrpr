use smallvec::SmallVec;

use crate::gp::program::{Program, ProgramContext, MAX_PROGRAM_NODE_CHILDREN};

use super::{problem::Request, Simulation};

pub struct RoutingContext<'a> {
    pub sim: &'a Simulation<'a>,
    pub vehicle: usize,
    pub request: &'a Request,
}

pub struct SequencingContext<'a> {
    pub sim: &'a Simulation<'a>,
    pub vehicle: usize,
    pub request: &'a Request,
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
        5 => y.min(x),
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
            0 => self.sim.vehicles[self.vehicle].queue.len() as f32,
            1 => self.sim.vehicles[self.vehicle].total_queued_demand,
            2 => self.sim.vehicles[self.vehicle].time_cost(
                self.sim.problem,
                self.request,
                self.sim.time,
            ),
            _ => unreachable!(),
        }
    }

    fn num_terminals() -> usize {
        3
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
            0 => self.request.demand,
            1 => self.sim.vehicles[self.vehicle].time_cost(
                self.sim.problem,
                self.request,
                self.sim.time,
            ),
            2 => self.sim.time - self.request.open,
            _ => unreachable!(),
        }
    }

    fn num_terminals() -> usize {
        3
    }
}
