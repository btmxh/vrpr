use std::{
    cmp::Reverse,
    collections::{BinaryHeap, HashMap},
};

use ordered_float::OrderedFloat;

use crate::{log, ROUTE, SIM};

use self::{
    ctx::{RoutingContext, RoutingProgram, SequencingContext, SequencingProgram},
    problem::{Problem, Request},
};

pub mod ctx;
pub mod problem;

pub enum Event<'a> {
    Requests(Vec<&'a Request>, f32),
    VehicleFinish {
        vehicle: usize,
        request: &'a Request,
        time: f32,
    },
}

impl Event<'_> {
    pub fn time(&self) -> f32 {
        match self {
            Self::Requests(_, time) => *time,
            Self::VehicleFinish { time, .. } => *time,
        }
    }

    pub fn time_ordered(&self) -> OrderedFloat<f32> {
        OrderedFloat(self.time())
    }
}

impl PartialEq for Event<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.time_ordered().eq(&other.time_ordered())
    }
}

impl PartialOrd for Event<'_> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for Event<'_> {}
impl Ord for Event<'_> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        OrderedFloat(self.time()).cmp(&OrderedFloat(other.time()))
    }
}

pub struct VehicleState<'a> {
    cur_request: &'a Request,
    queue: Vec<(&'a Request, f32)>,
    total_queued_demand: f32,
    total_demand: f32,
    busy_until: f32,
    route: HashMap<usize, f32>,
    dropped: HashMap<usize, f32>,
}

impl<'a> VehicleState<'a> {
    pub fn new(problem: &'a Problem) -> Self {
        Self {
            cur_request: &problem.depot,
            queue: Vec::new(),
            total_demand: problem.truck_capacity,
            total_queued_demand: 0.0,
            busy_until: 0.0,
            route: Default::default(),
            dropped: Default::default(),
        }
    }

    pub fn time_cost(&self, problem: &'a Problem, req: &'a Request, time: f32) -> f32 {
        (self.distance_to(req) / problem.truck_speed).max(req.open - time)
    }

    pub fn raw_time_cost(&self, problem: &'a Problem, req: &'a Request, _: f32) -> f32 {
        self.distance_to(req) / problem.truck_speed
    }

    pub fn time_until_open(&self, req: &'a Request, time: f32) -> f32 {
        time - req.time
    }

    pub fn distance_to(&self, request: &'a Request) -> f32 {
        Self::dist(
            self.cur_request.x - request.x,
            self.cur_request.y - request.y,
        )
    }

    fn dist(x: f32, y: f32) -> f32 {
        (x * x + y * y).sqrt()
    }

    pub fn enqueue(&mut self, request: &'a Request, time: f32) {
        self.queue.push((request, time));
        self.total_queued_demand += request.demand;
    }

    pub fn median(x: impl Iterator<Item = f32>) -> f32 {
        // match x.len() {
        //     0 => 0.0,
        //     n => x.iter().copied().sum::<f32>() / n as f32,
        // }
        let mut x: Vec<f32> = x.collect();
        x.sort_unstable_by_key(|f| OrderedFloat::from(*f));
        match x.len() {
            0 => 0.0,
            n if n % 2 == 1 => x[n / 2],
            n => 0.5 * (x[n / 2] + x[n / 2 - 1]),
        }
    }

    pub fn median_queue_pos(&self) -> (f32, f32) {
        let x = self.queue.iter().map(|r| r.0.x);
        let y = self.queue.iter().map(|r| r.0.y);
        (Self::median(x), Self::median(y))
    }
}

pub struct Simulation<'a> {
    problem: &'a Problem,
    routing_rule: &'a RoutingProgram<'a>,
    sequencing_rule: &'a SequencingProgram<'a>,
    time: f32,
    vehicles: Vec<VehicleState<'a>>,
    events: BinaryHeap<Reverse<Event<'a>>>,
}

impl<'a> Simulation<'a> {
    pub fn new(
        problem: &'a Problem,
        routing_rule: &'a RoutingProgram<'a>,
        sequencing_rule: &'a SequencingProgram<'a>,
    ) -> Self {
        Self {
            problem,
            routing_rule,
            sequencing_rule,
            time: 0.0,
            vehicles: (0..problem.num_trucks)
                .map(|_| VehicleState::new(problem))
                .collect(),
            events: BinaryHeap::new(),
        }
    }

    pub fn simulate(&mut self, time_slot: f32) -> (f32, usize) {
        let mut batched_requests = HashMap::<i32, Vec<&'a Request>>::new();
        for request in self.problem.requests.iter() {
            let timeslot_idx = (request.time / time_slot).ceil() as i32;
            batched_requests
                .entry(timeslot_idx)
                .or_default()
                .push(request);
        }

        for (idx, requests) in batched_requests {
            self.events
                .push(Reverse(Event::Requests(requests, idx as f32 * time_slot)));
        }

        let mut total_distance = 0f32;
        let mut total_failed = 0usize;
        while let Some(Reverse(event)) = self.events.pop() {
            self.time = event.time();
            log!(SIM, "sim_time", time = self.time);
            match event {
                Event::Requests(requests, _) => {
                    for request in requests {
                        self.handle_request(request)
                    }
                }
                Event::VehicleFinish {
                    vehicle, request, ..
                } => self.handle_vehicle_finish(vehicle, request),
            }
            for vehicle in 0..self.problem.num_trucks {
                self.update_vehicle_queue(vehicle, &mut total_failed, &mut total_distance);
            }
        }

        for vehicle in 0..self.problem.num_trucks {
            self.route_vehicle_to(vehicle, &self.problem.depot, &mut total_distance);
        }
        for vehicle in 0..self.problem.num_trucks {
            log!(
                ROUTE,
                "route_log",
                vehicle = vehicle,
                route = self.vehicles[vehicle].route,
                dropped = self.vehicles[vehicle].dropped
            );
        }

        (total_distance, total_failed)
    }

    fn handle_request(&mut self, request: &'a Request) {
        let vehicles = 0..self.vehicles.len();
        let vehicle = vehicles
            .min_by_key(|vehicle| {
                (
                    OrderedFloat(self.routing_rule.calc(&RoutingContext {
                        sim: self,
                        vehicle: *vehicle,
                        request,
                    })),
                    // self.vehicles[*vehicle].queue.len(),
                )
            })
            .expect("`vehicles` should not be empty");
        self.vehicles[vehicle].enqueue(request, self.time);
        log!(
            SIM,
            "vehicle_assigned",
            vehicle = vehicle,
            request = request.idx
        );
    }

    fn handle_vehicle_finish(&mut self, vehicle: usize, request: &'a Request) {
        log!(
            SIM,
            "vehicle_served",
            vehicle = vehicle,
            request = request.idx
        );
    }

    fn update_vehicle_queue(
        &mut self,
        vehicle: usize,
        total_failed: &mut usize,
        total_distance: &mut f32,
    ) {
        if self.time < self.vehicles[vehicle].busy_until {
            return;
        }

        let mut cache = HashMap::<usize, OrderedFloat<f32>>::new();

        while let Some(index) = (0..self.vehicles[vehicle].queue.len()).min_by_key(|i| {
            let request_idx = self.vehicles[vehicle].queue[*i].0.idx;
            *cache.entry(request_idx).or_insert_with(|| {
                OrderedFloat(self.sequencing_rule.calc(&SequencingContext {
                    sim: self,
                    vehicle,
                    request: self.vehicles[vehicle].queue[*i].0,
                    ready_time: self.vehicles[vehicle].queue[*i].1,
                }))
            })
        }) {
            let queue = &mut self.vehicles[vehicle].queue;
            let request = queue[index].0;
            if request.demand > self.vehicles[vehicle].total_demand {
                // return to depot
                self.route_vehicle_to(vehicle, &self.problem.depot, total_distance);
                return;
            }

            self.vehicles[vehicle].queue.swap_remove(index);
            self.vehicles[vehicle].total_queued_demand -= request.demand;
            let start_time =
                self.time + self.vehicles[vehicle].time_cost(self.problem, request, self.time);
            if start_time > request.close {
                self.vehicles[vehicle]
                    .dropped
                    .insert(request.idx, start_time);
                *total_failed += 1;
                log!(SIM, "vehicle_skipped", request = request.idx);
                continue;
            }

            self.route_vehicle_to(vehicle, request, total_distance);
        }
    }

    fn route_vehicle_to(&mut self, vehicle: usize, request: &'a Request, total_distance: &mut f32) {
        let state = &mut self.vehicles[vehicle];
        let distance = state.distance_to(request);
        *total_distance += distance;
        let time = (self.time + distance / self.problem.truck_speed).max(request.open)
            + request.service_time;
        if request.idx == 0 {
            state.total_demand = self.problem.truck_capacity;
        } else {
            state.total_demand -= request.demand;
        }
        self.events.push(Reverse(Event::VehicleFinish {
            vehicle,
            request,
            time,
        }));
        state.route.insert(request.idx, time - request.service_time);
        state.cur_request = request;
        state.busy_until = time;
        log!(
            SIM,
            "vehicle_new_serve",
            request = request.idx,
            busy_until = time
        );
    }
}
