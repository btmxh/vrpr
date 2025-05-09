use std::{
    fs::File,
    io::{BufRead, BufReader},
};

use anyhow::{anyhow, Context};
use ordered_float::OrderedFloat;
use serde::Deserialize;

#[derive(Clone, Copy)]
pub struct Request {
    pub idx: usize,
    pub x: f32,
    pub y: f32,
    pub demand: f32,
    pub open: f32,
    pub close: f32,
    pub service_time: f32,
    pub time: f32,
    pub drone_serve: bool,
}

#[derive(Clone)]
pub struct VehicleFamily {
    pub speed: f32,
    pub drone: bool,
    pub capacity: f32,
    pub count: usize,
    pub charge_limit: f32,
}

#[derive(Clone)]
pub struct Problem {
    pub depot: Request,
    pub requests: Vec<Request>,
    pub vehicles: Vec<VehicleFamily>,
}

#[derive(Deserialize)]
pub struct Pj2ProblemFormat {
    truck_vel: f32,
    drone_vel: f32,
    truck_cap: f32,
    drone_cap: f32,
    drone_lim: f32,
    truck_num: usize,
    drone_num: usize,
    requests: Vec<Vec<f32>>,
    close: f32,
}

impl Problem {
    pub fn load(
        csv: &str,
        truck_speed: f32,
        truck_capacity: f32,
        num_trucks: usize,
    ) -> anyhow::Result<Problem> {
        let file = BufReader::new(File::open(csv)?);
        let mut requests = Vec::new();
        let lines = file.lines().skip(1);
        for (idx, line) in lines.enumerate() {
            let args = line?
                .split(',')
                .map(|tok| tok.parse::<f32>())
                .collect::<Result<Vec<f32>, _>>()?;
            let req = Request {
                idx,
                x: args[0],
                y: args[1],
                demand: args[2],
                open: args[3],
                close: args[4],
                service_time: 0.0,
                time: args[7],
                drone_serve: false,
            };
            requests.push(req);
        }
        let depot = requests.remove(0);
        Ok(Self {
            depot,
            requests,
            vehicles: vec![VehicleFamily {
                speed: truck_speed,
                count: num_trucks,
                capacity: truck_capacity,
                drone: false,
                charge_limit: f32::INFINITY,
            }],
        })
    }

    pub fn load_pj2(json: &str) -> anyhow::Result<Problem> {
        let pj2_problem: Pj2ProblemFormat =
            serde_json::from_reader(File::open(json).context("unable to open problem file")?)?;
        let requests: Vec<_> = pj2_problem
            .requests
            .into_iter()
            .enumerate()
            .map(|(idx, r)| Request {
                idx: idx + 1,
                x: r[0],
                y: r[1],
                demand: r[2],
                drone_serve: r[3] > 0.5,
                time: r[4],
                open: r[5],
                close: r[6],
                service_time: 0.0,
            })
            .collect();
        let depot_close = pj2_problem.close;
        Ok(Problem {
            depot: Request {
                idx: 0,
                x: 0.0,
                y: 0.0,
                demand: 0.0,
                drone_serve: true,
                time: 0.0,
                open: 0.0,
                close: depot_close,
                service_time: 0.0,
            },
            requests,
            vehicles: vec![
                VehicleFamily {
                    capacity: pj2_problem.truck_cap,
                    speed: pj2_problem.truck_vel,
                    drone: false,
                    count: pj2_problem.truck_num,
                    charge_limit: f32::INFINITY,
                },
                VehicleFamily {
                    capacity: pj2_problem.drone_cap,
                    speed: pj2_problem.drone_vel,
                    drone: true,
                    count: pj2_problem.drone_num,
                    charge_limit: pj2_problem.drone_lim,
                },
            ],
        })
    }

    pub fn clone_training(&self, time_limit: f32, stress_factor: f32) -> Self {
        let mut requests = Vec::new();
        let mut current_index = 0;
        let mut turn = 0.0f32;
        for mut req in self.requests.iter().cloned() {
            req.x *= stress_factor;
            req.y *= stress_factor;
            req.service_time *= stress_factor;
            if req.time > time_limit {
                let time_req = self.requests[current_index];
                req.time = time_limit * turn + (time_req.time + time_req.open * 1.5) / 2.5;
                req.open = time_limit * turn + time_req.open;
                req.close = time_limit * turn + time_req.close;

                current_index += 1;
                if self.requests[current_index].time > time_limit {
                    current_index = 0;
                    turn += 1.0;
                }
            }
            requests.push(req);
        }
        Self {
            depot: self.depot,
            requests,
            vehicles: self.vehicles.clone(),
        }
    }

    pub fn total_demand(&self) -> f32 {
        self.requests.iter().map(|r| r.demand).sum()
    }

    pub fn distance(&self, r1: &Request, r2: &Request) -> f32 {
        ((r1.y - r2.y).powi(2) + (r1.x - r2.x).powi(2)).sqrt()
    }
}
