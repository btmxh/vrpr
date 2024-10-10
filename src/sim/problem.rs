use std::{
    fs::File,
    io::{BufRead, BufReader},
};

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
}

#[derive(Clone)]
pub struct Problem {
    pub depot: Request,
    pub requests: Vec<Request>,
    pub truck_speed: f32,
    pub truck_capacity: f32,
    pub num_trucks: usize,
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
                service_time: 10.0,
                time: args[7],
            };
            requests.push(req);
        }
        let depot = requests.remove(0);
        Ok(Self {
            depot,
            requests,
            truck_speed,
            truck_capacity,
            num_trucks,
        })
    }

    pub fn to_training(mut self, time_limit: f32) -> Self {
        self.requests.retain(|r| r.time <= time_limit);
        self
    }

    pub fn clone_training(&self, time_limit: f32) -> Self {
        self.clone().to_training(time_limit)
    }

    pub fn total_demand(&self) -> f32 {
        self.requests.iter().map(|r| r.demand).sum()
    }
}
