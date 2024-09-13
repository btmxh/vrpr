use std::{cell::RefCell, env::args};

use gp::GPContext;
use lazy_static::lazy_static;
use log::Logger;
use lru::LruCache;
use ordered_float::OrderedFloat;
use rand::{rngs::SmallRng, Rng, RngCore, SeedableRng};
use sim::{
    ctx::{RoutingProgram, SequencingProgram},
    problem::Problem,
    Simulation,
};

pub mod gp;
pub mod log;
pub mod sim;

const TIME_SLOT: f32 = 50.0;
const NUM_GEN: usize = 100;

lazy_static! {
    static ref HEU: Logger = Logger::new("HEU");
    static ref SIM: Logger = Logger::new("SIM");
    static ref GP: Logger = Logger::new("GP");
    static ref LASTPOP: Logger = Logger::new("LASTPOP");
    static ref ROUTE: Logger = Logger::new("ROUTE");
    static ref DEBUG: Logger = Logger::new("DEBUG");
}

fn fitness(problem: &Problem, result: (f32, usize)) -> f32 {
    let (distance, num_fail) = result;
    let tot_dist = problem.truck_speed * problem.depot.close * problem.num_trucks as f32;
    distance / tot_dist * 0.1 + (num_fail as f32) / (problem.requests.len() as f32)
}

#[allow(non_snake_case)]
fn heuristics(problem: &Problem) -> anyhow::Result<()> {
    let CR = RoutingProgram::terminal(2);
    let CS = SequencingProgram::terminal(1);
    let W = SequencingProgram::terminal(0);
    let WIQ = RoutingProgram::terminal(1);
    for (name, r, s) in [("C+C", &CR, &CS), ("C+W", &CR, &W), ("WIQ+C", &WIQ, &CS)] {
        let mut simulation = Simulation::new(problem, r, s);
        let result = simulation.simulate(TIME_SLOT);
        log!(HEU, "{name} {result:?} {0}", fitness(problem, result));
        for vehicle in 0..problem.num_trucks {
            log!(ROUTE, "{:?}", simulation.get_route(vehicle));
        }
    }
    Ok(())
}

#[derive(Debug)]
struct Individual<'a> {
    routing: RoutingProgram<'a>,
    sequencing: SequencingProgram<'a>,
    pub result: Option<(f32, usize, f32)>,
}

impl<'a> Individual<'a> {
    pub fn ramp_half_and_half(gpc: &GPContext<impl RngCore>) -> Vec<Self> {
        let r_pop = gpc.ramp_half_and_half();
        let s_pop = gpc.ramp_half_and_half();
        r_pop
            .into_iter()
            .zip(s_pop)
            .map(|(routing, sequencing)| Self {
                routing,
                sequencing,
                result: None,
            })
            .collect()
    }

    pub fn crossover_with(&self, gpc: &GPContext<impl RngCore>, other: &Self) -> Self {
        Self {
            routing: gpc.crossover(&self.routing, &other.routing),
            sequencing: gpc.crossover(&self.sequencing, &other.sequencing),
            result: None,
        }
    }

    pub fn mutate(&self, gpc: &GPContext<impl RngCore>) -> Self {
        Self {
            routing: gpc.mutation(&self.routing),
            sequencing: gpc.mutation(&self.sequencing),
            result: None,
        }
    }

    pub fn evaluate(
        &mut self,
        cache: &mut LruCache<String, (f32, usize, f32)>,
        problem: &Problem,
    ) -> f32 {
        if let Some((_, _, fitness)) = self.result {
            return fitness;
        }

        let cache_key = format!("{}:{}", self.routing, self.sequencing);
        let result = *cache.get_or_insert(cache_key, || {
            let (dist, nb_fail) =
                Simulation::new(problem, &self.routing, &self.sequencing).simulate(TIME_SLOT);
            let fitness = fitness(problem, (dist, nb_fail));
            (dist, nb_fail, fitness)
        });

        self.result = Some(result);
        result.2
    }
}

fn select_parent<'a>(gpc: &GPContext<impl RngCore>, pop: &'a [Individual<'a>]) -> usize {
    rand::seq::index::sample(&mut *gpc.rng.borrow_mut(), pop.len(), 8)
        .into_iter()
        .max_by_key(|i| OrderedFloat(pop[*i].result.unwrap().2))
        .unwrap()
}

fn gp(problem: &Problem) -> anyhow::Result<()> {
    let training_problem = problem.clone_training(problem.depot.close * 0.2);
    log!(DEBUG, "{}", training_problem.requests.len());
    let gpc = GPContext {
        rng: RefCell::new(SmallRng::from_entropy()),
        num_population: 100,
        max_depth: 6,
    };
    let mut cache = LruCache::unbounded();
    let mut pop = Individual::ramp_half_and_half(&gpc);
    for gen in 1..=NUM_GEN {
        for i in pop.iter_mut() {
            i.evaluate(&mut cache, &training_problem);
        }

        pop.sort_unstable_by_key(|i| OrderedFloat(i.result.unwrap().2));
        pop.truncate(gpc.num_population);

        log!(GP, "{gen} {:?}", pop[0].result);
        log!(GP, "{}", pop[0].routing);
        log!(GP, "{}", pop[0].sequencing);
        let (dist, nb_fail) =
            Simulation::new(problem, &pop[0].routing, &pop[0].sequencing).simulate(TIME_SLOT);
        log!(
            GP,
            "{:?}",
            (dist, nb_fail, fitness(problem, (dist, nb_fail)))
        );

        if gen == NUM_GEN {
            for i in pop.iter() {
                log!(LASTPOP, "{}", i.routing);
                log!(LASTPOP, "{}", i.sequencing);
            }
        }

        for _ in 0..pop.len() {
            let p1 = select_parent(&gpc, &pop[0..gpc.num_population]);
            if gpc.rng.borrow_mut().gen_bool(0.8) {
                let p2 = select_parent(&gpc, &pop[0..gpc.num_population]);
                let c = pop[p1].crossover_with(&gpc, &pop[p2]);
                pop.push(c);
            } else {
                let m = pop[p1].mutate(&gpc);
                pop.push(m);
            }
        }
    }
    Ok(())
}

fn main() -> anyhow::Result<()> {
    let path = args().nth(1).expect("usage: cargo run -- [problem path]");
    let problem = Problem::load(&path, 50.0 / 60.0, 1300.0, 10)?;
    if HEU.enabled() {
        heuristics(&problem)?;
    }
    if GP.enabled() {
        gp(&problem)?;
    }
    Ok(())
}
