use std::{
    fmt::{self, Debug, Display, Formatter},
    marker::PhantomData,
};

use smallvec::SmallVec;

pub const MAX_PROGRAM_NODE_CHILDREN: usize = 2;

pub trait ProgramContext {
    fn num_terminals() -> usize;
    fn num_internals() -> usize;
    fn internal_num_children(index: usize) -> usize;

    fn terminal(&self, index: usize) -> f32;
    fn internal(
        &self,
        index: usize,
        child_values: SmallVec<[f32; MAX_PROGRAM_NODE_CHILDREN]>,
    ) -> f32;

    fn format_terminal(index: usize, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "TERM{index}")
    }

    fn format_internal(index: usize, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "INT{index}")
    }
}

#[derive(Debug)]
pub enum Node {
    Const(f32),
    Terminal(usize),
    Internal(usize),
}

struct DisplayNode<'p, C: ProgramContext> {
    program: &'p Program<C>,
    index: usize,
}

impl<'p, C: ProgramContext> Display for DisplayNode<'p, C> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match Node::from(self.program.nodes[self.index]) {
            Node::Const(value) => write!(f, "{}", value)?,
            Node::Terminal(index) => C::format_terminal(index, f)?,
            Node::Internal(index) => {
                C::format_internal(index, f)?;
                write!(f, "(")?;
                for (i, child_index) in
                    Program::<C>::child_indices(self.index, C::internal_num_children(index))
                        .enumerate()
                {
                    if i != 0 {
                        write!(f, ", ")?;
                    }
                    write!(
                        f,
                        "{}",
                        DisplayNode {
                            program: self.program,
                            index: child_index
                        }
                    )?;
                }
                write!(f, ")")?;
            }
        }
        Ok(())
    }
}

impl From<u8> for Node {
    fn from(x: u8) -> Self {
        match x {
            0..=128 => Self::Const(f32::from(i16::from(x) - 64) / 16.0),
            129..=192 => Self::Terminal((x - 129).into()),
            193..=255 => Self::Internal((x - 193).into()),
        }
    }
}

impl From<Node> for u8 {
    fn from(value: Node) -> Self {
        match value {
            Node::Const(x) => (x * 16.0 + 64.0) as u8,
            Node::Terminal(index) => (129 + index).try_into().unwrap(),
            Node::Internal(index) => (193 + index).try_into().unwrap(),
        }
    }
}

#[derive(Default)]
pub struct Program<C: ProgramContext> {
    // 0-128: const values (normalized into x/16 - 1)
    // 129-192: terminals
    // 193-255: internals
    pub nodes: Vec<u8>,
    _marker: PhantomData<C>,
}

impl<C: ProgramContext> Clone for Program<C> {
    fn clone(&self) -> Self {
        Self {
            nodes: self.nodes.clone(),
            _marker: PhantomData,
        }
    }
}

impl<C: ProgramContext> Debug for Program<C> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct("Program")
            .field(
                "nodes",
                &self
                    .nodes
                    .iter()
                    .map(|val| Node::from(*val))
                    .collect::<Vec<_>>(),
            )
            .finish()
    }
}

impl<C: ProgramContext> Program<C> {
    pub fn new() -> Self {
        Self {
            nodes: vec![],
            _marker: PhantomData,
        }
    }

    pub fn terminal(index: usize) -> Self {
        Self {
            nodes: vec![Node::Terminal(index).into()],
            _marker: PhantomData,
        }
    }

    pub fn child_indices(index: usize, num_children: usize) -> impl Iterator<Item = usize> {
        (0..num_children).map(move |i| index * MAX_PROGRAM_NODE_CHILDREN + i + 1)
    }

    pub fn ensure_index_exist(&mut self, index: usize) {
        self.nodes.resize(self.nodes.len().max(index + 1), 0);
    }

    pub fn generate_at(
        &mut self,
        index: usize,
        num_children: usize,
        value: u8,
        mut gen_child_fn: impl FnMut(&mut Self, usize, usize),
    ) {
        self.ensure_index_exist(index);
        self.nodes[index] = value;
        for (i, child_index) in Self::child_indices(index, num_children).enumerate() {
            gen_child_fn(self, i, child_index);
        }
    }

    pub fn calc_at(&self, c: &C, index: usize) -> f32 {
        match Node::from(self.nodes[index]) {
            Node::Const(x) => x,
            Node::Terminal(i) => c.terminal(i),
            Node::Internal(i) => {
                let num_children = C::internal_num_children(i);
                let child_values = Self::child_indices(index, num_children)
                    .map(|child_index| self.calc_at(c, child_index))
                    .collect();
                c.internal(i, child_values)
            }
        }
    }

    pub fn calc(&self, c: &C) -> f32 {
        self.calc_at(c, 0)
    }

    pub fn collect_all_active_indices(&self, dest: &mut Vec<usize>, index: usize) {
        dest.push(index);
        if let Node::Internal(i) = Node::from(self.nodes[index]) {
            for child_index in Self::child_indices(index, C::internal_num_children(i)) {
                self.collect_all_active_indices(dest, child_index);
            }
        }
    }

    pub fn all_active_indices(&self) -> Vec<usize> {
        let mut indices = Vec::new();
        self.collect_all_active_indices(&mut indices, 0);
        indices
    }
}

impl<C: ProgramContext> Display for Program<C> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            DisplayNode {
                program: self,
                index: 0,
            }
        )
    }
}
