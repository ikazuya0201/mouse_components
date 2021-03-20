use core::cmp::Reverse;
use core::convert::{Infallible, TryFrom, TryInto};
use core::fmt::Debug;

use generic_array::{ArrayLength, GenericArray};
use heap::BinaryHeap;
use heapless::{consts::*, Vec};
use num::{Bounded, Saturating};
use spin::Mutex;
use typenum::Unsigned;

use super::{
    compute_shortest_path, BoundedNode, BoundedPathNode, GoalSizeUpperBound, Graph, RouteNode,
};
use crate::operators::{SearchCommander as ISearchCommander, SearchCommanderError};
use crate::utils::itertools::repeat_n;

/// A trait that converts a given path to nodes which are on the path.
pub trait GraphConverter<Node> {
    type SearchNode;
    type SearchNodes: IntoIterator<Item = Self::SearchNode>;

    ///checker nodes are search nodes on which agent can check
    ///if walls on the given path exist.
    fn convert_to_checker_nodes<Nodes: core::ops::Deref<Target = [Node]>>(
        &self,
        path: Nodes,
    ) -> Self::SearchNodes;
}

/// A trait that checks whether a given node can be visited or not.
///
/// This returns None when it does not have enough information to check.
pub trait NodeChecker<Node> {
    fn is_available(&self, node: &Node) -> Option<bool>;
}

/// A trait that returns a node determined by itself and a given route.
pub trait NextNode<Route>: Sized {
    type Error;

    fn next(&self, route: &Route) -> Result<Self, Self::Error>;
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
enum State {
    Waiting,
    Solving,
    Initial,
    Final,
}

type CandidateSizeUpperBound = U4;

/// An implementation of [SearchCommander](crate::operators::SearchCommander) required by
/// [SearchOperator](crate::operators::SearchOperator).
pub struct SearchCommander<Node, RunNode, SearchNode, Route, Maze> {
    start: RunNode,
    goals: Vec<RunNode, GoalSizeUpperBound>,
    initial_route: Route,
    final_route: Route,
    current: Mutex<Node>,
    state: Mutex<State>,
    candidates: Mutex<Vec<SearchNode, CandidateSizeUpperBound>>,
    maze: Maze,
}

impl<Node, RunNode, SearchNode, Route, Maze> SearchCommander<Node, RunNode, SearchNode, Route, Maze>
where
    RunNode: Clone,
{
    pub fn new(
        start: RunNode,
        goals: &[RunNode],
        current: Node,
        initial_route: Route,
        final_route: Route,
        maze: Maze,
    ) -> Self {
        Self {
            start: start.clone(),
            goals: goals.into_iter().cloned().collect(),
            initial_route,
            final_route,
            current: Mutex::new(current),
            state: Mutex::new(State::Initial),
            candidates: Mutex::new(Vec::new()),
            maze,
        }
    }
}

impl<Node, RunNode, SearchNode, Route, Maze> core::fmt::Debug
    for SearchCommander<Node, RunNode, SearchNode, Route, Maze>
where
    Maze: core::fmt::Debug,
    SearchNode: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "candidates: {:?}", self.candidates.lock())?;
        writeln!(f, "maze: {:?}", &self.maze)
    }
}

impl<Node, RunNode, Cost, Maze> ISearchCommander
    for SearchCommander<
        Node,
        RunNode,
        Maze::SearchNode,
        <Maze::SearchNode as RouteNode>::Route,
        Maze,
    >
where
    RunNode::UpperBound: ArrayLength<Maze::SearchNode>
        + ArrayLength<Cost>
        + ArrayLength<Reverse<Cost>>
        + ArrayLength<(RunNode, Reverse<Cost>)>
        + ArrayLength<(Maze::SearchNode, Reverse<Cost>)>
        + ArrayLength<Option<RunNode>>
        + ArrayLength<Option<usize>>
        + Unsigned,
    RunNode::PathUpperBound: ArrayLength<RunNode>,
    RunNode: PartialEq + Copy + Debug + Into<usize> + BoundedNode + BoundedPathNode,
    Maze::SearchNode: PartialEq + Copy + Debug + Into<usize> + RouteNode + TryFrom<Node>,
    Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<RunNode, Cost = Cost>
        + Graph<<Maze as GraphConverter<RunNode>>::SearchNode, Cost = Cost>
        + GraphConverter<RunNode>
        + NodeChecker<<Maze as GraphConverter<RunNode>>::SearchNode>,
    Node: From<Maze::SearchNode> + Clone + NextNode<<Maze::SearchNode as RouteNode>::Route>,
    <Maze::SearchNode as RouteNode>::Route: Clone,
    <Maze::SearchNode as RouteNode>::Error: core::fmt::Debug,
{
    type Error = Infallible;
    type Command = (Node, <Maze::SearchNode as RouteNode>::Route);

    //must return the current pose and next kind
    //TODO: write test
    fn next_command(&self) -> Result<Self::Command, SearchCommanderError<Self::Error>> {
        let mut state = self.state.lock();
        match *state {
            State::Initial => {
                let mut current = self.current.lock();
                let mut next = current
                    .next(&self.initial_route)
                    .unwrap_or_else(|_| unreachable!());
                core::mem::swap(&mut *current, &mut next);
                *state = State::Solving;
                Ok((next, self.initial_route.clone()))
            }
            State::Final => Err(SearchCommanderError::SearchFinish),
            State::Solving => {
                let mut current = self.current.lock();
                if let Some(candidates) = self.next_node_candidates(
                    &current
                        .clone()
                        .try_into()
                        .unwrap_or_else(|_| unimplemented!()),
                ) {
                    *self.candidates.lock() = candidates;
                    *state = State::Waiting;
                    Err(SearchCommanderError::Waiting)
                } else {
                    let tmp = current.clone();
                    *current = current
                        .next(&self.final_route)
                        .unwrap_or_else(|_| unreachable!());
                    *state = State::Final;
                    Ok((tmp, self.final_route.clone()))
                }
            }
            State::Waiting => {
                let mut next = None;
                let mut candidates = self.candidates.lock();
                if candidates.is_empty() {
                    return Err(SearchCommanderError::Waiting);
                }
                for &node in candidates.iter() {
                    let is_available = self
                        .maze
                        .is_available(&node)
                        .ok_or(SearchCommanderError::Waiting)?;
                    if is_available {
                        next = Some(node);
                        break;
                    }
                }
                candidates.clear();
                let next = next.unwrap_or_else(|| unreachable!("This is bug."));
                let kind = {
                    <Maze::SearchNode as TryFrom<Node>>::try_from(self.current.lock().clone())
                        .unwrap_or_else(|_| unimplemented!())
                        .route(&next)
                        .unwrap_or_else(|err| unreachable!("This is bug: {:?}", err))
                };
                let mut next = Node::from(next);
                let mut current = self.current.lock();
                core::mem::swap(&mut next, &mut current);
                *state = State::Solving;
                Ok((next, kind))
            }
        }
    }
}

//TODO: write test
impl<Node, RunNode, Cost, Route, Maze> SearchCommander<Node, RunNode, Maze::SearchNode, Route, Maze>
where
    RunNode::UpperBound: ArrayLength<Maze::SearchNode>
        + ArrayLength<Cost>
        + ArrayLength<Reverse<Cost>>
        + ArrayLength<(RunNode, Reverse<Cost>)>
        + ArrayLength<(Maze::SearchNode, Reverse<Cost>)>
        + ArrayLength<Option<RunNode>>
        + ArrayLength<Option<usize>>
        + Unsigned,
    RunNode::PathUpperBound: ArrayLength<RunNode>,
    RunNode: PartialEq + Copy + Debug + Into<usize> + BoundedNode + BoundedPathNode,
    Maze::SearchNode: PartialEq + Copy + Debug + Into<usize>,
    Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<RunNode, Cost = Cost>
        + Graph<<Maze as GraphConverter<RunNode>>::SearchNode, Cost = Cost>
        + GraphConverter<RunNode>,
{
    fn next_node_candidates(
        &self,
        current: &Maze::SearchNode,
    ) -> Option<Vec<Maze::SearchNode, U4>> {
        let shortest_path = compute_shortest_path(&self.start, &self.goals, &self.maze)?;
        let checker_nodes = self.maze.convert_to_checker_nodes(shortest_path);

        let candidates = self
            .maze
            .successors(current)
            .into_iter()
            .collect::<Vec<(Maze::SearchNode, Cost), U4>>();

        let mut dists = repeat_n(Cost::max_value(), <RunNode::UpperBound as Unsigned>::USIZE)
            .collect::<GenericArray<_, RunNode::UpperBound>>();
        let mut heap = BinaryHeap::<Maze::SearchNode, Reverse<Cost>, RunNode::UpperBound>::new();
        for node in checker_nodes {
            heap.push_or_update(node, Reverse(Cost::min_value()))
                .unwrap();
            dists[node.into()] = Cost::min_value();
        }
        while let Some((node, Reverse(cost))) = heap.pop() {
            if candidates.iter().any(|&(cand, _)| cand == node) {
                continue;
            }
            for (next, edge_cost) in self.maze.predecessors(&node) {
                let next_cost = cost.saturating_add(edge_cost);
                if dists[next.into()] > next_cost {
                    dists[next.into()] = next_cost;
                    heap.push_or_update(next, Reverse(next_cost)).unwrap();
                }
            }
        }

        let mut candidates = candidates
            .into_iter()
            .map(|(node, cost)| (node, cost.saturating_add(dists[node.into()])))
            .filter(|&(_, cost)| cost < Cost::max_value())
            .collect::<Vec<(Maze::SearchNode, Cost), U4>>();

        if candidates.is_empty() {
            return None;
        }

        candidates.sort_unstable_by_key(|elem| elem.1);
        Some(candidates.into_iter().map(|(node, _)| node).collect())
    }
}

impl<Node, RunNode, SearchNode, Route, Maze> SearchCommander<Node, RunNode, SearchNode, Route, Maze>
where
    RunNode: BoundedPathNode + BoundedNode + Clone + Into<usize> + PartialEq,
    RunNode::PathUpperBound: ArrayLength<RunNode>,
    RunNode::UpperBound: Unsigned
        + ArrayLength<Maze::Cost>
        + ArrayLength<Option<RunNode>>
        + ArrayLength<(RunNode, Reverse<Maze::Cost>)>
        + ArrayLength<Option<usize>>,
    Maze: Graph<RunNode>,
    Maze::Cost: Bounded + Saturating + Copy + Ord,
{
    pub fn compute_shortest_path(&self) -> Option<Vec<RunNode, RunNode::PathUpperBound>> {
        compute_shortest_path(&self.start, &self.goals, &self.maze)
    }
}
