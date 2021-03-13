use crate::commanders::CommanderState;
use crate::estimator::EstimatorState;
use crate::tracker::RobotState;

pub struct State<Node> {
    current_node: Node,
    robot_state: RobotState,
}

impl<Node> State<Node> {
    pub fn new(current_node: Node, robot_state: RobotState) -> Self {
        Self {
            current_node,
            robot_state,
        }
    }
}

macro_rules! impl_into {
    ($into: ident { $($into_field: ident: $from_field: ident,)* }) => {
        impl<'a, Node> Into<$into>
            for &'a State<Node>
        {
            fn into(self) -> $into {
                $into {
                    $($into_field: self.$from_field.clone(),)+
                }
            }
        }
    };

    ($into: ident < $($param: ident),* > { $($into_field: ident: $from_field: ident,)* }) => {
        impl<'a, Node> Into<$into<$($param),*>>
            for &'a State<Node>
        where $($param: Clone),*
        {
            fn into(self) -> $into<$($param),*> {
                $into {
                    $($into_field: self.$from_field.clone(),)+
                }
            }
        }
    };
}

impl_into! {
    EstimatorState {
        state: robot_state,
    }
}

impl_into! {
    CommanderState<Node> {
        current_node: current_node,
    }
}
