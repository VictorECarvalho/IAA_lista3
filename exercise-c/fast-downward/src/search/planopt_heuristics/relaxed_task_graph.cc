#include "relaxed_task_graph.h"

#include <iostream>
#include <vector>

using namespace std;

namespace planopt_heuristics {
RelaxedTaskGraph::RelaxedTaskGraph(const TaskProxy &task_proxy)
    : relaxed_task(task_proxy),
      variable_node_ids(relaxed_task.propositions.size()) {
    for (size_t i = 0; i < relaxed_task.propositions.size(); ++i) {
        variable_node_ids[i] = graph.add_node(NodeType::OR);
    }

    for (const auto &op : relaxed_task.operators) {
        NodeID precondition_node = graph.add_node(NodeType::AND);
        for (int precondition : op.preconditions) {
            graph.add_edge(precondition_node, variable_node_ids[precondition]);
        }

        NodeID effect_node = graph.add_node(NodeType::OR);
        graph.add_edge(effect_node, precondition_node);

        for (int effect : op.effects) {
            graph.add_edge(variable_node_ids[effect], effect_node);
        }
    }

    initial_node_id = graph.add_node(NodeType::AND);
    for (int proposition : relaxed_task.initial_state) {
        graph.add_edge(initial_node_id, variable_node_ids[proposition]);
    }

    goal_node_id = graph.add_node(NodeType::AND);
    for (int goal : relaxed_task.goal) {
        graph.add_edge(goal_node_id, variable_node_ids[goal]);
    }
}

void RelaxedTaskGraph::change_initial_state(const GlobalState &global_state) {
    // Remove all initial edges that where introduced for relaxed_task.initial_state.
    for (PropositionID id : relaxed_task.initial_state) {
        graph.remove_edge(variable_node_ids[id], initial_node_id);
    }

    // Switch initial state of relaxed_task
    relaxed_task.initial_state = relaxed_task.translate_state(global_state);

    // Add all initial edges for relaxed_task.initial_state.
    for (PropositionID id : relaxed_task.initial_state) {
        graph.add_edge(variable_node_ids[id], initial_node_id);
    }
}

bool RelaxedTaskGraph::is_goal_relaxed_reachable() {
    graph.most_conservative_valuation();
    
    const auto &goal_node = graph.get_node(goal_node_id);

    return goal_node.forced_true;
}

int RelaxedTaskGraph::additive_cost_of_goal() {
    // Compute the weighted most conservative valuation of the graph and use it
    // to return the h^add value of the goal node.

    // TODO: add your code for exercise 2 (c) here.
    return -1;
}

int RelaxedTaskGraph::ff_cost_of_goal() {
    // TODO: add your code for exercise 2 (e) here.
    return -1;
}

}
