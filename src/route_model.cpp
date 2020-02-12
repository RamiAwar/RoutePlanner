#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // Create RouteModel nodes.
    int counter = 0;
    for (Model::Node node : this->Nodes()) {
        m_Nodes.emplace_back(Node(counter, this, node));
        counter++;
    }
    CreateNodeToRoadHashmap();
}


void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}


RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
  
    Node* closest_node = nullptr;
    float min_distance = std::numeric_limits<int>::max();

    for (int index : node_indices) {
        Node node = parent_model->SNodes()[index];
        float d = node.distance(*this);

        // Make sure we're not returning the node itself!
        if (d != 0.0 && !node.visited) {
            if (min_distance > d) {
                min_distance = d;
                closest_node = &parent_model->SNodes()[index]; // we want the main index, not address of the copy
            }
        }

    }

    return closest_node;
    
}


void RouteModel::Node::FindNeighbors() {
    // Loop over all roads that traverse this node, and check all their nodes, and find the
    // nearest neighbors to this node on all these roads that pass through current node
    for (auto& road : this->parent_model->node_to_road[this->index]) {
        RouteModel::Node* neighbor = FindNeighbor(this->parent_model->Ways()[road->way].nodes);
        if (neighbor != nullptr) this->neighbors.push_back(neighbor);
    }
}


RouteModel::Node& RouteModel::FindClosestNode(float x, float y) {
    
    RouteModel::Node input;
    input.x = x;
    input.y = y;

    float min_dist = std::numeric_limits<float>::max();
    float dist = 0.0;
    int closest_idx = 0;

    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                dist = SNodes()[node_idx].distance(input);
                if (dist < min_dist) {
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }

    return SNodes()[closest_idx];
}