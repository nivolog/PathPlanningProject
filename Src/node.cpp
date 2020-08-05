//
// Created by vlad on 04.08.2020.
//

#include "node.h"

Node::Node() {
    g = 0;
}

Node::Node(int i, int j, int id) : x(i), y(j), id(id) {}

bool Node::operator==(const Node &rhs) const {
    return x == rhs.x &&
           y == rhs.y;
}

bool Node::operator!=(const Node &rhs) const {
    return !(rhs == *this);
}

void Node::checkG(const double &newG) {
    g = fmin(g, newG);
}

const std::pair<int, int> &Node::getCoordinates() const {
    return std::make_pair(this->x, this->y);
}

double Node::getF() const {
    return F;
}

void Node::setF(double f) {
    F = f;
}

double Node::getG() const {
    return g;
}

void Node::setG(double g) {
    Node::g = g;
}

double Node::getH() const {
    return H;
}

void Node::setH(double h) {
    H = h;
}

int Node::getId() const {
    return id;
}

void Node::setId(int id) {
    Node::id = id;
}

void Node::print() {
    std::cout << "Node " << "x " << "is: " << this->x << std::endl;
    std::cout << "Node " << "y " << "is: " << this->y << std::endl;
    std::cout << "Node " << "id " << "is: " << this->id << std::endl;
    std::cout << "Node " << "f " << "is: " << this->F << std::endl;
    std::cout << "Node " << "g " << "is: " << this->g << std::endl;
    std::cout << "Node " << "h " << "is: " << this->H << std::endl;
    std::cout << "Node " << "parent x " << "is: " << parent->x << std::endl;
    std::cout << "Node " << "parent y " << "is: " << parent->y << std::endl;

}
