//
// Created by vlad on 04.08.2020.
//

#include "closed.h"

void Closed::insertNode(Node &node) {
    Closed.insert(std::make_pair(node.id, node));
}

std::unordered_map <int, Node>::iterator  Closed::findNode(int id) {
    return Closed.find(id);
}

void Closed::output(){
    std::cout << "CLOSED is: ";
    if (Closed.empty()) std::cout << "EMPTY" << std::endl;
    for (auto it : Closed)
    {
        std::cout << "(" << it.second.x << ", "
                  << it.second.y << ") ";
    }
    std::cout << std::endl;
}

bool Closed::isIn(int id) {
    return findNode(id) != Closed.end();
}

void Closed::clear() {
    Closed.clear();
}

int Closed::size() {
    return Closed.size();
}

void Closed::log(std::string typeLog) {
    if("size" == typeLog){
        std::cout << "Closed container size is " << size() << std::endl;
    }
}
