//
// Created by vlad on 04.08.2020.
//

#include "open.h"


void Open::insertNode(Node &node) {
    openContainer.insert(node);
}

Node Open::pop() {
    Node tmp = *(openContainer.begin());
    openContainer.erase(openContainer.begin());
    openID.erase(tmp.getId());
    return tmp;
}

Node Open::getNode(std::set<Node>::iterator it) {
    Node tmp = *it;
    openContainer.erase(it);
    return tmp;
}

void Open::insertId(int id) {
    openID.insert(id);
}

int Open::size() {
    return openContainer.size();
}

std::set<Node>::iterator Open::findNode(Node &node) {
    auto open_iterator = openContainer.begin();

    auto is_inside_open = openID.find(node.getId());
    if (is_inside_open != openID.end()) {
        while (((*open_iterator).getId() != node.getId()) &&
               (open_iterator != openContainer.end())) {
            ++open_iterator;
        }
    } else open_iterator = openContainer.end();
    return open_iterator;
}

void Open::erase_node(std::set<Node>::iterator it) {
    openContainer.erase(it);
}

std::set<Node>::iterator Open::end() {
    return openContainer.end();
}

void Open::clear() {
    openContainer.clear();
    openID.clear();
}

void Open::log(std::string typeLog) {
    if (typeLog == "size"){
        std::cout << "Open container size is " << openContainer.size() << std::endl;
        std::cout << "Open-ID container size is " << openID.size() << std::endl;
    }
}