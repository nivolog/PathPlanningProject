//
// Created by vlad on 04.08.2020.
//

#ifndef PATHPLANNING_CLOSED_H
#define PATHPLANNING_CLOSED_H


#include <unordered_map>
#include "node.h"
#include <iostream>

class Closed {
private:
    std::unordered_map <int, Node> Closed;
public:
    void insertNode(Node &node);
    std::unordered_map <int, Node>::iterator findNode(int id);
    bool isIn(int id);
    void output();
    void clear();
    int size();
};



#endif //PATHPLANNING_CLOSED_H
