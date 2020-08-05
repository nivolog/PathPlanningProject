//
// Created by vlad on 04.08.2020.
//

#ifndef PATHPLANNING_OPEN_H
#define PATHPLANNING_OPEN_H


#include "set"
#include "unordered_set"
#include "node.h"
#include <iostream>

struct comparator{
    using is_transparent = int;

    bool operator()(const Node& m, const Node& n) const{
        if (m.F < n.F){
            return true;
        }else if(m.F > n.F){
            return false;
        }else if(m.g < n.g){
            return true;
        }else if(m.g > n.g){
            return false;
        }else if(m.x < n.x){
            return true;
        }else if(m.x > n.x){
            return false;
        }else return m.y < n.y; //getCoordinates().second
    }
};

class Open {
private:
    std::set <Node, comparator> openContainer;
    std::unordered_set <int> openID;
public:
    void insertNode(Node &node);
    void insertId(int id);
    void erase_node(std::set <Node>::iterator it);
    std::set<Node>::iterator findNode(Node &node);
    Node getNode(std::set<Node>::iterator it);
    Node pop();


    void log(std::string typeLog);
    int size();
    std::set<Node>::iterator end();
    void clear();
};



#endif //PATHPLANNING_OPEN_H
