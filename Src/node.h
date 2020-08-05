#ifndef NODE_H
#define NODE_H

//That's the data structure for storing a single search node.
//You MUST store all the intermediate computations occuring during the search
//incapsulated to Nodes (so NO separate arrays of g-values etc.)
#include <cmath>
#include <list>
#include <iostream>

struct Node
{
    int     x, y; //grid cell coordinates
    double  F, g, H; //f-, g- and h-values of the search node
    Node    *parent; //backpointer to the predecessor node (e.g. the node which g-value was used to set the g-velue of the current node)
    int     id; //id of current node
    Node();

    Node(int i, int j, int id);

    bool operator==(const Node &rhs) const;

    bool operator!=(const Node &rhs) const;

    void checkG (double const &newG);

    const std::pair<int, int> &getCoordinates() const;

    double getF() const;

    void setF(double f);

    double getG() const;

    void setG(double g);

    double getH() const;

    void setH(double h);

    int getId() const;

    void setId(int id);

    void print();
};
#endif
