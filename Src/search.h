#ifndef SEARCH_H
#define SEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <list>
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>
#include "node.h"
#include "open.h"
#include "closed.h"


class Search
{
    public:
        Search();
        ~Search(void);
        SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);

    protected:
        //Parameters
        double hWeight;
        int metricType;
        bool allowDiagonal;
        bool cutCorners;
        bool allowSqueeze;
        bool breakingTies; // todo:check if it works by default
        bool goal_is_not_reached;
        //Included classes
        Closed                          closed;
        Open                            open;
        SearchResult                    sresult; //This will store the search result
        std::list<Node>                 lppath, hppath; //
        Map                             occupancyMap;

        //logs
        std::unordered_map <int, class Node>::iterator address_of_goal_node;



        bool ok();

        double computeCost(Node &one, Node& two);
        double hCalc(Node &cur, Node &goal);
        double gCalc(Node &predecessor, Node &successor);
        double fCalc(double const &g, double const &h);

        std::list<Node> getSuccessors(Node* current, Node &goal, Map &occupancyMap);
        void updateOpen(Node &new_node, Node *Pred);

        void createLPPath(Node &start);
        void createHPPath();

        //CODE HERE to define other members of the class
};
#endif
