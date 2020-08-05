#include "search.h"

#define SQRT2 1.4142135623

Search::Search()
{
    allowDiagonal = false;
    cutCorners = false;
    allowSqueeze = false;
    breakingTies = true;
    sresult.pathfound = false;
    goal_is_not_reached = true;
    hWeight = 1;
}

Search::~Search() {}

bool Search::ok() {
    return (goal_is_not_reached && (open.size() != 0));
}

double Search::computeCost(Node &one, Node& two) {
    int dx = one.x - two.x;
    int dy = one.y - two.y;
    return std::abs(dx*dy)>0 ? SQRT2 : 1;
}

double Search::hCalc(Node &cur, Node &goal) {
    switch (metricType){
        case 2://euclidean
            return hWeight * hypot(abs(cur.x - goal.x), abs(cur.y - goal.y));
        case 1://manhattan
            return hWeight* (abs(cur.x - goal.x) + abs(cur.y - goal.y));
        case 3://chebyshev
            return hWeight * fmax(abs(cur.x - goal.x), abs(cur.y - goal.y));
        case 0://diagonal
            return hWeight * (abs(cur.x - goal.x) + abs(cur.y - goal.y) + (SQRT2 - 2) * fmin(abs(cur.x - goal.x), abs(cur.y - goal.y)));
        default://dijkstra
            return 0;
    }
}

double Search::gCalc(Node &predecessor, Node &successor) {
    return  predecessor.getG() + computeCost(predecessor, successor);
}

double Search::fCalc(double const &g, double const &h) {
    return g+h;
}


std::list<Node> Search::getSuccessors(Node *current, Node &goal, Map &occupancyMap) {
    std::cout << "debug point: start successors-1\n";
    int x = (*current).x;
    int y = (*current).y;
    std::cout << x << " " << y << std::endl;
    std::list <Node> successors;
    std::cout << "debug point: start successors-2\n";
    for (int i = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j){
            if ((i+j)%2 == 1){ //for horizontal or vertical move
                if (occupancyMap.CellIsTraversable(y - 1 + j , x - 1 + i)){
                    Node tmp = Node(x-1+i, y-1+j, (y-1+j)*occupancyMap.getMapHeight() + (x-1+i) );
                    if(!closed.isIn(tmp.getId())) {
                        tmp.setH(hCalc(tmp, goal));
                        tmp.setG(gCalc(*current,tmp));
                        tmp.setF(fCalc(tmp.getG(), tmp.getH()));
                        tmp.parent = current;
                        successors.push_back(tmp);
                    }
                }
            }else if ((i*j) != 1){//except node itself
                if(allowDiagonal){//if this flag is false there is no reason to check diagonal nodes
                    if (occupancyMap.CellIsTraversable(y - 1 + j, x - 1 + i)){
                        if(cutCorners){
                            if(allowSqueeze){
                                Node tmp = Node(x-1+i, y-1+j, (y-1+j)*occupancyMap.getMapHeight() + (x-1+i) );
                                if(!closed.isIn(tmp.getId())) {
                                    tmp.setH(hCalc(tmp, goal));
                                    tmp.setG(gCalc(*current,tmp));//maybe here
                                    tmp.setF(fCalc(tmp.getG(), tmp.getH()));
                                    tmp.parent = current;
                                    successors.push_back(tmp);
                                }
                            }else{ //if allowSqueeze = false
                                //remember, that 'i' and 'j' point to diagonal nodes
                                if (occupancyMap.CellIsTraversable(y-(j-(j-1)%2), x - i) &&
                                    occupancyMap.CellIsTraversable(y-j,x - (i-(i-1)%2) )){ //only if both side nodes are accessible we go
                                    Node tmp = Node(x-1+i, y-1+j, (y-1+j)*occupancyMap.getMapHeight() + (x-1+i) );
                                    if(!closed.isIn(tmp.getId())) {
                                        tmp.setH(hCalc(tmp, goal));
                                        tmp.setG(gCalc(*current,tmp));
                                        tmp.setF(fCalc(tmp.getG(), tmp.getH()));
                                        tmp.parent = current;
                                        successors.push_back(tmp);
                                    }
                                }
                            }
                        }else{ //if cutCorners = false
                            if (occupancyMap.CellIsTraversable(y-(j-(j-1)%2), x - i) ||
                                occupancyMap.CellIsTraversable(y-j,x - (i-(i-1)%2) )){ //if one side node is accessible we go
                                Node tmp = Node(x-1+i, y-1+j, (y-1+j)*occupancyMap.getMapHeight() + (x-1+i) );
                                if(!closed.isIn(tmp.getId())) {
                                    tmp.setH(hCalc(tmp, goal));
                                    tmp.setG(gCalc(*current,tmp));
                                    tmp.setF(fCalc(tmp.getG(), tmp.getH()));
                                    tmp.parent = current;
                                    successors.push_back(tmp);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return successors;
}

void Search::updateOpen(Node &new_node, Node *Pred) {
    auto open_iterator = open.findNode(new_node);
    open.insertId(new_node.getId());//Возможно совместить с методом insertNode
    std::cout << "debug point 3\n";
    if (open_iterator != open.end()) {
        Node tmp = open.getNode(open_iterator);
        std::cout << "debug point 4\n";
        if (tmp.parent->getG() > (*Pred).getG()) {
            tmp.parent = Pred;
        }
        tmp.checkG(new_node.getG());
        open.insertNode(tmp);
    } else {
        //remove new
        std::cout << "debug point 5\n";
        new_node.parent = new Node;
        std::cout << "debug point 6\n";
        new_node.parent = Pred;
        std::cout << "debug point 7\n";
        open.insertNode(new_node);
        std::cout << "debug point 8\n";
    }
}

SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    //loading options
    this->metricType = 1;
    this->allowDiagonal = options.allowdiagonal;
    this->cutCorners = options.cutcorners;
    this->allowSqueeze = options.allowsqueeze;
    sresult.numberofsteps = 0;
    std::cout <<options.metrictype << std::endl;
    std::cout << metricType << std::endl;
    Node successor_node;
    std::list <Node> successors;
    Node *pred;
    occupancyMap = map;
    Node start = occupancyMap.getStartNode();
            //Node(occupancyMap.start_i, occupancyMap.start_j, occupancyMap.start_j*occupancyMap.getMapWidth()+occupancyMap.start_i)
    Node goal = occupancyMap.getGoalNode();
            //Node(occupancyMap.goal_i, occupancyMap.goal_j, occupancyMap.goal_j*occupancyMap.getMapWidth()+occupancyMap.goal_i)
    open.insertNode(start);
    std::cout<<"starting planning\n";
    std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
    while (ok()) {

        successor_node = open.pop();

        if (successor_node == goal) {
            sresult.numberofsteps++;
            goal_is_not_reached = false;
            closed.insertNode(successor_node);
            address_of_goal_node = closed.findNode(successor_node.getId());
            continue;
        }



        std::cout << "Currently working on :  " << successor_node.x << "  " << successor_node.y << std::endl;

        open.log("size");

        closed.insertNode(successor_node);

        pred = &closed.findNode(successor_node.getId())->second;                                                       //Maybe better return NODE or *NODE

        successors = getSuccessors(pred, goal, occupancyMap);
        /*
         *         for (auto & successor : successors){
           successor.print();
         * }
         */

        for (auto & successor : successors){
            updateOpen(successor, pred);
        }

        sresult.numberofsteps++;
    }
    createLPPath(start);
    createHPPath();
    std::chrono::time_point<std::chrono::steady_clock> end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> algorithmWorkingTime = end_time - start_time;


    sresult.lppath = &lppath;
    sresult.hppath = &hppath;
    sresult.nodescreated =  open.size() + closed.size();
    sresult.time = algorithmWorkingTime.count();
    //todo:

    //

    return sresult;
}

void Search::createLPPath(Node &start) {
    std::cout << "started creating path" << std::endl;
    if(!goal_is_not_reached){
        Node path_node = (*address_of_goal_node).second;
        Node path_node_next;
        lppath.push_front(path_node);
        bool pathNotComplete = true;


        while(pathNotComplete){
            path_node_next = *(path_node.parent);
            if (path_node_next ==  start){
                pathNotComplete = false;
                lppath.push_front(start);
                sresult.pathfound = true;
                continue;
            }
            else{
                lppath.push_front(path_node_next);
                path_node = path_node_next;
            }
        }
    }
    else{
        std::cout << "ERROR: Goal was not reached during working" << std::endl;
    }
}

void Search::createHPPath() {
    int prev_direction, direction = -1;          //0 - north, 1 - north-east, 2 - east, 3 - south-east, etc.
    int dx, dy;
    //hppath.push_back(*lppath.begin());
    for (auto it = lppath.begin(); it != std::prev(lppath.end(), 2); ++it){

        auto nx = std::next(it, 1);
        dx = (*nx).x - (*it).x;
        dy = (*nx).y - (*it).y;
        if(dx == 1){
            if(dy == 1){
                direction = 3;
            }else if (dy == -1){
                direction = 5;
            }else{
                direction = 4;
            }
        }else if (dx == -1){
            if(dy == 1){
                direction = 1;
            }else if (dy == -1){
                direction = 7;
            }else{
                direction = 0;
            }
        }else{
            if(dy == 1){
                direction = 2;
            }else if (dy == -1){
                direction = 6;
            }else{
                direction = -1;
            }
        }

        if(prev_direction == -1){
            continue;
        }else if(direction == -1){
                std::cout << "Error occurred: could not find hppath!\n";
                break;
        }else{
            if(direction == prev_direction){
                continue;
            }else{
                hppath.push_back(*it);
            }
        }
        prev_direction = direction;

    }
    hppath.push_back(*std::prev(lppath.end(), 1));
}

/*void Search::makePrimaryPath(Node curNode)
{
    //need to implement
}*/

/*void Search::makeSecondaryPath()
{
    //need to implement
}*/
