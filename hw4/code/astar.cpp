#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <cstdarg>
#include <algorithm>
#include <random>
#include <set>
#include <type_traits>
#include "astar.h"
#include "graph.h"

using namespace std;

Astar::Astar(adjNode **head, vector<graphNode> nodes, int startNode, int goalNode){
    vertPointers = head;
    // Initialize open and closed queues
    //static const headstrt =
    graphNodes = nodes;
    //for(int i=0; i<graphNodes.size(); i++){
    //    cout << graphNodes[i].vertex << ", ";
    //}
    //cout << endl;
    start = startNode;
    goal = goalNode;
    //num_iterations = 0;
    //adjNode* ptr = head[0];
    //while (ptr != nullptr) {
    //    cout << "(" << 0 << ", " << ptr->vertex
    //        << ", " << ptr->weight << ") ";
    //    ptr = ptr->next;
        //cout << i << endl;
    //}
    //cout << "Vertex: " << start.vertex << ", " << start.next->vertex << endl;
    //start.back = nullptr;
    //open.insert(start);// Put start node in open queue
}

vector<graphNode> Astar::astar_search(){
    vector<graphNode> path;
    int endVert;
    bool in_closed;
    bool in_open;
    graphNode* ptr;
    adjNode* searchNode;
    adjNode* temp;
    adjNode tempNode, parent;
    set<adjNode>::iterator it, end, ind;
    vector<graphNode>::iterator vec_it;
    pair<set<adjNode>::iterator,bool> ret_open, ret_closed;
    num_iterations = 0;
    //path.push_back(start);

    // Place start's adjacent nodes into open list
    searchNode = vertPointers[start];
    //cout << "searchNode Vert: " << searchNode->vertex << endl;
    while(searchNode != nullptr){
        it = open.find(*searchNode);
        end = open.end();
        in_closed = closed.find(*searchNode) != closed.end();
        in_open = open.find(*searchNode) != open.end();
        //cout << "Node to add: " << searchNode << ", | " << searchNode->vertex <<endl;
        //cout << "open set: ";
        
        
        //cout << "in closed: " << in_closed << endl;
        //cout << "in open: " << in_open << endl;
        searchNode->total += searchNode->weight;
        if(!in_closed && !in_open && searchNode->vertex!=0){ // If node not in closed or open set
            //cout << "here" << endl;
            ret_open = open.insert(*searchNode); // Insert node in open list
            
            //cout << "Node: " << searchNode->vertex <<endl;
            //cout << "return.second: " << ret.second << endl;
            graphNodes[searchNode->vertex].parent = &(graphNodes[0]);
            graphNodes[searchNode->vertex].weight = searchNode->weight;
            graphNodes[searchNode->vertex].total += graphNodes[searchNode->vertex].weight;
            //searchNode->back = nullptr;// update backpointers
        }
        else{
            //cout << "it: " << &it<< ", open list end: "<< &end << endl;
        }
        //for (it=open.begin(); it!=open.end(); ++it){//<< " | ";//
        //    cout << ' ' << " , " << it->vertex << "-" << it->weight<< "," << it->heuristic<< " | ";
        //}
        //cout<< endl;
        searchNode = searchNode->next;

    }
    //num_iterations++;// Increment for first iteration
    //return path;
    //cout << "open set: ";
    //for (it=open.begin(); it!=open.end(); ++it){
    //    cout << ' ' << &it << " , " << it->vertex << " | ";
    //}
    //return path;
    //cout<< endl;
    if(open.empty()){
        cout << "EMPTY" <<endl;
        return path;
    }

    it = open.begin();
    adjNode n_best = *it;
    while(!open.empty()){
        it = open.begin();
        n_best = *it; // Select Node with lowest cost
        num_iterations++;// Increment for first iteration
        //cout << "nbest vertex: " << n_best.vertex << endl;
        // If goal reached -> exit
        if(n_best.vertex == goal){
            //cout << "Parent Vert: " << n_best.back << endl;
            //cout<< "THATS IT!" <<endl;
            break;
        }
        
        ret_closed = closed.insert(n_best); // Add Node to closed list
        //open_vec.push_back(n_best);
        //parent = open_vec.back();
        //cout << "nbest addr: " << &n_best <<endl;
        //cout << "it addr: " << &(*(ret.first)) << ", inserted?: " << ret.second << ", empty: " << closed.empty()<<endl;
        //cout << endl<< "closed set: ";
        //for (it=closed.begin(); it!=closed.end(); ++it){
        //    cout << ' ' << &(*it) << " , " << it->vertex << " | ";
       //}
        open.erase(it); // Remove Node from open list
        
        //return path;
        // Get adjacent nodes from adjacency list
        //vector<adjNode> adjNodes = getAdjacentNodes(vertPointers[n_best.vertex]);
        // Search adjacent nodes and add nodes to open list if not in closed list
        //searchNode = n_best;
        searchNode  = vertPointers[n_best.vertex];
        //cout << "searchNode Vert1: " << searchNode->vertex << endl;
        //cout << "good" << endl;
        while(searchNode != nullptr){
            in_open = false;
            in_closed = false;
            for (ind=open.begin(); ind!=open.end(); ++ind){
                if((*ind).vertex == searchNode->vertex){
                    in_open = true;
                };
            }
            for (ind=closed.begin(); ind!=closed.end(); ++ind){
                if((*ind).vertex == searchNode->vertex){
                    in_closed = true;
                };
            }
            //in_closed = closed.find(*searchNode) != closed.end();
            //cout << "in closed: " << in_closed << endl;
            //cout << "in open: " << in_open << endl;
            //in_open = open.find(*searchNode) != open.end();
            //searchNode->total += searchNode->weight;
            if(!in_closed && !in_open && searchNode->vertex!=0){ // If node n ot in closed or open set
                //cout << "here" << endl;
                ret_open = open.insert(*searchNode); // Insert node in open list
                //searchNode->total += searchNode->weight;
                //open_vec.push_back(*searchNode);
                graphNodes[searchNode->vertex].parent = &(graphNodes[n_best.vertex]);
                //cout << "Node: " << graphNodes[searchNode->vertex].vertex << ", Parent: " << graphNodes[n_best.vertex].vertex  <<endl;
                graphNodes[searchNode->vertex].weight = searchNode->weight;
                graphNodes[searchNode->vertex].total += graphNodes[n_best.vertex].total + searchNode->weight;
            }
            else if((graphNodes[n_best.vertex].total + searchNode->weight)<graphNodes[searchNode->vertex].total){
                graphNodes[searchNode->vertex].parent = &(graphNodes[n_best.vertex]);
                //cout << "Node: " << graphNodes[searchNode->vertex].vertex << ", Parent: " << graphNodes[n_best.vertex].vertex <<endl;
                graphNodes[searchNode->vertex].weight = searchNode->weight;
                graphNodes[searchNode->vertex].total = graphNodes[n_best.vertex].total + searchNode->weight;
            }
            searchNode = searchNode->next;
        }
        //cout << "open set: ";
        //for (it=open.begin(); it!=open.end(); ++it){
        //    cout << ' ' << it->vertex << " | ";
        //}
        //cout << endl<< "closed set: ";
        //for (it=closed.begin(); it!=closed.end(); ++it){
        //    cout << ' ' << it->vertex << " | ";
        ///}
        //break;
        //cout<< endl;
    }
    // Retrace Path
    
    //cout << endl;
    //cout << "hi" << endl;
    path_length = 0;
    valid_path = 1;
    if(open.empty()){
        valid_path = 0;
        return path;
    }
    //graphNodes[0].parent = nullptr;
    //for (int i = 0; i < graphNodes.size(); i++){
    //    cout << graphNodes[i].vertex << ", ";
    //}
    ptr = &(graphNodes[n_best.vertex]);
    
    //cout << "howdy" << endl;
    //cout <<"POINTER: "<< ptr->vertex<< ", " << endl;
    while(ptr != nullptr){
        //cout << "vertex: " << (*ptr).vertex << endl;
        //cout << ptr->vertex << ", ";
        //vec_it = path.begin();
        path.insert ( path.begin() , *ptr );
        path_length += (*ptr).weight;
        //path.push_back(*ptr);
        //cout << "n_best next: " << n_best.back->vertex << endl;
        //endVert = ptr->parent->vertex;
        ptr = ptr->parent;
    }

    return path;

}
