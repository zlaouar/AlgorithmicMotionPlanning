#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>

struct adjNode{
    int vertex;
    double weight, heuristic;
    double total = 0;
    adjNode* next;
    adjNode* back;
    /*
    adjNode operator==(const adjNode& rhs){
        if(vertex == rhs.vertex && weight == )
        return 
    }*/
};
struct graphNode{
    int vertex;
    double weight, total;
    graphNode* parent;
};

struct graphEdge{
    int start_vert, end_vert;
    double weight, heuristic;
};

class Graph{
    private:
        graphNode temp;
        adjNode* adjListNode(int vertex, int parent, double weight, double heuristic, adjNode* head){
            adjNode* newNode = new adjNode;
            newNode->vertex = vertex;
            newNode->weight = weight;
            newNode->heuristic = heuristic;
            newNode->next = head;
            /*if(vertex == nodes.size()){
                temp.vertex = vertex;
                temp.weight = weight;
                temp.parent = nullptr;//&(nodes[parent]);
                nodes.push_back(temp);
                //std::cout << "sizeNodes: "<< nodes.size() <<", vertex: " << temp.vertex << ", parent: " << temp.parent->vertex <<std::endl;
            }*/
            
            return newNode;
        }
        int N; // number of nodes in graph
    public:
        std::vector<graphNode> nodes;
        

        // Create array of pointers for adjacency list
        adjNode **head;

        // Constructor   
        void createGraph(std::vector<graphEdge> edges, int n, int N){
            // create head node
            head = new adjNode * [N]();
            this->N = N;
            //nodes.push_back({0,0,nullptr});
            // Initialize head pointer for all vertices
            for(int i=0; i<N; i++){
                head[i] = nullptr;
                nodes.push_back({i,0,0,nullptr});
            }

            // Create directed graph by extending edges to adjacent vertices
            for(int i=0; i<n; i++){
                int start_vert = edges[i].start_vert;
                int end_vert = edges[i].end_vert;
                double weight = edges[i].weight;
                double heuristic = edges[i].heuristic;

                // Create new node
                adjNode* newNode = adjListNode(end_vert, start_vert, weight, heuristic, head[start_vert]);

                head[start_vert] = newNode;
            }
        }
        void deleteGraph(){
            //std::cout << "what"  << head[0]->vertex<< std::endl;
            for(int i=0; i< N; i++){
                delete[] head[i];
                //delete[] head;
            }
            
            nodes.clear();
        }
        // Destructor
        ~Graph(){
            for(int i=0; i< N; i++){
                delete[] head[i]; 
                //delete[] head;
            }
        }

};
#endif