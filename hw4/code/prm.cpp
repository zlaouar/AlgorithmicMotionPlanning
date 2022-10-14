#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <random>
#include <cstdarg>
#include "prm.h"
#include "Graph.h"
#include "time.h"
#include "potentials.h"
//#include "cobstacle.h"
#include "astar.h"


using namespace std;

Prm::Prm(int arg1, double arg2, std::vector<int> arg3, std::vector<int> arg4, Point arg5, Point arg6){
    n = arg1;
    r = arg2;
    x_bound = arg3;
    y_bound = arg4;
    qstart = arg5;
    qgoal = arg6;
}
void Prm::retrieveObstacles(int num_obstacles,...){
    /*
    This function retrieves all obstacles from user and calculates grid height and width
    */
    vector<double> xvec,yvec;
    vector<Point> obstacle;
    xvec.push_back(qstart.x);
    xvec.push_back(qgoal.x);
    yvec.push_back(qstart.y);
    yvec.push_back(qgoal.y);
    va_list arg_list;
    va_start(arg_list,num_obstacles);
    for(int i=0; i<num_obstacles; i++){
        obstacle = va_arg(arg_list, vector<Point>);
        obstacles.push_back(obstacle);
        // Insert x and y terms into vectors
        midpoint.push_back({0,0});
        for(int j=0; j<obstacle.size(); j++){
            xvec.push_back(obstacle[j].x);
            yvec.push_back(obstacle[j].y);
            midpoint[i].x+=obstacle[j].x;
            midpoint[i].y+=obstacle[j].y;
        }
        
        midpoint[i].x = midpoint[i].x/obstacle.size();
        midpoint[i].y = midpoint[i].y/obstacle.size();
    }
    //cout << "done" << endl;
}
void Prm::createPrm(){
    // Initialize Vars
    double rand_x, rand_y;
    //Graph graph(graphEdge[], n, N);

    // (1) Put start node into vertex set V
    prm_verts.push_back(qstart);

    // Randomly sample n configurations from c-space -> put into vector
    /* initialize random seed: */
    random_device rd1;
    //random_device rd2;
    default_random_engine gen1(rd1());//rd1()time(nullptr)); //Standard mersenne_twister_engine seeded with rd()
    //default_random_engine gen2(rd2()); //Standard mersenne_twister_engine seeded with rd()
    uniform_real_distribution<double> dis_x(x_bound[0], x_bound[1]);
    uniform_real_distribution<double> dis_y(y_bound[0], y_bound[1]);
    //cout << "collision: " << midpoint[0].x << "," << midpoint[0].y << "|"<< valid(3.5,0.5) <<endl;
    for(int i=0; i<n; i++){
        /* generate random x and y config: */
        rand_x = dis_x(gen1);
        rand_y = dis_y(gen1);

        // Put sample in prm vertex set
        if(valid(rand_x,rand_y)){
            prm_verts.push_back({rand_x,rand_y});
        }
        
    }
    // Add goal node to vert list
    prm_verts.push_back(qgoal);
    //cout << "vert size: " << prm_verts.size() << endl;
    
    //return;
    // Connect samples
    //vector<Point> neigbors;
    double weight,heuristic;
    for(int i=0; i<prm_verts.size(); i++){
        for(int j=0; j<prm_verts.size(); j++){
            weight = dist(prm_verts[i],prm_verts[j]);
            if(weight<r && i!=j){ // pick neigbors only
                // check for collision
                //cout << "_______________________________________________" << endl;
                //cout << "Vert1: " << i << ", Vert2: " << j << ", weight: "<<weight << ", collision: " << endl;//checkCollision({prm_verts[i],prm_verts[j]})<<endl;
                if(!checkCollision({prm_verts[i],prm_verts[j]})){ // Add to graph
                    heuristic = dist(prm_verts[j],qgoal);
                    edges.push_back({i,j,weight,heuristic}); // add edge connecting the samples
                }
            }
        }
    }    

}
vector<graphNode> Prm::path_smooth(vector<graphNode> path_to_smooth, int num_its){
    random_device rd;
    vector<graphNode> smooth_path = path_to_smooth;
    vector<Point>::iterator it;
    double dist_temp;
    smooth_path_length = 0;
    int min_ind;
    //random_device rd2;
    default_random_engine gen(rd());//rd1()time(nullptr)); //Standard mersenne_twister_engine seeded with rd()
    //default_random_engine gen2(rd2()); //Standard mersenne_twister_engine seeded with rd()
    uniform_int_distribution<int> dis(0,smooth_path.size()-1);
    int ind1,ind2;
    //for(int k=0; k<smooth_path.size(); k++){
    //    cout << smooth_path[k].vertex << ", ";
    //}
    //cout << endl;
    for(int i=0; i<num_its; i++){
        dis.param(uniform_int_distribution<int>::param_type(0,smooth_path.size()-1));
        ind1 = dis(gen);
        ind2 = dis(gen);
        min_ind = min(ind1,ind2);
        if(!checkCollision({prm_verts[smooth_path[ind1].vertex],prm_verts[smooth_path[ind2].vertex]})){
            //cout << "inds: " << ind1 << ", " << ind2 << "| verts: " << smooth_path[ind1].vertex << ", " << smooth_path[ind2].vertex << endl;
            for(int j=0; j<abs(ind2-ind1)-1; j++){
                //cout << "erase: " << smooth_path[min_ind+1].vertex <<endl;
                smooth_path.erase(smooth_path.begin() + (min_ind+1));
                //for(int k=0; k<smooth_path.size(); k++){
                //    cout << smooth_path[k].vertex << ", ";
                //}
                //cout << "Size: " << smooth_path.size()<< endl;
            }
        }
    }
    for(int i=0; i<smooth_path.size()-1; i++){
        dist_temp = dist(prm_verts[smooth_path[i].vertex],prm_verts[smooth_path[i+1].vertex]);
        smooth_path_length += dist_temp;
    }
    return smooth_path;
}
bool Prm::valid(double x, double y){
    //vector<Point> obstacle;
    //int count = 0;
    bool collision;
    bool hit = false;
    int count = 0;
    vector<Point> obstacle;
    line l1,l2;
    for(int i=0; i<midpoint.size(); i++){
        //cout << "midpoint: " << center_x << "," << center_y << endl;
        l1.p1 = midpoint[i];
        l1.p2 = {x,y};
        for(int i=0; i<obstacles.size(); i++){
            obstacle = obstacles[i];
            obstacle.push_back(obstacle[0]); // Append the first obstacle vertex to the end of the vector
            
            for(int j=0; j<obstacle.size()-1; j++){
                l2.p1 = obstacle[j];
                l2.p2 = obstacle[j+1];
                //cout << "Check Intersect: " << l1.p1.x << ","<< l1.p1.y << "->" <<
                //l1.p2.x << ","<< l1.p2.y << "|" << l2.p1.x << ","<< l2.p1.y << "->" <<
                //l2.p2.x << ","<< l2.p2.y <<endl;
                collision = intersect(l1,l2); // check if lines intersect
                //cout << "collision: " << collision <<endl;
                if(collision){
                    hit = true;
                    //count++;
                    //cout << "HELLO" << endl;
                    //break; // Collision not detected
                }
            }
            //cout << "Collision: " << colli
        }
        if(!hit){
            return false;
        }
        hit = false;
    }  
    
    return true;
}
bool Prm::checkCollision(vector<Point> edge){
    /*
    This function checks for collision between the robot and any obstacles by detecting 
    intersections between robot primitives and obstacle primitives
    */
    bool collision;
    vector<Point> obstacle;
    line l1,l2;
    l1.p1 = edge[0];
    l1.p2 = edge[1];
    for(int i=0; i<obstacles.size(); i++){
        obstacle = obstacles[i];
        obstacle.push_back(obstacle[0]); // Append the first obstacle vertex to the end of the vector
        
        for(int j=0; j<obstacle.size()-1; j++){
            l2.p1 = obstacle[j];
            l2.p2 = obstacle[j+1];
            //cout << "Check Intersect: " << l1.p1.x << ","<< l1.p1.y << "->" <<
            //l1.p2.x << ","<< l1.p2.y << "|" << l2.p1.x << ","<< l2.p1.y << "->" <<
            //l2.p2.x << ","<< l2.p2.y <<endl;
            collision = intersect(l1,l2); // check if lines intersect
            
            if(collision){
                return true; // Collision detected
            }
        }
        //cout << "Collision: " << colli
        
    }
    //cout << "_______________________________________________" << endl;
    return false; // No collision detected
}
double Prm::dist(Point q1, Point q2){
    /*
    This function computes the euclidean distance between two points in configuration space
    */

    double dist = sqrt(pow(q1.y-q2.y,2) + pow(q1.x-q2.x,2));
    return dist;
}

bool Prm::intersect(line l1, line l2){
    /*
    This function detects intersection between two lines
    */
    int dir1 = orientation(l1.p1, l1.p2, l2.p1);
    int dir2 = orientation(l1.p1, l1.p2, l2.p2);
    int dir3 = orientation(l2.p1, l2.p2, l1.p1);
    int dir4 = orientation(l2.p1, l2.p2, l1.p2);
    if(dir1 != dir2 && dir3 != dir4){
        //cout << "intersecting: 1" << endl;
        return true; //they are intersecting
        
    }
    if(dir1==0 && collinear(l1, l2.p1)){ //when p1 of line2 are on the line1
        //cout << "intersecting: 2" << endl;
        return true;
    }
    if(dir2==0 && collinear(l1, l2.p2)){ //when p2 of line2 are on the line1
        //cout << "intersecting: 3" << endl;
        return true;
    }
    if(dir3==0 && collinear(l2, l1.p1)){ //when p1 of line1 are on the line2
        //cout << "intersecting: 4" << endl;
        return true;
    }
    if(dir4==0 && collinear(l2, l1.p2)){ //when p2 of line1 are on the line2
        //cout << "intersecting: 5" << endl;
        return true;
    }
    //cout << "intersecting: False" << endl;
    return false;
    
}

int Prm::orientation(Point a, Point b, Point c){
    /*
    This function determines the orientation for three different points
    */
    double dir_val = ((b.y-a.y)*(c.x-b.x))-((b.x-a.x)*(c.y-b.y));
    //cout << "dir Val: " << dir_val << endl;
    if (dir_val == 0){
        return 0;     //collinear
    }
    else if(dir_val < 0){
        return 2;    //anti-clockwise direction 
    }
    return 1; //clockwise direction
}

bool Prm::collinear(line c, Point p){
    /*
    This function checks if point p is on a line c
    */
    if(p.x <= max(c.p1.x, c.p2.x) && p.x >= min(c.p1.x, c.p2.x) && (p.y <= max(c.p1.y, c.p2.y) && p.y >= min(c.p1.y, c.p2.y))){
        return true;
    }
    return false;
}