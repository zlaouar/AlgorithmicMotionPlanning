#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <random>
#include <cstdarg>
#include "rrt.h"
#include "Graph.h"
#include "time.h"
#include "potentials.h"
//#include "cobstacle.h"
#include "astar.h"


using namespace std;

RRT::RRT(int arg1, double arg2, double arg3, double arg4, std::vector<int> arg5, std::vector<int> arg6, Point arg7, Point arg8){
    n = arg1;
    r = arg2;
    e = arg3;
    pgoal = arg4;
    x_bound = arg5;
    y_bound = arg6;
    qstart = arg7;
    qgoal = arg8;
}
void RRT::retrieveObstacles(int num_obstacles,...){
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
void RRT::createRRT(){
    // Initialize Vars
    int vnear;
    int num_its = 0;
    double rand_x, rand_y, p;
    double min_dist;
    Point qnear, qrand, qdiff, qdiff_short;
    Point qnew = qstart;
    double weight,heuristic;
    valid_path = false;
    // (1) Put start node into vertex set V
    rrt_verts.push_back(qstart);
    nodes.push_back({0,-1});

    // Randomly sample n configurations from c-space -> put into vector
    /* initialize random seed: */
    random_device rd1;
    //random_device rd2;
    default_random_engine gen1(rd1());//rd1()time(nullptr)); //Standard mersenne_twister_engine seeded with rd()
    //default_random_engine gen2(rd2()); //Standard mersenne_twister_engine seeded with rd()
    uniform_real_distribution<double> dis_x(x_bound[0], x_bound[1]);
    uniform_real_distribution<double> dis_y(y_bound[0], y_bound[1]);
    uniform_real_distribution<double> dis_p(0,1);
    //cout << "collision: " << midpoint[0].x << "," << midpoint[0].y << "|"<< valid(3.5,0.5) <<endl;
    
    // While no solution found
    while(num_its != n){
        /* generate random x and y config: */
        p = dis_p(gen1);
        if(p>pgoal){
            rand_x = dis_x(gen1);
            rand_y = dis_y(gen1);
            qrand = {rand_x,rand_y};
        }
        else{
            qrand = qgoal;
        }
        
        // Find nearest node to qrand
        min_dist = 100;
        qnear = rrt_verts[0];
        vnear = 0;
        for(int i=1; i<rrt_verts.size(); i++){
            if(dist(qrand,rrt_verts[i])<min_dist){
                min_dist = dist(qrand,rrt_verts[i]);
                qnear = rrt_verts[i];
                vnear = i; 
            }
        }
        //cout << "i: " << vnear  << ", size: " << rrt_verts.size() << endl;
        /*cout << "____________________________________" << endl;
        cout << "Verts: ";
        for(int i=0; i<rrt_verts.size(); i++){
            cout << rrt_verts[i].x << ", " << rrt_verts[i].y << " | ";
        }
        cout << endl;
        cout << "qrand: " << qrand.x << ", " << qrand.y << " | qnear: " << qnear.x << ", " << qnear.y <<endl;
        */
        // extend branch from qnear to qrand if collision free
        qdiff.x = qrand.x-qnear.x;
        qdiff.y = qrand.y-qnear.y;
        qdiff_short.x = ((qdiff.x/sqrt(pow(qdiff.x,2)+pow(qdiff.y,2)))*r)+qnear.x;
        qdiff_short.y = ((qdiff.y/sqrt(pow(qdiff.x,2)+pow(qdiff.y,2)))*r)+qnear.y;
        //weight = dist(qnear,qrand);
        if(!checkCollision({qnear,qdiff_short})){ // Add to tree
            qnew = qdiff_short;
            rrt_verts.push_back(qnew);
            nodes.push_back({nodes.back().vertex+1,vnear});
            edges.push_back({vnear,nodes.back().vertex,r,0}); // add edge connecting the samples
        }
        if(dist(qnew,qgoal)<e){
            //cout<< "Done" << endl;
            valid_path = true;
            break;
        }
        //cout << num_its <<endl;
        num_its++;
    }
    //cout << "hello" << endl;
    treeNode it = nodes.back();
    
    while(it.parent!=-1){
        //cout << it.vertex << ", ";
        path.insert(path.begin(),it.vertex);
        it = nodes[it.parent];
    }
    path.insert(path.begin(),0);
}

bool RRT::checkCollision(vector<Point> edge){
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
double RRT::dist(Point q1, Point q2){
    /*
    This function computes the euclidean distance between two points in configuration space
    */

    double dist = sqrt(pow(q1.y-q2.y,2) + pow(q1.x-q2.x,2));
    return dist;
}

bool RRT::intersect(line l1, line l2){
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

int RRT::orientation(Point a, Point b, Point c){
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

bool RRT::collinear(line c, Point p){
    /*
    This function checks if point p is on a line c
    */
    if(p.x <= max(c.p1.x, c.p2.x) && p.x >= min(c.p1.x, c.p2.x) && (p.y <= max(c.p1.y, c.p2.y) && p.y >= min(c.p1.y, c.p2.y))){
        return true;
    }
    return false;
}