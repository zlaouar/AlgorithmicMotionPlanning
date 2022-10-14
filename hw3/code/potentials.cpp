#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <cstdarg>
#include <algorithm>
#include "potentials.h"

using namespace std;

Potentials::Potentials(Point arg1, Point arg2, double arg3, double arg4, double arg5, double arg6, double arg7, vector<double> arg8){
    qstart = arg1;
    qgoal = arg2;
    alpha = arg3;
    zeta = arg4;
    epsilon = arg5;
    dstar = arg6;
    eta = arg7;
    Qstar = arg8;
    //buffer = 2;
}
Potentials::Potentials(Point arg1, Point arg2){
    qstart = arg1;
    qgoal = arg2;
}
void Potentials::retrieveObstacles(int num_obstacles,...){
    /*
    This function retrieves all obstacles from user and calculates grid height and width
    */
    vector<double> xvec,yvec;
    xvec.push_back(qstart.x);
    xvec.push_back(qgoal.x);
    yvec.push_back(qstart.y);
    yvec.push_back(qgoal.y);
    va_list arg_list;
    va_start(arg_list,num_obstacles);
    for(int i=0; i<num_obstacles; i++){
        vector<Point> obstacle = va_arg(arg_list, vector<Point>);
        obstacles.push_back(obstacle);
        // Insert x and y terms into vectors
        for(int j=0; j<obstacle.size(); j++){
            xvec.push_back(obstacle[j].x);
            yvec.push_back(obstacle[j].y);
            //cout << "xvec: " << obstacle[i].x << xvec.back().y;
        }
    }
    
}
vector<Point> Potentials::gradientDescent(){
    /*
    This function runs a gradient descent motion planning algorithm and returns a path to goal
    */
    cout << "Started: gradientDescent()" << endl;
    //discretizeMap();
    // Variable Initializations
    
    cout << "qstart: " << qstart.x << "," << qstart.y <<endl;
    //qstart.y = qstart.y-1;
    //qgoal.y = qgoal.y-1;
    q = qstart;
    //cout << "q: " << q.x << "," << q.y << endl;
    //cout << "qgoal: " << qgoal.x << "," << qgoal.y << endl;
    vector<Point> path;
    Point qold;
    Point delU = {epsilon,epsilon};
    Point delUatt;
    Point delUrep;
    
    
    // Start algorithm
    path.push_back(q); // Initialize path with start point
    //cout << "norm deLU: " << norm(delU) << "| " << q.x << "," << q.y << endl;
    while(norm(q-qgoal) > epsilon){
        //cout << "AAAAAAAAAAA: " << q.x <<"," << q.y<<endl; 
        delUatt = attrPotential(q);
        //cout << "delUatt: " << delUatt.x << endl;
        delUrep = repPotential(q);
        //cout << "finised rep" << endl;
        delU = delUrep + delUatt; //+ delUrep;
        q = path.back() + delU*-alpha;
        path.push_back(q);
        if(path.size() > 1000) {
            cout << "qend: " << q.x << "," << q.y <<endl;
            break;
        }

        if(norm(q-qgoal) < epsilon){
            cout<< "END" <<endl;
        }
        //cout << "norm deLU: " << norm(delU) << "| " << q.x << "," << q.y << endl;
    }
    //qold = repPotential(q);
    return path;
}
vector<vector<Point>> Potentials::vectorField(){
    //cout << "FIELD" << endl;
    vector<vector<Point>> field(gridH, vector<Point> (gridW, {0,0}));
    //cout << "F" << endl;
    Point delUatt,delUrep,delU;
    Point qgrad;
    
    for(int i=0; i<field.size(); i++){
        for(int j=0; j<field[0].size(); j++){
            qgrad.x = j;
            qgrad.y = gridH -1 -i;
            //cout << "________________________________________" << endl;
            //cout << "qgrad: " << qgrad.x << "," << qgrad.y <<endl;
            delUatt = attrPotential(qgrad); 
            //cout << "delUatt: " << delUatt.x << "," << delUatt.y << endl;
            delUrep = repPotential(qgrad);
           
            //cout << "delUrep: " << delUrep.x << "," << delUrep.y << endl;
            //delU = delUatt*-1;
            delU = delUrep*-1;
            //delU = (delUrep+delUatt)*-1;
            //cout << "delU: " << delU.x << "," << delU.y <<endl; 
            field[i][j].x = delU.x;
            field[i][j].y = delU.y;
        }
    }
    return field;
}


Point Potentials::attrPotential(Point cstate){
    /*
    This function computes the attractive component of the potential gradient
    */
    //cout << "Started: attrPotential()" << endl;
    Point delUatt{0,0};
    //cout << "q: " << q.x << "," << q.y << "| qgoal: " << qgoal.x << "," << qgoal.y << endl; 
    if(dist(cstate,qgoal)<= dstar){ // if not within threshold to goal -> use quadratic potential gradient
        //cout << "AAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHH" <<endl;
        delUatt = (cstate - qgoal)*zeta;
        //cout << "delUatt: " << delUatt.x << "," << delUatt.y <<endl;
    }
    else{ // if within threshold to goal -> use conic potential gradient
        //delUatt = (cstate - qgoal)*zeta;
        delUatt = ((cstate-qgoal)/(dist(cstate,qgoal)))*dstar*zeta;
    }
    return delUatt;
}


Point Potentials::repPotential(Point cspace){
    /*
    This function computes the repulsive component of the potential gradient
    */
    //cout << "Started: repPotential()" << endl;
    Point delUrep{0,0};
    
    //cspace = cspace*scale_factor;
    //cspace.x += abs(left) + buffer;
    //cspace.y += abs(bottom) + buffer;
    //cout << "q.x: " << cspace.x << ", q.y: " << cspace.y << endl;
    cspace.x = round(cspace.x);
    cspace.y = round(cspace.y);
    //cout << map[gridH-cspace.y-1][cspace.x] << endl;
    //cout << "q.x: " << cspace.x << ", q.y: " << cspace.y << endl;
    //cout << "y: " << gridH-cspace.y-1 << ", x: " << cspace.x << endl;
    vector<vector<int>> map = brushFire();
    /*
    for(int i=0; i<map.size(); i++){
        for(int j=0; j<map[0].size(); j++){
            cout << map[i][j];
        }
        cout << endl;
    }*/
    double x = cspace.x;
    double y = gridH-cspace.y-1;
    vector<bool> valid_dirs = validDirections(int(y),int(x));
    int dist = map[gridH-cspace.y-1][cspace.x];
    //cout << "Dist: " << dist << endl;
    int lowest_neigbor = dist;
    //cout << "valid_dirs: " << valid_dirs[0] << "," << valid_dirs[1] << "," << valid_dirs[2] << "," << valid_dirs[3] <<endl;
    
    if(valid_dirs[0]){
        if(map[y-1][x] < lowest_neigbor){ // top
            lowest_neigbor = map[y-1][x];
            delUrep = {0,1};
        }
    }
    if(valid_dirs[1]){
        if(map[y][x+1] < lowest_neigbor){ // right
            lowest_neigbor = map[y][x+1];
            delUrep = {1,0};
        }
    }

    if(valid_dirs[2]){
        if(map[y+1][x] < lowest_neigbor){ // bottom
            lowest_neigbor = map[y+1][x];
            delUrep = {0,-1};
        }
    }

    if(valid_dirs[3]){
        if(map[y][x-1] < lowest_neigbor){ // left
            lowest_neigbor = map[y][x-1];
            delUrep = {-1,0};
        }
    }
    //_______________________________________________________________
    if(valid_dirs[4]){
        if(map[y-1][x-1] < lowest_neigbor){ // top left
            lowest_neigbor = map[y-1][x-1];
            delUrep = {-0.75,0.75};
        }
    }
    if(valid_dirs[5]){
        if(map[y-1][x+1] < lowest_neigbor){ // top right
            lowest_neigbor = map[y-1][x+1];
            delUrep = {0.75,0.75};
        }
    }

    if(valid_dirs[6]){
        if(map[y+1][x+1] < lowest_neigbor){ // bottom right
            lowest_neigbor = map[y+1][x+1];
            delUrep = {0.75,-0.75};
        }
    }

    if(valid_dirs[7]){
        if(map[y+1][x-1] < lowest_neigbor){ // bottom left
            lowest_neigbor = map[y+1][x-1];
            delUrep = {-0.75,-0.75};
        }
    }

    delUrep = delUrep*eta*((1/Qstar[0])-(1/dist))*(1/pow(dist,2));
    
    return delUrep;
}
vector<Point> Potentials::waveFront(){
    /*
    This function runs a gradient descent motion planning algorithm and returns a path to goal
    */
    //cout << "Started: gradientDescent()" << endl;

    // Set goal pixel equal to 2
    cout << "goalx: " << qgoal.x << endl;
    map[gridH-qgoal.y-1][qgoal.x] = 2;
    // Variable Initializations
    //for(int i=0; i<map.size(); i++){
    //    for(int j=0; j<map[0].size(); j++){
    //        cout << map[i][j];
    //    }
    //    cout << endl;
    //}
    
    int num = 2;
    while(map[gridH-qstart.y-1][qstart.x]==0){
        for(int i=0; i<map.size(); i++){
            for(int j=0; j<map[0].size(); j++){
                //cout << "entering borderPixel()" <<endl;
                if(map[i][j] == 0 && borderPixel(i,j,num,false)==true){
                    //cout << "exited borderPixel()" <<endl;
                    map[i][j] = num+1;
                }
                //cout << map[i][j];
            }
        }
        num++;
        
    }
    cout << map[gridH-qstart.y-1][qstart.x] << endl;
    int x = qstart.x;
    int y = gridH-qstart.y-1;
    int xnew,ynew;
    int lowest_neigbor;
    vector<bool> valid_dirs;
    q = qstart;
    vector<Point> path;
    int dist = map[y][x];
    cout << "dist: " << dist << endl;
    path.push_back(q); // Initialize path with start point
    while(map[y][x]!=2){
        valid_dirs = validDirections(int(y),int(x));
        
        //cout << "Dist: " << dist << endl;
        lowest_neigbor = dist;
        //cout << "valid_dirs: " << valid_dirs[0] << "," << valid_dirs[1] << "," << valid_dirs[2] << "," << valid_dirs[3] <<endl;
        
        if(valid_dirs[0]){
            if(map[y-1][x] == dist-1){ // top
                lowest_neigbor = map[y-1][x];
                cout << lowest_neigbor << endl;
                xnew = x;
                ynew = y-1;
            }
        }
        if(valid_dirs[1]){
            if(map[y][x+1] == dist-1){ // right
                lowest_neigbor = map[y][x+1];
                cout << lowest_neigbor << endl;
                xnew = x+1;
                ynew = y;
            }
        }
        if(valid_dirs[2]){
            if(map[y+1][x] == dist-1){ // bottom
                lowest_neigbor = map[y+1][x];
                cout << lowest_neigbor << endl;
                xnew = x;
                ynew = y+1;
            }
        }
        if(valid_dirs[3]){
            if(map[y][x-1] == dist-1){ // left
                lowest_neigbor = map[y][x-1];
                cout << lowest_neigbor << endl;
                xnew = x-1;
                ynew = y;
            }
        }
        path.push_back({xnew,gridH-ynew-1});
        x = xnew;
        y = ynew;
        dist = lowest_neigbor;
        cout << "val: " << lowest_neigbor << endl;
        cout << "xnew: " << xnew << ", ynew: " << ynew << endl;
    }
    //for(int i=0; i<map.size(); i++){
    //    for(int j=0; j<map[0].size(); j++){
    //        cout << map[i][j];
    //    }
    //    cout << endl;
    //}
    for(int j=0; j<path.size(); j++){
        cout << path[j].x << "," << path[j].y << endl;
    }
    cout << "qstart: " << qstart.x << "," << qstart.y <<endl;
    //qstart.y = qstart.y-1;
    //qgoal.y = qgoal.y-1;
    
    //cout << "q: " << q.x << "," << q.y << endl;
    //cout << "qgoal: " << qgoal.x << "," << qgoal.y << endl;
    
    Point qold;
    
    // Start algorithm
    

    return path;
}

void Potentials::discretizeMap(){
    //cout << "Started: discretizeMap()" << endl;
    
    buffer = 2*scale_factor;
    vector<double> xvec,yvec;
    qstart = qstart*scale_factor;
    qgoal = qgoal*scale_factor;
    xvec.push_back(qstart.x);
    xvec.push_back(qgoal.x);
    yvec.push_back(qstart.y);
    yvec.push_back(qgoal.y);
    for(int i=0; i < obstacles.size(); i++){
        for(int j=0; j<obstacles[i].size(); j++){
            obstacles[i][j] = obstacles[i][j]*scale_factor;
            xvec.push_back(obstacles[i][j].x);
            yvec.push_back(obstacles[i][j].y);
        }
    }
    top = *max_element(yvec.begin(),yvec.end());
    bottom = *min_element(yvec.begin(),yvec.end());
    left = *min_element(xvec.begin(),xvec.end());
    right = *max_element(xvec.begin(),xvec.end());
    gridH = top-bottom+(2*buffer);
    cout << "gridH: "<< gridH <<endl;
    gridW = right-left+(2*buffer);
    cout << "gridW: "<< gridW <<endl;
    cout << "bottom: " << bottom << ", left: " << left << ", top: "<< top << ", right: " << right <<endl;
    qstart.x = qstart.x + abs(left) + buffer;
    qstart.y = qstart.y + abs(bottom) + buffer;
    qgoal.x = qgoal.x + abs(left) + buffer;
    qgoal.y = qgoal.y + abs(bottom) + buffer;
    
    cout << "qstart: " << qstart.x << "," << qstart.y <<endl;
    cout << "qgoal: " << qgoal.x << "," << qgoal.y <<endl;
    vector<vector<int>> map_dec(gridH, vector<int> (gridW, 0));
    map = map_dec; // create map of zeroes
    
    double xlow,xhigh,ylow,yhigh;
    Point delUrep;
    Point qnear;

    xvec.clear();
    yvec.clear();

    // Put start and goal on map
    //map[gridH-qstart.y][qstart.x] = 1;
    //map[gridH-qgoal.y][qgoal.x] = 1;
    for(int i=0; i<obstacles.size(); i++){
        
        
        xvec.push_back(obstacles[i][0].x);
        yvec.push_back(obstacles[i][0].y);

        for(int m=0; m < obstacles[i].size(); m++){
            xvec.push_back(obstacles[i][m].x);
            yvec.push_back(obstacles[i][m].y);
        }
        yhigh = *max_element(yvec.begin(),yvec.end());
        ylow = *min_element(yvec.begin(),yvec.end());
        xlow = *min_element(xvec.begin(),xvec.end());
        xhigh = *max_element(xvec.begin(),xvec.end());
        cout << "Buffer: " << buffer <<endl;
        cout << "xlow: " << xlow << ", xhigh: " << xhigh << ", ylow: " << ylow << ", yhigh: " << yhigh <<endl;
        for(int n = xlow; n < xhigh+1; n++){
            for(int p = ylow; p < yhigh+1; p++){
                //cout << height - (p +lowesty) << endl;
                map[gridH - (p + abs(bottom) + buffer)-1][n+abs(left)+buffer] = 1;
            }
        }
        xvec.clear();
        yvec.clear();
    }
    //for(int i=0; i<gridW; i++){
    //    map[gridH-buffer-1][i] = 1;
    //}
    /*
    if(gridW> 2*gridH){
        for(int i=0; i<gridW; i++){
            map[gridH-buffer-11][i] = 1;
        }
    }
    else{
        for(int i=0; i<gridH; i++){
            map[gridH-i-1][0+buffer] = 1;
        }
    }
    for(int i=0; i<map.size(); i++){
        for(int j=0; j<map[0].size(); j++){
            cout << map[i][j];
        }
        cout << endl;
    }
    */
    cout << "___________________________________________" << endl;
    
}
vector<vector<int>> Potentials::brushFire(){
    /*
    This function computes the repulsive potential gradient for a certain obstacle
    */
    //discretizeMap();
    
    bool grid_filled = true;
    int num = 1;
    while(1){ // while there are zeroes remaining in the map
        for(int i=0; i<map.size(); i++){
            for(int j=0; j<map[0].size(); j++){
                //cout << "entering borderPixel()" <<endl;
                if(map[i][j] == 0 && borderPixel(i,j,num,true)==true){
                    //cout << "exited borderPixel()" <<endl;
                    map[i][j] = num+1;
                    grid_filled = false;
                }
                //cout << map[i][j];
            }
        }/*
        for(int i=0; i<map.size(); i++){
            for(int j=0; j<map[0].size(); j++){
                cout << map[i][j];
            }
            cout << endl;
        }*/
        //cout << "___________________________________________" << endl;
        if(grid_filled){break;}
        grid_filled = true;
        num++;
        
    }
    return map;
}
bool Potentials::borderPixel(int i, int j, int num, bool eight_point){
    //cout << "entered borderPixel(): " << i << ","<< j  <<endl;
    
    vector<bool> valid_dirs = validDirections(i,j);
    //cout << "valid_dirs: " << valid_dirs[0] << "," << valid_dirs[1] << "," << valid_dirs[2] << "," << valid_dirs[3] <<endl;
    if(valid_dirs[0]){// top
        if(map[i-1][j] == num){ 
            //cout << "here3" << endl;
            return true;
        }
    }
    if(valid_dirs[1]){// right
        if(map[i][j+1] == num){ 
            //cout << "here2" << endl;
            return true;
        }
    }

    if(valid_dirs[2]){// bottom
        if(map[i+1][j] == num){ 
            //cout << "here1" << endl;
            return true;
        }
    }
    if(valid_dirs[3]){// left
        if(map[i][j-1] == num){ 
            //cout << "here4" << endl;
            return true;
        }
    }
    if(eight_point){
        //________________________________________________
        if(valid_dirs[4]){// top left
            if(map[i-1][j-1] == num){ 
                return true;
            }
        }
        if(valid_dirs[5]){// top right
            if(map[i-1][j+1] == num){ 
                return true;
            }
        }
        if(valid_dirs[6]){// bottom right
            if(map[i+1][j+1] == num){ 
                return true;
            }
        }
        if(valid_dirs[7]){// bottom left
            if(map[i+1][j-1] == num){ 
                return true;
            }
        }
    }
    return false;
}

vector<bool> Potentials::validDirections(int i, int j){
    vector<bool> valid_dirs{false,false,false,false,false,false,false,false};
    //cout << "i: " << i << ", j: " << j << endl;
    if(i!=0){// top
        valid_dirs[0] = true;
    }
    if(j!=gridW-1){// right
        valid_dirs[1] = true;
    }
    if(i!=gridH-1){// bottom
        valid_dirs[2] = true;
    }
    if(j!=0){// left
        valid_dirs[3] = true;
    }
    
    if (i!=0 && j!=0){ // top left
        valid_dirs[4] = true;
    }
    if (i!=0 && j!=gridW-1){ // top right
        valid_dirs[5] = true;
    }
    if (i!=gridH-1 && j!=0){ // bottom right
        valid_dirs[6] = true;
    }
    if (i!=gridH-1 && j!=gridW-1){ // bottom left
        valid_dirs[7] = true;
    }

    return valid_dirs;
}
double Potentials::norm(Point delU){
    return sqrt(pow(delU.x,2) + pow(delU.y,2));
}
double Potentials::dist(Point q, Point qgoal){
    /*
    This function computes the euclidean distance between two points in configuration space
    */
    double dist = sqrt(pow(q.y-qgoal.y,2) + pow(q.x-qgoal.x,2));

    return dist;
}