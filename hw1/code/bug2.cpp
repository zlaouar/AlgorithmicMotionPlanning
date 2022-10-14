//#include "Bugs.h"

#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>


using namespace std;

vector<vector<int>> generateMap(vector<vector<int>> obstacles, int w, int h, int scale_factor, int lowestx, int lowesty, int buffer){
    int const height = h;
    int const width = w;
    vector<vector<int>> map(h+1, vector<int> (w+1, 0));
    int xlow;
    int xhigh;
    int ylow;
    int yhigh;
    vector<vector<int>> WO(2, vector<int> (4, 0));
    for(int i=0; i<obstacles.size()+1; i++){
        if((i % 4 == 0 && i >= 3)){
            WO[0][0] = obstacles[i-4][0];
            WO[0][1] = obstacles[i-3][0];
            WO[0][2] = obstacles[i-2][0];
            WO[0][3] = obstacles[i-1][0];
            WO[1][0] = obstacles[i-4][1];
            WO[1][1] = obstacles[i-3][1];
            WO[1][2] = obstacles[i-2][1];
            WO[1][3] = obstacles[i-1][1];
            xlow = WO[0][0];
            xhigh = WO[0][0];
            ylow = WO[1][0];
            yhigh = WO[1][0];

            for(int m=0; m < WO[0].size(); m++){
                if(WO[0][m] < xlow){
                    xlow = WO[0][m];
                }
                else if(WO[0][m] > xhigh){
                    xhigh = WO[0][m];
                }

                if(WO[1][m] < ylow){
                    ylow = WO[1][m];
                }
                else if(WO[1][m] > yhigh){
                    yhigh = WO[1][m];
                }
            }
            cout << "xlow: "  << xlow << " xhigh: "  << xhigh << " ylow: "  << ylow << " yhigh: "  << yhigh << endl;
            
            for(int n = xlow; n < xhigh+1; n++){
                for(int p = ylow; p < yhigh+1; p++){
                    //cout << height - (p +lowesty) << endl;
                    map[height - (p +abs(lowesty) + buffer)][n+abs(lowestx)+buffer] = 1;
                }
            }
        }
    }
    return map;
}

vector<vector<int>> motionPlan(vector<vector<int>> obstacles, int* start, int* goal, int scale_factor){
    int buffer = 2*scale_factor;
    vector<int> qstart{start[0]*scale_factor,start[1]*scale_factor};
    vector<int> qgoal{goal[0]*scale_factor,goal[1]*scale_factor};
    vector<int> qrobot(2);
    
    int lowestx = 0;
    int highestx = 0;
    int lowesty = 0;
    int highesty = 0;
    cout << obstacles.size() << endl;
    for(int i=0; i < obstacles.size(); i++){
        obstacles[i][0] = obstacles[i][0]*scale_factor;
        obstacles[i][1] = obstacles[i][1]*scale_factor;

        if(obstacles[i][0] > highestx){
            highestx = obstacles[i][0];
        }
        else if(obstacles[i][0] < lowestx){
            lowestx = obstacles[i][0];
        }

        if(obstacles[i][1] > highesty){
            highesty = obstacles[i][1];
        }
        else if(obstacles[i][1] < lowesty){
            lowesty = obstacles[i][1];
        }
    }
    if(qgoal[0] < lowestx){
        lowestx = qgoal[0];
    }
    else if(qgoal[0] > highestx){
        highestx = qgoal[0];
    }
    if(qgoal[1] < lowesty){
        lowesty = qgoal[1];
    }
    else if(qgoal[1] > highesty){
        highesty = qgoal[1];
    }
    

    int const width = highestx - lowestx + 2*buffer;
    int const height = highesty - lowesty + 2*buffer;
    cout << "width: "<< width << " height: "<< height << endl;
    cout << "lowestx: "<< lowestx << " highestx: "<< highestx << " lowesty: "<< lowesty <<" highesty: " << highesty <<endl;
    qstart[0] = qstart[0] + abs(lowestx) + buffer;
    qstart[1] = qstart[1] + abs(lowesty) + buffer;
    qgoal[0] = qgoal[0] + abs(lowestx) + buffer;
    qgoal[1] = qgoal[1] + abs(lowesty) + buffer;
    cout << "qstart: " << qstart[0] << "," << qstart[1]<< "qgoal: " << qgoal[0] << "," << qgoal[1]<< endl;
    //return 0;
    vector<vector<int>> map = {generateMap(obstacles, width, height, scale_factor, lowestx, lowesty, buffer)};
    //return 0;
    map[height - qstart[1]][qstart[0]] = 1;
    map[height - qgoal[1]][qgoal[0]] = 1;
    for(int i=0; i<(height+1); i++){
        for(int j=0; j<(width+1); j++){
            cout << map[i][j];
        }
        cout << endl;
    }
    //return 0;
    // --------------------------BUG 2 Algorithm ---------------------------------------------------------------
    vector<int> qL = qstart;
    vector<int> qH;
    vector<vector<int>> qpath;
    int direction;
    string wall_dir;
    qrobot = qstart;
    int xdist;
    int ydist;
    bool switch_dir = true;
    int index = 1;
    // Find m line trajectory
    vector<vector<int>> mline;
    vector<int> mline_pos;
    mline_pos = qstart;
    mline.push_back(mline_pos);
    while(mline_pos!=qgoal){
        xdist = qgoal[0] - mline_pos[0];
        ydist = qgoal[1] - mline_pos[1];

        if(xdist != 0 && ydist != 0){
            if(switch_dir){ // move one step to the goal in x direction
                mline_pos[0] = mline_pos[0] + (xdist/abs(xdist)); 
                switch_dir = false;
            }
            else{ // move one step to the goal in y direction
                mline_pos[1] = mline_pos[1] + (ydist/abs(ydist));
                switch_dir = true;
            }
        }
        else if(xdist == 0 && ydist != 0){ // move one step to the goal in y direction 
            mline_pos[1]= mline_pos[1] + (ydist/abs(ydist));
        }
        else if(xdist != 0 && ydist == 0){ // move one step to the goal in y direction 
            mline_pos[0] = mline_pos[0] + (xdist/abs(xdist));
        }
        mline.push_back(mline_pos); // add mline position to mline trajectory
    }

    //for(int i=0; i<mline.size(); i++){
    //    cout << "mline: " << mline[i][0] << "," << mline[i][1] << endl;
    //}
    //return 0;
    //cout << qL[1] << endl;
    while(1){ // run until goal found
        while(1){ // motion to goal along m-line
            
            xdist = mline[index][0] - qrobot[0];
            ydist = mline[index][1] - qrobot[1];
            cout << "xdist: " << xdist << "ydist: " << ydist << endl;
            if(xdist == 0 && ydist != 0){ // move one step to the goal in y direction 
                direction = int(ydist/abs(ydist));
            }
            else if(xdist != 0 && ydist == 0){ // move one step to the goal in x direction 
                direction = int(xdist/abs(xdist))*2;
            }
            
            qrobot[0] = mline[index][0];
            qrobot[1] = mline[index][1];
            index++;
            //cout << map[height-qrobot[0]][qrobot[1]] << endl;
            if(map[height-qrobot[1]][qrobot[0]] != 1 && qrobot!=qgoal){ // robot didn't hit obstacle or goal
                qpath.push_back(qrobot); // add position to path
                cout << "qrobot: " << qrobot[0] << "," << qrobot[1] << endl;
            }
            else{ // robot hit obstacle or goal
                cout << "direction: " << direction << " switch_dir: " << switch_dir << endl;
                break;
            }
        }
        if(qrobot==qgoal){ //robot is at goal
            break;
        }
        
        //--------Obstacle follow--------//
        
        qrobot = qpath.back(); // revert robot position to last position
        qH = qrobot; // Define Hit point on the obstacle
        qL = qrobot; // Define Leave point on the obstacle to be updated
        double prev_dist = sqrt(pow(qgoal[0]-qL[0],2) + pow(qgoal[1]-qL[1],2)); // init prev dist to goal
        double new_dist; // init new dist to goal
        //double temp; //updates the prev dist
        //vector<int> last_pos;
        bool mline_reached = false;
        mline_pos = qrobot;
        vector<int> mline_pos_new = mline_pos;
        double milne_dist_prev = sqrt(pow(qgoal[0]-mline_pos[0],2) + pow(qgoal[1]-mline_pos[1],2));
        
        while(1){
            //figure out which direction is left on the obstacle
            //last_pos = qpath.back();
            int top = map[height-qrobot[1]-1][qrobot[0]];
            int right = map[height-qrobot[1]][qrobot[0]+1];
            int bottom = map[height-qrobot[1]+1][qrobot[0]];
            int left = map[height-qrobot[1]][qrobot[0]-1];

            if(direction == 2 && top == 0){
                qrobot[1]= qrobot[1] + 1; // move up
                wall_dir = "up";
            }
            else if(direction == 1 && left == 0){
                qrobot[0]= qrobot[0] - 1; // move left
                wall_dir = "left";
            }
            else if(direction == -1 && right == 0){
                qrobot[0]= qrobot[0] + 1; // move right
                wall_dir = "right";
            }
            else if(direction == -2 && bottom == 0){
                qrobot[1]= qrobot[1] - 1; // move down
                wall_dir = "down";
            }
            cout << "wall dir: " << wall_dir << endl;
            while(1){ // As long as robot is traveling along side of obstacle
                cout << "qrobot: " << qrobot[0] << "," << qrobot[1] << endl;
                int top = map[height-qrobot[1]-1][qrobot[0]];
                int right = map[height-qrobot[1]][qrobot[0]+1];
                int bottom = map[height-qrobot[1]+1][qrobot[0]];
                int left = map[height-qrobot[1]][qrobot[0]-1];
                
                if(wall_dir == "up"){
                    //cout << "right: " << right << endl;
                    if(right == 1 && top == 0){ // move up
                        qrobot[1]= qrobot[1] + 1;
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "up";
                    }
                    else if(right == 1 && top == 1){ // move left
                        qrobot[0]= qrobot[0] - 1;
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "left";
                    }
                    else if(right == 0){
                        qrobot[0]= qrobot[0] + 1; // move right
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "right";
                    }
                }
                else if(wall_dir == "down"){
                    if(left == 1 && bottom == 0){ // move down
                        qrobot[1]= qrobot[1] - 1;
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "down";
                    }
                    else if(left == 1 && bottom == 1){ // move right
                        qrobot[0]= qrobot[0] + 1;
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "right";
                    }
                    else if(left == 0){
                        qrobot[0]= qrobot[0] - 1; // move left
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "left";
                    }
                }
                else if(wall_dir == "right"){
                    if(bottom == 1 && right == 0){ // move right
                        qrobot[0]= qrobot[0] + 1;
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "right";
                    }
                    else if(bottom == 1 && right == 1){ // move up
                        qrobot[1]= qrobot[1] + 1;
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "up";
                    }
                    else if(bottom == 0){
                        qrobot[1]= qrobot[1] - 1; // move down
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "down";
                    }
                }
                else if(wall_dir == "left"){
                    if(top == 1 && left == 0){ // move left
                        qrobot[0]= qrobot[0] - 1;
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "left";
                    }
                    else if(top == 1 && left == 1){ // move down
                        qrobot[1]= qrobot[1] - 1;
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "down";
                    }
                    else if(top == 0){
                        qrobot[1]= qrobot[1] + 1; // move up
                        qpath.push_back(qrobot); // add qrobot position to qpath
                        wall_dir = "up";
                        //cout << "i was here" << endl;
                    }
                }
                cout << "wall dir: " << wall_dir << endl;
                // If the distance of the new position to goal is closer, update qL
                double new_dist = sqrt(pow(qgoal[0]-qrobot[0],2) + pow(qgoal[1]-qrobot[1],2));
                //cout << "obstacle hit at: " << qrobot[1] << "," << qrobot[0] << " new dist: " << new_dist << " prev dist: " << prev_dist << endl;
                if(new_dist < prev_dist){// determine what point is the closest point to goal
                    qL = qrobot;
                    prev_dist = new_dist;
                }
                for(int i=0; i<mline.size(); i++){
                    if(qrobot[0] == mline[i][0] && qrobot[1] == mline[i][1]){ // check if robot comes back to hit point
                        // Go to qL
                        //cout << "We made it, qLeave: " << qL[0] << "," << qL[1]<< endl;
                        mline_pos_new[0] = mline[i][0];
                        mline_pos_new[1] = mline[i][1];
                        mline_reached = true;
                        index = i+1; //update index along mline path
                        break;
                    }
                }
                double mline_dist = sqrt(pow(qgoal[0]-mline_pos_new[0],2) + pow(qgoal[1]-mline_pos_new[1],2));
                if(mline_reached && mline_dist < milne_dist_prev){
                    mline_pos = mline_pos_new;
                    qL = mline_pos;
                    cout << "mline reached at: " << mline_pos[0] << "," << mline_pos[1] << endl;
                    break;
                }
            }
            break; // motion to goal
        }
          
    }

    cout << "goal reached at: " << qrobot[0] << "," << qrobot[1] <<endl;
    qpath.push_back(qrobot);
   
    // remove offset from robot path
    for(int i=0; i<qpath.size(); i++){
        qpath[i][0] = qpath[i][0] - buffer - abs(lowestx);
        qpath[i][1] = qpath[i][1] - buffer - abs(lowesty);
    }
    return qpath;
}
int main()
{
    int scale_factor_map1 = 4;
    int start_map1[2] = {0,0};
    int goal_map1[2] = {10,10};

    int scale_factor_map2 = 4;
    int start_map2[2] = {0,0};
    int goal_map2[2] = {35,0};

    vector<vector<int>> obstacles_map1{{1,1},{2,1},{2,5},{1,5},
            {3,4},{4,4},{4,12},{3,12},
            {3,12},{12,12},{12,13},{3,13},
            {12,5},{13,5},{13,13},{12,13},
            {6,5},{12,5},{12,6},{6,6}};
    vector<vector<int>> obstacles_map2{{-6,-6},{25,-6},{25,-5},{-6,-5},
            {-6,5},{30,5},{30,6},{-6,6},
            {-6,-5},{-5,-5},{-5,5},{-6,5},
            {4,-5},{5,-5},{5,1},{4,1},
            {9,0},{10,0},{10,5},{9,5},
            {14,-5},{15,-5},{15,1},{14,1},
            {19,0},{20,0},{20,5},{19,5},
            {24,-5},{25,-5},{25,1},{24,1},
            {29,0},{30,0},{30,5},{29,5}};
    


    vector<vector<int>> qpath_bug2_map1 = motionPlan(obstacles_map1, start_map1, goal_map1, scale_factor_map1);
    vector<vector<int>> qpath_bug2_map2 = motionPlan(obstacles_map2, start_map2, goal_map2, scale_factor_map2);
    
    // Write path trajectory of path 1 to a csv file
    ofstream bug2_map1_file;
    bug2_map1_file.open ("bug2_map1.csv");
    for(int i=0; i<qpath_bug2_map1.size(); i++){
        bug2_map1_file << qpath_bug2_map1[i][0] << "," << qpath_bug2_map1[i][1] <<"\n";
    }
    bug2_map1_file.close();

    // Write path trajectory of path 2 to a csv file
    ofstream bug2_map2_file;
    bug2_map2_file.open ("bug2_map2.csv");
    for(int i=0; i<qpath_bug2_map2.size(); i++){
        bug2_map2_file << qpath_bug2_map2[i][0] << "," << qpath_bug2_map2[i][1] <<"\n";
    }
    bug2_map1_file.close();


    return 0;
}