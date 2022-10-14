#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include "cobstacle.h"
#include <cstdarg>
#include <algorithm>
#include <typeinfo>

using namespace std;

#define PI 3.14159265

vector<vector<double>> Cobstacle::getCobstacleTrans(vector<vector<double>> obstacle, vector<vector<double>> robot){
    /*
    This function generates a cspace obstacle by computing the Minkowski difference between a convex robot and obstacle
    */

    // Variable Initializations
    vector<vector<double>> cobstacle;
    vector<double> new_vertex(2);
    vector<double> obstacle_vert(2);
    vector<double> robot_vert(2);
    double obst_angle,robot_angle;

    // Put the robot in its local frame
    for(int i=0; i<robot.size()-1; i++){
        robot[i+1][0] = robot[i+1][0] - robot[0][0];
        robot[i+1][1] = robot[i+1][1] - robot[0][1];
    }
    robot[0][0] = 0;
    robot[0][1] = 0;
    
    
    // Negate the vertices of the robot for the minkowski difference
    for(int i=0; i<robot.size(); i++){
        robot[i][0] = robot[i][0] * (-1);
        robot[i][1] = robot[i][1] * (-1);
    }
    
    // Sort the vertices of the polygons in order of ascending consecutive angle
    robot = sortVertices(robot);
    obstacle = sortVertices(obstacle);
    
    // Append the polygon's first vertex to the end
    obstacle.push_back({obstacle[0][0],obstacle[0][1]});
    robot.push_back({robot[0][0],robot[0][1]});

    // Init vars for algorithm
    int i=0;
    int j=0;
    int n = obstacle.size()-1;
    int m = robot.size()-1;

    // Minkowski difference algorithm
    while(i<n || j<m){
        obstacle_vert = {obstacle[i][0],obstacle[i][1]};
        robot_vert = {robot[j][0],robot[j][1]};
        new_vertex = {obstacle_vert[0]+robot_vert[0],obstacle_vert[1]+robot_vert[1]}; // Add the appropriate vertices from robot and obstacle
        cobstacle.push_back(new_vertex);

        // Compute angle between two vertices for obstacle and robot
        if(i==n){
            obst_angle = 2*PI;
        }
        else{
            obst_angle = atan2(obstacle[i+1][1]-obstacle[i][1],obstacle[i+1][0]-obstacle[i][0]);
        }
        if(j==m){
            robot_angle = 2*PI;
        }
        else{
            robot_angle = atan2(robot[j+1][1]-robot[j][1],robot[j+1][0]-robot[j][0]);
        }

        if(obst_angle < 0.0){
            obst_angle = obst_angle + 2*PI;\
        }
        if(robot_angle < 0.0){
            robot_angle = robot_angle + 2*PI;
        }

        if(obst_angle < robot_angle){
            i++;
        }
        else if(obst_angle > robot_angle){
            j++;
        }
        else{
            i++;
            j++;
        }
        
    }

    return cobstacle;
}

vector<vector<double>> Cobstacle::getCobstacleRot(vector<vector<double>> obstacle, vector<vector<double>> robot, double rot_size){
    /*
    This function generates a cspace obstacle by computing the Minkowski difference between a convex robot and obstacle
    */

    // Variable Initializations
    vector<vector<double>> cobstacle;
    vector<vector<double>> threeD_cobstacle;
    vector<vector<double>> robot_temp;
    double total_angle = 0;
    double num_angles = 0;
    vector<double> new_vertex(2);
    vector<double> obstacle_vert(2);
    vector<double> robot_vert(2);
    double obst_angle,robot_angle;

    // Put the robot in its local frame
    for(int i=0; i<robot.size()-1; i++){
        robot[i+1][0] = robot[i+1][0] - robot[0][0];
        robot[i+1][1] = robot[i+1][1] - robot[0][1];
    }
    robot[0][0] = 0;
    robot[0][1] = 0;
    robot_temp = robot;

    // Sort the vertices of the obstacle in order of ascending consecutive angle
    obstacle = sortVertices(obstacle);
    
    // Append the polygon's first vertex to the end
    obstacle.push_back({obstacle[0][0],obstacle[0][1]});

    // Generate cspace obstacle for every rotation angle of the robot
    while(total_angle < 2*PI){
        // Rotate robot by discretization step size
        robot = robot_temp;
        robot = rotate(robot, total_angle);
        
        // negate the vertices of the robot for the minkowski difference
        for(int i=0; i<robot.size(); i++){
            robot[i][0] = robot[i][0] * (-1);
            robot[i][1] = robot[i][1] * (-1);
        }

        // Sort the vertices of the robot in order of ascending consecutive angle
        robot = sortVertices(robot);

        // Append the polygon's first vertex to the end
        robot.push_back({robot[0][0],robot[0][1]});

        // Init vars for algorithm
        int i=0;
        int j=0;
        int n = obstacle.size()-1;
        int m = robot.size()-1;
        
        // Minkowski difference algorithm
        while(i<n+1 || j<m+1){
            obstacle_vert = {obstacle[i][0],obstacle[i][1]};
            robot_vert = {robot[j][0],robot[j][1]};
             // Add the appropriate vertices from robot and obstacle
            new_vertex = {obstacle_vert[0]+robot_vert[0],obstacle_vert[1]+robot_vert[1]};
            cobstacle.push_back(new_vertex);

            // Compute angle between two vertices for obstacle and robot
            if(i==n){
                obst_angle = 2*PI;
            }
            else{
                obst_angle = atan2(obstacle[i+1][1]-obstacle[i][1],obstacle[i+1][0]-obstacle[i][0]);
            }
            if(j==m){
                robot_angle = 2*PI;
            }
            else{
                robot_angle = atan2(robot[j+1][1]-robot[j][1],robot[j+1][0]-robot[j][0]);
            }

            if(obst_angle < 0.0){
                obst_angle = obst_angle + 2*PI;
            }
            if(robot_angle < 0.0){
                robot_angle = robot_angle + 2*PI;
            }

            if(obst_angle < robot_angle){
                i++;
            }
            else if(obst_angle > robot_angle){
                j++;
            }
            else{
                i++;
                j++;
            }
        }
        
        // Append new obstacle to threeD cspace container
        for(i=0; i<cobstacle.size(); i++){
            threeD_cobstacle.push_back({cobstacle[i][0],cobstacle[i][1]});
        }
        cobstacle.clear(); // Clear the cspace obstacle container for next iteration
        
        // Add rotation angle increment to current rotation angle
        total_angle = total_angle + rot_size;
        num_angles+=1;
    }
    
    // Append the size of the cspace obstacles
    threeD_cobstacle.push_back({num_angles,num_angles});

    return threeD_cobstacle;
}

bool Cobstacle::compTwoAngles(const VertAng& a, const VertAng& b){
    /*
    This function compares two angles and returns true if the second is larger than the first
    */
    return a.angle < b.angle;
}
vector<vector<double>> Cobstacle::sortVertices(vector<vector<double>> polygon){
    /*
    This function sorts the vertices of a polygon in order of ascending angles 
    between consecutive polygon sides
    */
    vector<vector<double>> new_polygon;
    vector<VertAng> obstacle_struct;
    vector<double> angles;
    double angle;
    bool lowest_y = false;
    bool eq_lowest_y = false;
    vector<double> midpoint{0,0};
    
    // Compute midpoint: average of polygon vertices 
    for(int i=0; i<polygon.size(); i++){
        midpoint[0]+=polygon[i][0];
        midpoint[1]+=polygon[i][1];
    }
    midpoint[0] = midpoint[0]/polygon.size();
    midpoint[1] = midpoint[1]/polygon.size();
    
    // Compute angles between all vertices and the midpoint
    for(int i=0; i<polygon.size(); i++){
        angle = atan2(midpoint[1]-polygon[i][1],midpoint[0]-polygon[i][0]);
        if(angle<0.0){
            angle= angle + 2*PI;
        }
        angles.push_back(angle);
        
        VertAng vert_struct;
    
        vert_struct.vertex = {polygon[i][0],polygon[i][1]};
        vert_struct.angle = angles[i];
        obstacle_struct.push_back(vert_struct);
    }
    
    // Sort the vertices in order of ascending angle w.r.t. midpoint
    sort(obstacle_struct.begin(),obstacle_struct.end(),compTwoAngles); // sort the angles between consecutive edges of the polygon in ascending order
    
    vector<vector<double>> new_verts;
    for(int i=0; i<obstacle_struct.size(); i++){
        new_verts.push_back(obstacle_struct[i].vertex);
    }
    new_verts.push_back(obstacle_struct[0].vertex);
    for(int i=0; i<obstacle_struct.size(); i++){
        angle = atan2(new_verts[i+1][1]-new_verts[i][1],new_verts[i+1][0]-new_verts[i][0]);
        if(angle<0.0){
            angle= angle + 2*PI;
        }
        obstacle_struct[i].angle = angle;
    }
    // Sort the vertices of a polygon in order of ascending angles between consecutive polygon sides
    sort(obstacle_struct.begin(),obstacle_struct.end(),compTwoAngles);

    // Populate new polygon with sorted vertices
    for(int i=0; i<obstacle_struct.size(); i++){
        new_polygon.push_back(obstacle_struct[i].vertex);
    }
    
    return new_polygon;
}   


vector<vector<double>> Cobstacle::rotate(vector<vector<double>> robot, double angle){
    /*
    This function rotates a polygon about the origin a certain angle
    */
    vector<double> robot_temp;

    for(int i=0; i<robot.size(); i++){
        robot_temp = {robot[i][0],robot[i][1]};
        robot[i][0] = (robot_temp[0] * cos(angle)) - (robot_temp[1] * sin(angle));
        robot[i][1] = (robot_temp[0] * sin(angle)) + (robot_temp[1] * cos(angle));
    }
    return robot;

}

vector<vector<int>> Cobstacle::genCpaceMap(int num_args, ...){
    /*
    This function generates a cspace map of a 2-link planar robot with varying obstacles
    */

    // Variable Initializations
    va_list arg_list;
    va_start(arg_list,num_args);
    const int grid_disc = va_arg(arg_list, int);
    int link_length = va_arg(arg_list, int);
    bool collision;
    double theta_step = (2*PI)/grid_disc;
    double theta1,theta2;
    theta1 = 0;
    theta2 = 0;

    vector<double> config{theta1,theta2}; // Configuration q of 2 link robot arm
    // Build cspace map based on collisions between the robot arm and the obstacles
    vector<vector<int>> c_grid(grid_disc, vector<int> (grid_disc, 0));
    for(int k=0; k<num_args-2; k++){
        // Retrieve next obstacle from input list
        vector<vector<double>> obstacle = va_arg(arg_list, vector<vector<double>>);
        for(int i=0; i<c_grid.size(); i++){
            for(int j=0; j<c_grid.size(); j++){
                config = {theta1,theta2}; // robot cspace configuration
                
                // Check if the robot collides with obstacles for current config
                collision = checkCollision(link_length,obstacle,config); // Check for collisions between robot and obstacle
                
                if(collision){
                    c_grid[c_grid.size()-1-j][i] = 1; // Set grid cell value to 1 if collision is detected at configuration q
                }
                theta2+=theta_step;
                
            }
            theta1+=theta_step;
            theta2 = 0;
        }
        
        theta1 = 0;
        theta2 = 0;
    }
    va_end(arg_list); //clean memory reserved for valist

    return c_grid;
}
        
        
        
bool Cobstacle::checkCollision(int len, vector<vector<double>> obstacle, vector<double> config){
    /*
    This function checks for collision between the robot and any obstacles by detecting 
    intersections between robot primitives and obstacle primitives
    */
    bool collision;
    line l1, l2;
    obstacle.push_back({obstacle[0][0],obstacle[0][1]}); // Append the first obstacle vertex to the end of the vector
    for(int i=0; i<obstacle.size()-1; i++){
        for(int j=0; j<config.size(); j++){
            if(j==0){
                l1 = {{0,0},{len*cos(config[0]), len*sin(config[0])}};
                l2 = {{obstacle[i][0],obstacle[i][1]},{obstacle[i+1][0],obstacle[i+1][1]}};
            }
            else if(j==1){
                l1 = {{len*cos(config[0]), len*sin(config[0])},{len*cos(config[0]) +  len*(cos(config[0] + config[1])),len*(sin(config[0]) +  sin(config[0] + config[1]))}};
                l2 = {{obstacle[i][0],obstacle[i][1]},{obstacle[i+1][0],obstacle[i+1][1]}};
            }
            collision = intersect(l1,l2); // check if lines intersect
            
            if(collision){
                return true; // Collision detected
            }
            
        }
    }
    return false; // No collision detected
}

bool Cobstacle::intersect(line l1, line l2){
    /*
    This function detects intersection between two lines
    */
    int dir1 = orientation(l1.p1, l1.p2, l2.p1);
    int dir2 = orientation(l1.p1, l1.p2, l2.p2);
    int dir3 = orientation(l2.p1, l2.p2, l1.p1);
    int dir4 = orientation(l2.p1, l2.p2, l1.p2);
    if(dir1 != dir2 && dir3 != dir4){
        return true; //they are intersecting
    }
    if(dir1==0 && collinear(l1, l2.p1)){ //when p1 of line2 are on the line1
        return true;
    }
    if(dir2==0 && collinear(l1, l2.p2)){ //when p2 of line2 are on the line1
        return true;
    }
    if(dir3==0 && collinear(l2, l1.p1)){ //when p1 of line1 are on the line2
        return true;
    }
    if(dir4==0 && collinear(l2, l1.p2)){ //when p2 of line1 are on the line2
        return true;
    }
    return false;
    
}

int Cobstacle::orientation(Point a, Point b, Point c){
    /*
    This function determines the orientation for three different points
    */
    double dir_val = ((b.y-a.y)*(c.x-b.x))-((b.x-a.x)*(c.y-b.y));
    
    if (dir_val == 0){
        return 0;     //collinear
    }
    else if(dir_val < 0){
        return 2;    //anti-clockwise direction 
    }
    return 1; //clockwise direction
}

bool Cobstacle::collinear(line c, Point p){
    /*
    This function checks if point p is on a line c
    */
    if(p.x <= max(c.p1.x, c.p2.x) && p.x <= min(c.p1.x, c.p2.x) && (p.y <= max(c.p1.y, c.p2.y) && p.y <= min(c.p1.y, c.p2.y))){
        return true;
    }
    return false;
}
