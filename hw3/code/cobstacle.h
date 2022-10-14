#ifndef COBSTACLE_H
#define COBSTACLE_H

#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>


class Cobstacle{
    
    public:
        // Problem 5: Generating cspace obstacles
        struct VertAng{std::vector<double> vertex; double angle;};
        std::vector<std::vector<double>> getCobstacleTrans(std::vector<std::vector<double>> obstacle, std::vector<std::vector<double>> robot);
        std::vector<std::vector<double>> getCobstacleRot(std::vector<std::vector<double>> obstacle, std::vector<std::vector<double>> robot, double rot_size);
        std::vector<std::vector<double>> sortVertices(std::vector<std::vector<double>> polygon);
        static bool compTwoAngles(const VertAng& a,const VertAng& b);
        std::vector<std::vector<double>> rotate(std::vector<std::vector<double>> robot, double angle);

        // Problem 8: Generating cspace map for 2-link planar robot
        struct Point{double x, y;};
        struct line{Point p1, p2;};
        std::vector<std::vector<int>> genCpaceMap(int num, ...);
        bool checkCollision(int link_length, std::vector<std::vector<double>> obstacle, std::vector<double> config);
        bool intersect(line l1, line l2);
        int orientation(Point a, Point b, Point c);
        bool collinear(line c, Point p);
};
#endif