#ifndef LIDAR_LOCALIZATION_H
#define LIDAR_LOCALIZATION_H

#define NODES_LEN 8192
#define MAX_TABLE_X 3000
#define MAX_TABLE_Y 2000
#define BEACON_DETECT_RANGE 100

#include <iostream>
#include <cstdint>
#include <cmath>
#include <list>
#include <chrono>
#include <ctime>
#include <rplidar.h>

using namespace std;

class Localization {
private:
    int x_robot;
    int y_robot;
    int alpha_robot;
    int robotPositionGap;
    pair<int, int> ennemyPosition;
    int ennemyPositionGap;
    uint64_t lastUpdate;
    pair<int, int> beaconsPositions[3];
    int beaconsRadius;
public:
    pair<double, double> robotToCartesian(sl_lidar_response_measurement_node_hq_t node, int x_robot, int y_robot, int alpha_robot);
    bool isInsideMap(pair<double, double> position);
    pair<int, int> averagePosition(list<pair<double, double>> positions);
    int getMaxGap(list<pair<double, double>> positionList, pair<int, int> referencePosition);
    int getBeaconNumber(pair<double, double> position);
    pair<int, int> getCircleCenter(list<pair<double, double>> detectedPositions, int radius);
    void processPoints(sl_lidar_response_measurement_node_hq_t[NODES_LEN], int count);
};


#endif //LIDAR_LOCALIZATION_H