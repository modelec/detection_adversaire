#ifndef LIDAR_LOCALIZATION_H
#define LIDAR_LOCALIZATION_H

#define NODES_LEN 8192
#define MAX_TABLE_X 1500
#define MAX_TABLE_Y 2000
#define BEACON_DETECT_RANGE 100
#define PROXIMITY_ALERT_RANGE 250
#define AGGLOMERATES_TRIGGER 100
#define BEACONS_RADIUS 50

#include <iostream>
#include <cmath>
#include <list>
#include <chrono>
#include <ctime>
#include <rplidar.h>
#include <TCPSocket/TCPClient.hpp>
#include <TCPSocket/TCPUtils.hpp>

using namespace std;
using namespace TCPSocket;

class Localization : public TCPClient{
private:
    int x_robot;
    int y_robot;
    int alpha_robot;
    int robotPositionGap = -1;
    pair<int, int> enemyPosition = make_pair(-1, -1);
    int enemyPositionGap = -1;
    pair<int, int> beaconsPositions[3];
    int beaconsRadius = BEACONS_RADIUS;
    bool lidarHealth = false;
    bool started = false;
public:
    Localization(int x_robot, int y_robot, int alpha_robot, const char* ip = "127.0.0.1", int port = 8080) : TCPClient(ip, port) {
        this->x_robot = x_robot;
        this->y_robot = y_robot;
        this->alpha_robot = alpha_robot;
    };
    void setLidarHealth(bool ok);
    void setRobotPosition(int x, int y, int alpha);
    [[nodiscard]] bool getLidarHealth() const;
    [[nodiscard]] vector<int> getRobotPosition() const;
    [[nodiscard]] vector<int> getAvoidance() const;
    [[nodiscard]] bool isStarted() const;
    static pair<double, double> robotToCartesian(sl_lidar_response_measurement_node_hq_t node, int x_robot, int y_robot, int alpha_robot);
    static int distanceBetween(pair<double, double> pos1, pair<double, double> pos2);
    static bool isInsideMap(pair<double, double> position);
    static pair<int, int> getAveragePosition(const list<pair<double, double>> &positions);
    static int getMaxGap(const list<pair<double, double>>& positionList, pair<int, int> referencePosition);
    static pair<int, int> getCircleCenter(list<pair<double, double>> detectedPositions, int radius);
    static vector<list<pair<double, double>>> getAgglomerates(list<pair<double, double>> &positionsList);
    pair<int, int> getMostProbableAgglomerateAveragePos(vector<list<pair<double, double>>> &agglomerated_points);
    int getBeaconNumber(pair<double, double> position);
    void processPoints(sl_lidar_response_measurement_node_hq_t[NODES_LEN], size_t count);
    void handleMessage(const string &message) override;
    void sendMessage(const string &recipient, const string &verb, const string &data);
    void sendProximityAlert(int distance, int theta);
};

#endif //LIDAR_LOCALIZATION_H