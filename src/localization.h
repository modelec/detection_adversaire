#ifndef LIDAR_LOCALIZATION_H
#define LIDAR_LOCALIZATION_H

#define NODES_LEN 8192
#define MAX_TABLE_X 3000 //Attention : mode demi table
#define MAX_TABLE_Y 2000
#define BEACON_DETECT_RANGE 100
#define PROXIMITY_ALERT_RANGE 400
#define BORDER_DETECT_TRIGGER 50
#define AGGLOMERATES_TRIGGER 250
#define BEACONS_RADIUS 50
#define TRIANGULATION_ROUNDS 3
#define POSITION_CORRECT_RANGE 25
#define YELLOW_TEAM_BEACONS_POS {make_pair(3094,72), make_pair(3094,1928), make_pair(-94,1000)} //Attention : mode demi table
#define BLUE_TEAM_BEACONS_POS {make_pair(-94,72), make_pair(-94,1928), make_pair(3094,1000)} //Attention : mode demi table

#define LIDAR_LOG_DEBUG_MODE

#include <iostream>
#include <vector>
#include <list>
#include <cmath>
#include <rplidar.h>
#include <TCPSocket/TCPClient.hpp>
#include <TCPSocket/TCPUtils.hpp>

#ifdef LIDAR_LOG_DEBUG_MODE
#include <chrono>
#endif

using namespace std;
using namespace TCPSocket;

class Localization : public TCPClient{
private:
    int x_robot;
    int y_robot;
    int alpha_robot;
    pair<int, int> enemyPosition = make_pair(-1, -1);
    int enemyPositionGap = -1;
    pair<int, int> beaconsPositions[3];
    bool lidarHealth = false;
    bool started = false;
    bool beaconsMode = true;
    bool proximityLastRound = false;
    bool positionIncorrectLastRound = false;
    bool triangulationMode = false;
public:
    Localization(int x_robot, int y_robot, int alpha_robot, const char* ip = "127.0.0.1", int port = 8080) : TCPClient(ip, port) {
        this->x_robot = x_robot;
        this->y_robot = y_robot;
        this->alpha_robot = alpha_robot;
    };
    void setLidarHealth(bool ok);
    void setRobotPosition(int x, int y, int alpha);
    void setBeaconsMode(bool state);
    void setBeaconsPosition(pair<int, int> positions[3]);
    [[nodiscard]] bool getLidarHealth() const;
    [[nodiscard]] bool getBeaconsMode() const;
    [[nodiscard]] vector<int> getAvoidance() const;
    [[nodiscard]] bool isStarted() const;
    [[nodiscard]] bool isTriangulating() const;
    static pair<double, double> robotToCartesian(sl_lidar_response_measurement_node_hq_t node, int x_robot, int y_robot, int alpha_robot);
    static int distanceBetween(pair<double, double> pos1, pair<double, double> pos2);
    static bool isInsideMap(pair<double, double> position);
    static pair<int, int> getAveragePosition(const list<pair<double, double>> &positions);
    static int getMaxGap(const list<pair<double, double>>& positionList, pair<int, int> referencePosition);
    static vector<list<pair<double, double>>> getAgglomerates(list<pair<double, double>> &positionsList);
    static int rplidarToTrigoRadians(double rplidarDegrees);
    static pair<double, double> lineEquationFromPoints(pair<double, double> p1, pair<double, double> p2);
    static vector<pair<double,double>> intersectionBetweenCircles(pair<double,double> c1, double r1, pair<double,double> c2, double r2);
    static pair<double,double> intersectionBetweenLines(pair<double,double> l1, pair<double,double> l2);
    list<pair<double, double>> getMostProbableAgglomerate(vector<list<pair<double, double>>> &agglomerated_points);
    vector<int> determineRobotPosition(vector<pair<double, int>> beaconsDistanceAndAngleRelative, vector<bool> beaconsDetected);
    vector<pair<double, int>> extractBeaconsMeasuredPoints(sl_lidar_response_measurement_node_hq_t nodes[NODES_LEN], size_t count);
    int getBeaconNumber(pair<double, double> position);
    void processPoints(sl_lidar_response_measurement_node_hq_t[NODES_LEN], size_t count);
    void processTriangulation(const vector<pair<double, int>>& overallNearestBeaconDetectedPointRelative);
    void handleMessage(const string &message) override;
    void sendMessage(const string &recipient, const string &verb, const string &data);
    void sendProximityAlert(int distance, int theta);
};

#endif //LIDAR_LOCALIZATION_H