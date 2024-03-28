#ifndef LIDAR_LOCALIZATION_H
#define LIDAR_LOCALIZATION_H

#define NODES_LEN 8192
#define MAX_TABLE_X 3000
#define MAX_TABLE_Y 2000
#define BEACON_DETECT_RANGE 100
#define PROXIMITY_ALERT_RANGE 200

#include <iostream>
#include <cstdint>
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
    int robotPositionGap;
    pair<int, int> enemyPosition;
    int enemyPositionGap;
    pair<int, int> beaconsPositions[3];
    int beaconsRadius;
    bool lidarHealth = false;
    bool started = false;
public:
    void setLidarHealth(bool ok);
    [[nodiscard]] bool getLidarHealth() const;
    void setRobotPosition(int x, int y, int alpha);
    [[nodiscard]] vector<int> getRobotPosition() const;
    [[nodiscard]] vector<int> getAvoidance() const;
    [[nodiscard]] bool isStarted() const;
    static pair<double, double> robotToCartesian(sl_lidar_response_measurement_node_hq_t node, int x_robot, int y_robot, int alpha_robot);
    static bool isInsideMap(pair<double, double> position);
    static pair<int, int> averagePosition(const list<pair<double, double>>& positions);
    static int getMaxGap(const list<pair<double, double>>& positionList, pair<int, int> referencePosition);
    int getBeaconNumber(pair<double, double> position);
    static pair<int, int> getCircleCenter(list<pair<double, double>> detectedPositions, int radius);
    void processPoints(sl_lidar_response_measurement_node_hq_t[NODES_LEN], size_t count);
    explicit Localization(const char* ip = "127.0.0.1", int port = 8080) : TCPClient(ip, port) {}
    void handleMessage(const string& message) override;
    void sendMessage(const string& recipient, const string& verb, const string& data);
    void sendProximityAlert(int distance, int theta);
};

#endif //LIDAR_LOCALIZATION_H