#include <iostream>
#include <csignal>
#include <cmath>
#include <rplidar.h>
#include <list>

#ifndef get_size
#define get_size(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define X_ROBOT 1000.f
#define Y_ROBOT 1000.f
#define ALPHA_ROBOT 0.f
#define MAX_X 3000.f
#define MAX_Y 2000.f

bool stop_signal_received;
void stop_loop(int) {
    stop_signal_received = true;
}

using namespace std;

int main() {
    sl::Result<sl::IChannel *> channel = sl::createSerialPortChannel("/dev/ttyUSB0", 115200);
    sl::ILidarDriver *drv = *sl::createLidarDriver();
    auto res = drv->connect(*channel);
    if (SL_IS_OK(res)) {
        drv->startScan(false, true);
        sl_result op_result;
        signal(SIGINT, stop_loop);
        while (true) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t count = get_size(nodes);
            op_result = drv->grabScanDataHq(nodes, count);
            if (SL_IS_OK(op_result)) {
                drv->ascendScanData(nodes, count);
                list<pair<double, double>> points_inside;
                for (int pos = 0; pos < (int)count; ++pos) {
                    double alpha_lidar = 2.f * M_PI - (double)nodes[pos].angle_z_q14 * 0.5f * M_PI / 16384.f;
                    double alpha = ALPHA_ROBOT + alpha_lidar;
                    if (alpha > 2.f * M_PI) {
                        alpha = alpha - 2.f * M_PI;
                    } else if (alpha < 0) {
                        alpha = alpha + 2.f * M_PI;
                    }
                    double x_detected = X_ROBOT + (double)nodes[pos].dist_mm_q2 / 4.0f * cos(alpha);
                    double y_detected = Y_ROBOT + (double)nodes[pos].dist_mm_q2 / 4.0f * sin(alpha);
                    if (x_detected < MAX_X && x_detected > 0.f && y_detected < MAX_Y && y_detected > 0.f && nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0) {
                        pair<int, int> position_detected;
                        position_detected = make_pair(x_detected, y_detected);
                        points_inside.emplace_back(position_detected);
                    }
                }
                auto it = points_inside.begin();
                while (it != points_inside.end()){
                    double distance_from_prev;
                    double distance_to_next;
                    if(it == points_inside.begin()){
                        distance_from_prev = sqrt(pow((*prev(points_inside.end())).first - (*it).first, 2) + pow((*prev(points_inside.end())).second - (*it).second, 2));
                        distance_to_next = sqrt(pow((*next(it)).first - (*it).first, 2) + pow((*next(it)).second - (*it).second, 2));
                    } else if(it == prev(points_inside.end())) {
                        distance_from_prev = sqrt(pow((*prev(it)).first - (*it).first, 2) + pow((*prev(it)).second - (*it).second, 2));
                        distance_to_next = sqrt(pow((*points_inside.begin()).first - (*it).first, 2) + pow((*points_inside.begin()).second - (*it).second, 2));
                    } else {
                        distance_from_prev = sqrt(pow((*prev(it)).first - (*it).first, 2) + pow((*prev(it)).second - (*it).second, 2));
                        distance_to_next = sqrt(pow((*next(it)).first - (*it).first, 2) + pow((*next(it)).second - (*it).second, 2));
                    }
                    if (distance_from_prev > 0.1 && distance_to_next > 0.1){
                        cout << "False detection : x : " << (*it).first << " y : " << (*it).second << endl;
                        points_inside.erase(it++);
                    }
                }
                cout << "Detected " << points_inside.size() << " correct points this round" << endl;
                it = points_inside.begin();
                while (it != points_inside.end()){
                    cout << "Correct detection : x : " << (*it).first << " y : " << (*it).second << endl;
                }
            }
            if (stop_signal_received) {
                break;
            }
        }
        drv->stop();
        drv->setMotorSpeed(0);
        drv->disconnect();
    }
    delete *channel;
    delete drv;
    return 0;
}
