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
                /* GETTING POINTS INSIDE OF MAP ONLY */
                list<pair<double, double>> points_inside;
                for (int pos = 0; pos < (int) count; ++pos) {
                    double alpha_lidar = 2.f * M_PI - (double) nodes[pos].angle_z_q14 * 0.5f * M_PI / 16384.f;
                    double alpha = ALPHA_ROBOT + alpha_lidar;
                    if (alpha > 2.f * M_PI) {
                        alpha = alpha - 2.f * M_PI;
                    } else if (alpha < 0) {
                        alpha = alpha + 2.f * M_PI;
                    }
                    double x_detected = X_ROBOT + (double) nodes[pos].dist_mm_q2 / 4.0f * cos(alpha);
                    double y_detected = Y_ROBOT + (double) nodes[pos].dist_mm_q2 / 4.0f * sin(alpha);
                    if (x_detected < MAX_X && x_detected > 0.f && y_detected < MAX_Y && y_detected > 0.f &&
                        nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0) {
                        pair<int, int> position_detected;
                        position_detected = make_pair(x_detected, y_detected);
                        points_inside.emplace_back(position_detected);
                    }
                }
                /* FILTERING POINTS */
                auto it = points_inside.begin();
                double min_x = 0;
                double max_x = 0;
                double min_y = 0;
                double max_y = 0;
                while (it != points_inside.end()) {
                    double distance_from_prev;
                    double distance_to_next;
                    if (it == points_inside.begin()) {
                        distance_from_prev = sqrt(pow((*prev(points_inside.end())).first - (*it).first, 2) +
                                                  pow((*prev(points_inside.end())).second - (*it).second, 2));
                        distance_to_next = sqrt(
                                pow((*next(it)).first - (*it).first, 2) + pow((*next(it)).second - (*it).second, 2));
                    } else if (it == prev(points_inside.end())) {
                        distance_from_prev = sqrt(
                                pow((*prev(it)).first - (*it).first, 2) + pow((*prev(it)).second - (*it).second, 2));
                        distance_to_next = sqrt(pow((*points_inside.begin()).first - (*it).first, 2) +
                                                pow((*points_inside.begin()).second - (*it).second, 2));
                    } else {
                        distance_from_prev = sqrt(
                                pow((*prev(it)).first - (*it).first, 2) + pow((*prev(it)).second - (*it).second, 2));
                        distance_to_next = sqrt(
                                pow((*next(it)).first - (*it).first, 2) + pow((*next(it)).second - (*it).second, 2));
                    }
                    if (distance_from_prev > 100.f && distance_to_next > 100.f) {
                        cout << "False detection : x : " << (*it).first << " y : " << (*it).second << endl;
                        points_inside.erase(it++);
                    } else {
                        if (min_x > (*it).first || min_x == 0) {
                            min_x = (*it).first;
                        }
                        if (max_x < (*it).first || max_x == 0) {
                            max_x = (*it).first;
                        }
                        if (min_y > (*it).second || min_y == 0) {
                            min_y = (*it).second;
                        }
                        if (max_y < (*it).second || max_y == 0) {
                            max_y = (*it).second;
                        }
                        it++;
                    }
                }
                cout << "Detected " << points_inside.size() << " correct points this round max_x " << max_x << " max_y "
                     << max_y << " min_x  " << min_x << " min_y " << min_y << endl;
                /* GETTING MEDIATORS CARTESIAN EQUATIONS */
                list<pair<double, double>> mediators;
                for (auto i = points_inside.begin(); i != prev(points_inside.end()); i++) {
                    for (auto j = next(i); j != points_inside.end(); j++) {
                        double x_middle = ((*i).first + (*j).first) / 2.f;
                        double y_middle = ((*i).second + (*j).second) / 2.f;
                        double a = ((*i).first - (*j).first) / ((*j).second - (*i).second);
                        double b = y_middle - a * x_middle;
                        pair<int, int> mediator_equation;
                        mediator_equation = make_pair(a, b);
                        mediators.emplace_back(mediator_equation);
                        cout << "Point iterator : " << distance(points_inside.begin(), i) << ","
                             << distance(points_inside.begin(), j) << " Mediator between x "
                             << (*i).first << " y " << (*i).second << " and x " << (*j).first << " y "
                             << (*j).second << " :  a = " << a << " b = " << b << endl;
                    }
                }
                /* GETTING MEDIATORS INTERSECTIONS COORDINATES */
                list<pair<double, double>> intersections;
                for (auto i = mediators.begin(); i != prev(mediators.end()); i++) {
                    for (auto j = next(i); j != mediators.end(); j++) {
                        pair<double, double> intersection;
                        double x_intersect = ((*i).second - (*j).second) / ((*j).first - (*i).first);
                        intersection = make_pair(x_intersect, (*i).second - (*i).first * x_intersect);
                        intersections.emplace_back(intersection);
                        cout << "Mediator iterator : " << distance(mediators.begin(), i) << ","
                             << distance(mediators.begin(), j) << " Intersection between mediator a = " << (*i).first
                             << " b = " << (*i).second << " and a = " << (*j).first << " b = " << (*j).second
                             << " at coordinates x " << x_intersect << " y " << (*i).second - (*i).first * x_intersect
                             << endl;
                    }
                }
                /* ESTIMATING ENEMY POS */
                double total_x = 0;
                double total_y = 0;
                unsigned int n = 0;
                for (auto iter = intersections.begin(); iter != intersections.end(); iter++) {
                    if ((*iter).first > min_x && (*iter).first < max_x && (*iter).second > min_y &&
                        (*iter).second < max_y) {
                        n++;
                        total_x += (*iter).first;
                        total_y += (*iter).second;
                        cout << "N : " << n << " Intersection x " << (*iter).first << " y " << (*iter).second << endl;
                    }
                }
                if (n > 0) {
                    cout << "Detected enemy position : x : " << total_x / (double) n << " y : " << total_y / (double) n
                         << " from " << n << " points" << endl;
                } else {
                    cout << "Could not precisely detect ennemy pos";
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
