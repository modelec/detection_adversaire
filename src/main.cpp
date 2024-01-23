#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <rplidar.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define X_LIDAR 1.f
#define Y_LIDAR 1.f
#define ALPHA_LIDAR 0.f

bool stop_signal_received;
void stop_loop(int) {
    stop_signal_received = true;
}

double edge_distance(double x, double y, double alpha) {
    if (M_PI / 2.f > alpha && alpha >= 0.f) {
        double border_right_x_distance = 3.f - x;
        double border_right_y_distance = x * sin(alpha) / cos(alpha);
        double border_top_x_distance = (2.f - y) * cos(alpha) / sin(alpha);
        double border_top_y_distance = 2.f - y;
        double distance_top = sqrt(pow(border_top_x_distance, 2) + pow(border_top_y_distance, 2));
        double distance_right = sqrt(pow(border_right_x_distance, 2) + pow(border_right_y_distance, 2));
        return std::min(distance_top, distance_right);
    } else if (M_PI > alpha && alpha >= M_PI / 2.f) {
        double border_top_x_distance = (2.f - y) * cos(alpha) / sin(alpha);
        double border_top_y_distance = 2.f - y;
        double border_left_x_distance = x;
        double border_left_y_distance = (3.f - x) * sin(alpha) / cos(alpha);
        double distance_top = sqrt(pow(border_top_x_distance, 2) + pow(border_top_y_distance, 2));
        double distance_left = sqrt(pow(border_left_x_distance, 2) + pow(border_left_y_distance, 2));
        return std::min(distance_top, distance_left);
    } else if (M_PI * 3.f / 2.f > alpha && alpha >= M_PI) {
        double border_left_x_distance = x;
        double border_left_y_distance = (3.f - x) * sin(alpha) / cos(alpha);
        double border_bottom_x_distance = y * cos(alpha) / sin(alpha);
        double border_bottom_y_distance = y;
        double distance_left = sqrt(pow(border_left_x_distance, 2) + pow(border_left_y_distance, 2));
        double distance_bottom = sqrt(pow(border_bottom_x_distance, 2) + pow(border_bottom_y_distance, 2));
        return std::min(distance_left, distance_bottom);
    } else if (2.f * M_PI > alpha && alpha >= M_PI * 3.f / 2.f) {
        double border_bottom_x_distance = y * cos(alpha) / sin(alpha);
        double border_bottom_y_distance = y;
        double border_right_x_distance = 3.f - x;
        double border_right_y_distance = x * sin(alpha) / cos(alpha);
        double distance_bottom = sqrt(pow(border_bottom_x_distance, 2) + pow(border_bottom_y_distance, 2));
        double distance_right = sqrt(pow(border_right_x_distance, 2) + pow(border_right_y_distance, 2));
        return std::min(distance_bottom, distance_right);
    }
    return 100.f;
}

bool is_inside(double x_lidar, double y_lidar, double alpha_lidar, double distance, double angle){
    double alpha = angle / 360.f * 2.f * M_PI + alpha_lidar;
    if(alpha >= (2 * M_PI)){
        alpha = alpha - 2.f * M_PI;
    }
    else if(alpha < 0){
        alpha = alpha + 2.f * M_PI;
    }
    double wall_distance = edge_distance(x_lidar, y_lidar, alpha);
    return (distance < wall_distance);
}

using namespace sl;

int main(int argc, const char * argv[]) {
    Result<IChannel*> channel = createSerialPortChannel("/dev/ttyUSB0", 115200);
    ILidarDriver * drv = *createLidarDriver();
    auto res = drv->connect(*channel);
    if(SL_IS_OK(res)){
        drv->startScan(0,1);
        sl_result op_result;
        signal(SIGINT, stop_loop);
        while(true) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t   count = _countof(nodes);
            op_result = drv->grabScanDataHq(nodes, count);
            if (SL_IS_OK(op_result)) {
                drv->ascendScanData(nodes, count);
                for (int pos = 0; pos < (int)count ; ++pos) {
		    if(nodes[pos].dist_mm_q2/4.0f !=0 && nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0){
                    	if(is_inside(X_LIDAR, Y_LIDAR, ALPHA_LIDAR, nodes[pos].dist_mm_q2/4000.0f, ((nodes[pos].angle_z_q14 * 90.f) / 16384.f))){
                            printf("Detected inside echo at angle: %03.2f Dist: %08.0fmm\n",
                                (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                                nodes[pos].dist_mm_q2/4.0f
			    );
                    	}
		    }
                }
            }
            if (stop_signal_received){
                break;
            }
        }
        drv->stop();
        drv->setMotorSpeed(0);
        drv->disconnect();
    }
    delete *channel;
    delete drv;

    drv = nullptr;

    return 0;
}
