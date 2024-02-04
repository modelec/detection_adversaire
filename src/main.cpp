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

#define X_ROBOT 1000.f
#define Y_ROBOT 1000.f
#define ALPHA_ROBOT 0.f
#define MAX_X 3000.f
#define MAX_Y 2000.f

bool stop_signal_received;
void stop_loop(int) {
    stop_signal_received = true;
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
		    double alpha_lidar = 2.f * M_PI - nodes[pos].angle_z_q14 * 0.5f * M_PI / 16384.f;
                    double alpha = ALPHA_ROBOT + alpha_lidar;
                    if(alpha > 2.f * M_PI){
                        alpha = alpha - 2.f * M_PI;
                    }
                    else if(alpha < 0){
                        alpha = alpha + 2.f * M_PI;
                    }
                    double x_detected = X_ROBOT + nodes[pos].dist_mm_q2/4.0f * cos(alpha);
                    double y_detected = Y_ROBOT + nodes[pos].dist_mm_q2/4.0f * sin(alpha);
                    if(x_detected < MAX_X && x_detected > 0.f && y_detected < MAX_Y && y_detected > 0.f && nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0){
                        printf("Detected inside : x : %04.2f y : %04.2f\n", x_detected, y_detected);
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
