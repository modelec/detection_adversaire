#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

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
                    printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                           (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ",
                           (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                           nodes[pos].dist_mm_q2/4.0f,
                           nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                }
            }
            if (stop_signal_received){
                break;
            }
        }
        drv->stop()
        drv->setMotorSpeed(0);
        drv->disconnect();
    }
    delete *channel;
    delete drv;

    drv = nullptr;

    return 0;
}