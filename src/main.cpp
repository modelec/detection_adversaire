#include <csignal>
#include "localization.h"

#ifndef get_size
#define get_size(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

bool stop_signal_received;

void stop_loop(int) {
    stop_signal_received = true;
}

using namespace std;

int main() {
    Localization localizer;
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
                localizer.processPoints(nodes, count);
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
