#include <csignal>
#include <thread>
#include "localization.h"

#ifndef get_size
#define get_size(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

bool stop_signal_received;

void stop_loop(int) {
    stop_signal_received = true;
}

using namespace std;
using namespace std::this_thread;
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    //TCP socket connection
    int port = 8080;
    if (argc == 2)
    {
        port = std::stoi(argv[1]);
    }
    Localization localizer(-1, -1, -1, "127.0.0.1", port);
    localizer.start();
    //LIDAR connection
    sl::Result<sl::IChannel *> channel = sl::createSerialPortChannel("/dev/ttyUSB0", 115200);
    sl::ILidarDriver *drv = *sl::createLidarDriver();
    auto res = drv->connect(*channel);
    if (SL_IS_OK(res)) {
        //checking LIDAR health status
        sl_result op_result;
        sl_lidar_response_device_health_t healthinfo;
        op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) {
            localizer.setLidarHealth(true);
            localizer.sendMessage("strat", "ready", "1");
        }
        signal(SIGINT, stop_loop);
        while(true){
            //stop motor and scan
            drv->stop();
            drv->setMotorSpeed(0);
            //waiting until signal to start
            while(!localizer.isStarted()){
                sleep_for(10ms);
                if (stop_signal_received) {
                    break;
                }
            }
            //start scanning
            drv->startScan(false, true);
            while(localizer.isStarted()) {
                //get scan data
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
            if (stop_signal_received) {
                break;
            }
        }
        localizer.stop();
        drv->stop();
        drv->setMotorSpeed(0);
        drv->disconnect();
    }
    delete *channel;
    delete drv;
    return 0;
}
