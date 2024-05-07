#include <csignal>
#include <thread>
#include "localization.h"

#ifndef get_size
#define get_size(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

bool stop_signal_received = false;

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
    localizer.sendMessage("strat", "ready", "1");
    //LIDAR connection
    sl::Result<sl::IChannel *> channel = sl::createSerialPortChannel("/dev/USB_LIDAR", 115200);
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
            bool alreadyInTriangulationMode = false;
            vector<pair<double, int>> overallNearestBeaconDetectedPointRelative(3, make_pair(-1, -1));
            unsigned int measurementCounter = 0;
            while(localizer.isStarted()) {
                //Detect a first triangulation round
                if(localizer.isTriangulating()){
                    if(!alreadyInTriangulationMode){
                        alreadyInTriangulationMode = true;
                        fill(overallNearestBeaconDetectedPointRelative.begin(), overallNearestBeaconDetectedPointRelative.end(), make_pair(-1, -1));
                        measurementCounter = 0;
                    }
                }
                //Get scan data
                sl_lidar_response_measurement_node_hq_t nodes[8192];
                size_t count = get_size(nodes);
                op_result = drv->grabScanDataHq(nodes, count);
                if (SL_IS_OK(op_result)) {
                    drv->ascendScanData(nodes, count);
                    if(localizer.isTriangulating()){
                        //Triangulation mode
                        measurementCounter++;
                        vector<pair<double, int>> nearestBeaconDetectedPointRelative = localizer.extractBeaconsMeasuredPoints(nodes, count);
                        for(unsigned int i = 0; i<3; i++){
                            if((nearestBeaconDetectedPointRelative[i].first < overallNearestBeaconDetectedPointRelative[i].first && nearestBeaconDetectedPointRelative[i].first != -1) || overallNearestBeaconDetectedPointRelative[i].first == -1){
                                overallNearestBeaconDetectedPointRelative[i] = nearestBeaconDetectedPointRelative[i];
                            }
                        }
                    } else {
                        //Normal operation mode
                        localizer.processPoints(nodes, count);
                    }
                }
                //Detect a last triangulation round
                if(localizer.isTriangulating() && measurementCounter == TRIANGULATION_ROUNDS){
                    localizer.processTriangulation(overallNearestBeaconDetectedPointRelative);
                    alreadyInTriangulationMode = false;
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
