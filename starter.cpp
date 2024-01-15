#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

int main(int argc, const char * argv[]) {
    Result<IChannel*> channel = createSerialPortChannel("/dev/ttyUSB0", 115200);
    ILidarDriver * drv = *createLidarDriver();

    auto res = drv->connect(*channel);

    if(SL_IS_OK(res)){
        drv->startScan(0,1);
        delay(10000);
        drv->stop();
        drv->setMotorSpeed(0);
        delay(10000);
        drv->startScan(0,1);
        delay(10000);
        drv->stop();
        drv->setMotorSpeed(0);
        drv->disconnect();
    }

    delete *channel;
    delete drv;

    drv = nullptr;

    return 0;
}
