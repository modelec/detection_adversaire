#include "localization.h"


pair<double, double> Localization::robotToCartesian(sl_lidar_response_measurement_node_hq_t node, int x_robot, int y_robot, int alpha_robot){
    double alpha_lidar = 2.f * M_PI - (double) node.angle_z_q14 * 0.5f * M_PI / 16384.f;
    double alpha = alpha_robot + alpha_lidar;
    if (alpha > 2.f * M_PI) {
        alpha = alpha - 2.f * M_PI;
    } else if (alpha < 0) {
        alpha = alpha + 2.f * M_PI;
    }
    double x_detected =  x_robot + (double) node.dist_mm_q2 / 4.0f * cos(alpha);
    double y_detected = y_robot + (double) node.dist_mm_q2 / 4.0f * sin(alpha);
    pair<double, double> position = make_pair(x_detected, y_detected);
    return position;
}


bool Localization::isInsideMap(pair<double, double> pos){
    return (pos.first < MAX_TABLE_X && pos.first > 0.f && pos.second < MAX_TABLE_Y && pos.second > 0.f);
}


pair<int, int> Localization::averagePosition(list<pair<double, double>> positions){
    double total_x = 0;
    double total_y = 0;
    int n = 0;
    for (auto iter = positions.begin(); iter != positions.end(); iter++) {
        n++;
        total_x += (*iter).first;
        total_y += (*iter).second;
    }
    if (n > 0) {
        int x_average = (int) total_x / n;
        int y_average = (int) total_y / n;
        pair<double, double> average = make_pair(x_average, y_average);
        return average;
    } else {
        //TODO : pas de points => pas de moyenne
    }
}


int Localization::getBeaconNumber(pair<double, double> position){
    for(int n=0; n<3; n++){
        if(sqrt(pow(this->beaconsPositions[n].first - position.first, 2) + pow(this->beaconsPositions[n].second - position.second, 2)) < BEACON_DETECT_RANGE){
            return n;
        }
    }
    return -1;
}


int getMaxGap(list<pair<double, double>> positionList, pair<int, int> referencePosition){
    int maxGap = 0;
    for (auto iter = positionList.begin(); iter != positionList.end(); iter++) {
        int gap = sqrt(pow((*iter).first - referencePosition.first, 2) + pow((*iter).second - referencePosition.second, 2));
        if(gap > maxGap){
            maxGap = gap;
        }
    }
    return maxGap;
}


pair<int, int> getCircleCenter(list<pair<double, double>> detectedPositions, int radius){
    // Implementing https://math.stackexchange.com/a/1781546
    list<pair<double, double>> deductedCenters;
    double min_x = (*detectedPositions.begin()).first;
    double max_x = (*detectedPositions.begin()).first;
    double min_y = (*detectedPositions.begin()).second;
    double max_y = (*detectedPositions.begin()).second;
    for (auto iter = detectedPositions.begin(); iter != detectedPositions.end(); iter++) {
        if((*iter).first < min_x){
            min_x = (*iter).first;
        }
        if((*iter).first > max_x){
            max_x = (*iter).first;
        }
        if((*iter).second < min_y){
            min_y = (*iter).second;
        }
        if((*iter).second > max_y){
            max_y = (*iter).second;
        }
        auto iterPlus3 = iter;
        for(int i=0; i<3; i++){
            if(next(iterPlus3) == detectedPositions.end()){
                iterPlus3 = detectedPositions.begin();
            } else {
                iterPlus3 = next(iterPlus3);
            }
        }
        double xa = ((*iter).first - (*iterPlus3).first) / 2.f;
        double ya = ((*iter).second - (*iterPlus3).second) / 2.f;
        double x0 = (*iter).first + xa;
        double y0 = (*iter).second + ya;
        double a = sqrt(pow((*iter).first, 2) + pow((*iter).second, 2));
        double b = sqrt(pow(radius, 2) - pow(a, 2));
        deductedCenters.emplace_back(x0 + b*ya/a, y0 - b*xa/a);
        deductedCenters.emplace_back(x0 - b*ya/a, y0 + b*xa/a);
    }
    double total_x = 0;
    double total_y = 0;
    unsigned int n = 0;
    for (auto & deductedCenter : deductedCenters) {
        if (deductedCenter.first > min_x && deductedCenter.first < max_x && deductedCenter.second > min_y &&deductedCenter.second < max_y) {
            n++;
            total_x += deductedCenter.first;
            total_y += deductedCenter.second;
        }
    }
    if(n>0){
        return make_pair((int)total_x/n, (int)total_y/n);
    } else {
        //TODO : error
    }
}


void Localization::processPoints(sl_lidar_response_measurement_node_hq_t nodes[NODES_LEN], int count){
    list<pair<double, double>> points_inside;
    list<pair<double, double>> beacons_points[3];
    for (int pos = 0; pos < count; ++pos) {
        pair<int, int> position = this->robotToCartesian(nodes[pos], this->x_robot, this->y_robot, this->alpha_robot);
        if(this->isInsideMap(position)){
            points_inside.emplace_back(position);
        } else {
            int beaconNumber = this->getBeaconNumber(position);
            if(beaconNumber != -1){
                beacons_points[beaconNumber].emplace_back(position);
            }
        }
    }
    int xBeaconsShift = 0;
    int yBeaconsShift = 0;
    for(int i=0; i<3; i++){
        pair<int, int>beaconDetectedPosition = this->getCircleCenter(beacons_points[i], this->beaconsRadius);
        int xShift = beaconDetectedPosition.first - this->beaconsPositions[i].first;
        int yShift = beaconDetectedPosition.second - this->beaconsPositions[i].second;
        xBeaconsShift = xBeaconsShift + xShift/3;
        yBeaconsShift = yBeaconsShift + yShift/3;
    }
    //TODO remove false detections
    //TODO check agglomeration of points and keep only the one next to the ennemy previous position => replace points_inside by agglomerate points in next lines
    pair<int, int> averageDetection = this->averagePosition(points_inside);
    int maxGap = this->getMaxGap(points_inside, averageDetection);
    //All writes at the same time to prevent inconsistent data
    this->x_robot = this->x_robot + xBeaconsShift;
    this->y_robot = this->y_robot + yBeaconsShift;
    this->ennemyPosition = averageDetection;
    this->ennemyPositionGap = maxGap;
    this->lastUpdate = (uint64_t)chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
}