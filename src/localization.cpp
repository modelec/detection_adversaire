#include "localization.h"

void Localization::setLidarHealth(bool ok) {
    this->lidarHealth = ok;
}


bool Localization::getLidarHealth() const {
    return this->lidarHealth;
}


void Localization::setRobotPosition(int x, int y, int alpha) {
    this->x_robot = x;
    this->y_robot = y;
    this->alpha_robot = alpha;
}


void Localization::setBeaconsMode(bool state){
    this->beaconsMode = state;
}


vector<int> Localization::getRobotPosition() const {
    vector<int> data;
    data.push_back(this->x_robot);
    data.push_back(this->y_robot);
    data.push_back(this->robotPositionGap);
    return data;
}


bool Localization::getBeaconsMode() const{
    return this->beaconsMode;
}


vector<int> Localization::getAvoidance() const {
    vector<int> data;
    data.push_back(this->enemyPosition.first);
    data.push_back(this->enemyPosition.second);
    data.push_back(this->enemyPositionGap);
    return data;
}


bool Localization::isStarted() const {
    return this->started;
}


pair<double, double>
Localization::robotToCartesian(sl_lidar_response_measurement_node_hq_t node, int x_robot, int y_robot,
                               int alpha_robot) {
    //lidar angle ((double) node.angle_z_q14 * 90.f) / 16384.f is in degrees so we divide by 360 and multiply by 2pi
    //lidar rotation is anti-trigonometric, so we do 2pi * (1 - alpha_degrees)
    //lidar is facing the back of the robot, so we add pi to this angle => 2pi * (0.5 - alpha_degrees)
    double alpha_lidar = 2.f * M_PI * (0.5 - (((double) node.angle_z_q14 * 90.f) / 16384.f) / 360.f);
    //alpha_robot is in hundredths of radians
    double alpha = (double)alpha_robot/100 + alpha_lidar;
    if (alpha > 2.f * M_PI) {
        alpha = alpha - 2.f * M_PI;
    } else if (alpha < 0) {
        alpha = alpha + 2.f * M_PI;
    }
    double x_detected = x_robot + (double) node.dist_mm_q2 / 4.0f * cos(alpha);
    double y_detected = y_robot - (double) node.dist_mm_q2 / 4.0f * sin(alpha);
    pair<double, double> position = make_pair(x_detected, y_detected);
    return position;
}


int Localization::distanceBetween(pair<double, double> pos1, pair<double, double> pos2) {
    return (int) sqrt(pow(pos1.first - pos2.first, 2) + pow(pos1.second - pos2.second, 2));
}


bool Localization::isInsideMap(pair<double, double> pos) {
    //Trigger used here to prevent border false detections due to
    return (pos.first + BORDER_DETECT_TRIGGER < MAX_TABLE_X && pos.first - BORDER_DETECT_TRIGGER > 0.f && pos.second + BORDER_DETECT_TRIGGER < MAX_TABLE_Y && pos.second - BORDER_DETECT_TRIGGER > 0.f);
}


pair<int, int> Localization::getAveragePosition(const list<pair<double, double>> &positions) {
    double total_x = 0;
    double total_y = 0;
    int n = 0;
    for (auto &position: positions) {
        n++;
        total_x += position.first;
        total_y += position.second;
    }
    if (n > 0) {
        int x_average = (int) total_x / n;
        int y_average = (int) total_y / n;
        return make_pair(x_average, y_average);
    } else {
        return make_pair(-1, -1);
    }
}


int Localization::getBeaconNumber(pair<double, double> position) {
    for (int n = 0; n < 3; n++) {
        if (Localization::distanceBetween(this->beaconsPositions[n], position) < BEACON_DETECT_RANGE) {
            return n;
        }
    }
    return -1;
}


int Localization::getMaxGap(const list<pair<double, double>> &positionList, pair<int, int> referencePosition) {
    int maxGap = 0;
    for (auto &iter: positionList) {
        int gap = Localization::distanceBetween(iter, referencePosition);
        if (gap > maxGap) {
            maxGap = gap;
        }
    }
    return maxGap;
}


pair<int, int> Localization::getCircleCenter(list<pair<double, double>> detectedPositions, int radius) {
    // Implementing https://math.stackexchange.com/a/1781546
    list<pair<double, double>> deductedCenters;
    double min_x = (*detectedPositions.begin()).first;
    double max_x = (*detectedPositions.begin()).first;
    double min_y = (*detectedPositions.begin()).second;
    double max_y = (*detectedPositions.begin()).second;
    for (auto iter = detectedPositions.begin(); iter != detectedPositions.end(); iter++) {
        if ((*iter).first < min_x) {
            min_x = (*iter).first;
        }
        if ((*iter).first > max_x) {
            max_x = (*iter).first;
        }
        if ((*iter).second < min_y) {
            min_y = (*iter).second;
        }
        if ((*iter).second > max_y) {
            max_y = (*iter).second;
        }
        auto iterPlus3 = iter;
        for (int i = 0; i < 3; i++) {
            if (next(iterPlus3) == detectedPositions.end()) {
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
        deductedCenters.emplace_back(x0 + b * ya / a, y0 - b * xa / a);
        deductedCenters.emplace_back(x0 - b * ya / a, y0 + b * xa / a);
    }
    double total_x = 0;
    double total_y = 0;
    unsigned int n = 0;
    for (auto &deductedCenter: deductedCenters) {
        if (deductedCenter.first > min_x && deductedCenter.first < max_x && deductedCenter.second > min_y &&
            deductedCenter.second < max_y) {
            n++;
            total_x += deductedCenter.first;
            total_y += deductedCenter.second;
        }
    }
    if (n > 0) {
        return make_pair((int) total_x / n, (int) total_y / n);
    } else {
        return make_pair(-1, -1);
    }
}


vector<list<pair<double, double>>> Localization::getAgglomerates(list<pair<double, double>> &positionsList) {
    vector<list<pair<double, double>>> agglomerated_points;
    list<pair<double, double>> new_list;
    agglomerated_points.push_back(new_list);
    unsigned int i=0;
    auto it = positionsList.begin();
    while (it != positionsList.end()) {
        double distance_from_prev;
        double distance_to_next;
        if (it == positionsList.begin()) {
            distance_from_prev = Localization::distanceBetween(positionsList.back(), *it);
            distance_to_next = Localization::distanceBetween(*next(it), *it);
        } else if (it == prev(positionsList.end())) {
            distance_from_prev = Localization::distanceBetween(*prev(it), *it);
            distance_to_next = Localization::distanceBetween(positionsList.front(), *it);
        } else {
            distance_from_prev = Localization::distanceBetween(*prev(it), *it);
            distance_to_next = Localization::distanceBetween(*next(it), *it);
        }
        if (distance_from_prev > AGGLOMERATES_TRIGGER && distance_to_next > AGGLOMERATES_TRIGGER) {
            //Removing solo points
            //Do not move it++ elsewhere (https://stackoverflow.com/questions/596162/can-you-remove-elements-from-a-stdlist-while-iterating-through-it)
            positionsList.erase(it++);
        } else {
            if(distance_to_next > AGGLOMERATES_TRIGGER){
                list<pair<double, double>> empty_list;
                agglomerated_points.push_back(empty_list);
                i++;
            }
            //Agglomerating points next to each others
            agglomerated_points[i].push_back(*it);
            it++;
        }
    }
    //Checking if last agglomerate and first one are the same and if so merge them
    if(agglomerated_points.size() > 1 && Localization::distanceBetween(positionsList.front(), positionsList.back()) < AGGLOMERATES_TRIGGER){
        list<pair<double, double>> lastAgglomerate = agglomerated_points.back();
        list<pair<double, double>> firstAgglomerate = agglomerated_points.front();
        firstAgglomerate.splice(firstAgglomerate.end(), lastAgglomerate);
        agglomerated_points.erase(agglomerated_points.end());
        agglomerated_points[0] = firstAgglomerate;
    }
    return agglomerated_points;
}


list<pair<double, double>> Localization::getMostProbableAgglomerate(vector<list<pair<double, double>>> &agglomerated_points) {
    pair<int ,int> last_enemy_pos = this->enemyPosition;
    pair<int, int> most_probable_average = Localization::getAveragePosition(agglomerated_points[0]);
    unsigned int most_probable_index = 0;
    int best_distance = Localization::distanceBetween(most_probable_average, last_enemy_pos);
    for(unsigned int i=1; i<agglomerated_points.size(); i++){
        pair<int, int>agglomerate_average = Localization::getAveragePosition(agglomerated_points[i]);
        int distance = Localization::distanceBetween(agglomerate_average, last_enemy_pos);
        if(distance < best_distance){
            best_distance = distance;
            most_probable_average = agglomerate_average;
            most_probable_index = i;
        }
    }
    return agglomerated_points[most_probable_index];
}


void Localization::processPoints(sl_lidar_response_measurement_node_hq_t nodes[NODES_LEN], size_t count) {
    list<pair<double, double>> points_inside;
    list<pair<double, double>> beacons_points[3];
    bool proximityAlert = false;
    for (int pos = 0; pos < count; ++pos) {
        //checking measurement quality
        if(nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0){
            //Checking for direct proximity
            if((double)nodes[pos].dist_mm_q2/4.0f < PROXIMITY_ALERT_RANGE) {
                proximityAlert = true;
                if (this->proximityLastRound) {
                    int angle_radians = (int)((((double) nodes[pos].angle_z_q14 * 90.f) / 16384.f) * 2.f * M_PI / 360.f);
                    int angle_robot_base = angle_radians > 2*M_PI ? angle_radians - M_PI : angle_radians + M_PI;
                    this->sendProximityAlert((int) ((double) nodes[pos].dist_mm_q2 / 4.0f),angle_robot_base);
                }
            }
            //Select points inside map and next to beacons
            pair<int, int> position = Localization::robotToCartesian(nodes[pos], this->x_robot, this->y_robot, this->alpha_robot);
            if (Localization::isInsideMap(position)) {
                points_inside.emplace_back(position);
            } else {
                int beaconNumber = this->getBeaconNumber(position);
                if (beaconNumber != -1) {
                    beacons_points[beaconNumber].emplace_back(position);
                }
            }
        }
    }
    //Get agglomerates without solo points
    vector<list<pair<double, double>>> agglomerated_points = Localization::getAgglomerates(points_inside);
    //Get most probable agglomerate average position
    list<pair<double, double>> ennemyAgglomerate = Localization::getMostProbableAgglomerate(agglomerated_points);
    pair<int,int> averageDetection = Localization::getAveragePosition(ennemyAgglomerate);
    int maxGap = Localization::getMaxGap(ennemyAgglomerate, averageDetection);
    //Measure beacons shift
    int xBeaconsShift = 0;
    int yBeaconsShift = 0;
    /*for (int i = 0; i < 3; i++) {
        pair<int, int> beaconDetectedPosition = Localization::getCircleCenter(beacons_points[i], this->beaconsRadius);
        int xShift = beaconDetectedPosition.first - this->beaconsPositions[i].first;
        int yShift = beaconDetectedPosition.second - this->beaconsPositions[i].second;
        xBeaconsShift = xBeaconsShift + xShift / 3;
        yBeaconsShift = yBeaconsShift + yShift / 3;
    }*/

    //All writes at the same time to prevent inconsistent data
    //TODO : appliquer la même transformation à la position ennemie que celle appliquée à la position du robot
    //this->x_robot = this->x_robot + xBeaconsShift;
    //this->y_robot = this->y_robot + yBeaconsShift;
    this->enemyPosition = averageDetection;
    this->enemyPositionGap = maxGap;
    this->proximityLastRound = proximityAlert;
}


void Localization::sendMessage(const string &recipient, const string &verb, const string &data) {
    this->TCPClient::sendMessage(("lidar;" + recipient + ";" + verb + ";" + data).c_str());
}


void Localization::handleMessage(const std::string &message) {
    vector<string> tokens = split(message, ";");
    string issuer = tokens[0];
    string recipient = tokens[1];
    string verb = tokens[2];
    string data = tokens[3];
    if (contains(recipient, "lidar") || contains(recipient, "all")) {
        if (contains(verb, "ping")) {
            this->sendMessage(issuer, "pong", "");
        }
        if (contains(verb, "get health")) {
            if (this->getLidarHealth()) {
                this->sendMessage(issuer, "set health", "1");
            } else {
                this->sendMessage(issuer, "set health", "0");
            }
        }
        if (contains(verb, "get data")) {
            vector<int> position = this->getRobotPosition();
            vector<int> avoidance = this->getAvoidance();
            this->sendMessage(issuer, "set position", to_string(position[0]) + "," + to_string(position[1]) + "," + to_string(position[2]));
            this->sendMessage(issuer, "set avoidance",to_string(avoidance[0]) + "," + to_string(avoidance[1]) + "," + to_string(avoidance[2]));
        }
        if (contains(verb, "start")) {
            this->started = true;
        }
        if (contains(verb, "stop")) {
            this->started = false;
        }
        if (contains(verb, "set pos")) {
            vector<string> position = split(data, ",");
            this->setRobotPosition(stoi(position[0]), stoi(position[1]), stoi(position[2]));
        }
        if (contains(verb, "set beacon mode")) {
            this->setBeaconsMode(stoi(data));
        }
        if (contains(verb, "set beacon position")) {
            vector<string> position = split(data, ",");
            this->setRobotPosition(stoi(position[0]), stoi(position[1]), stoi(position[2]));
        }
    }
}


void Localization::sendProximityAlert(int distance, int angle) {
    this->sendMessage("all", "stop proximity", to_string(distance) + "," + to_string(angle));
}
