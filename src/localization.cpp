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


vector<int> Localization::getRobotPosition() const {
    vector<int> data;
    data.push_back(this->x_robot);
    data.push_back(this->y_robot);
    data.push_back(this->robotPositionGap);
    return data;
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
    double alpha_lidar = 2.f * M_PI - (double) node.angle_z_q14 * 0.5f * M_PI / 16384.f;
    double alpha = alpha_robot + alpha_lidar;
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
    return (pos.first < MAX_TABLE_X && pos.first > 0.f && pos.second < MAX_TABLE_Y && pos.second > 0.f);
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
    unsigned int i=0;
    auto it = positionsList.begin();
    while (it != positionsList.end()) {
        double distance_from_prev;
        double distance_to_next;
        if (it == positionsList.begin()) {
            distance_from_prev = Localization::distanceBetween(*positionsList.end(), *it);
            distance_to_next = Localization::distanceBetween(*next(it), *it);
        } else if (it == prev(positionsList.end())) {
            distance_from_prev = Localization::distanceBetween(*prev(it), *it);
            distance_to_next = Localization::distanceBetween(*positionsList.begin(), *it);
        } else {
            distance_from_prev = Localization::distanceBetween(*prev(it), *it);
            distance_to_next = Localization::distanceBetween(*next(it), *it);
        }
        if (distance_from_prev > AGGLOMERATES_TRIGGER && distance_to_next > AGGLOMERATES_TRIGGER) {
            //Remove solo points
            positionsList.erase(it);
        } else {
            if(distance_to_next > AGGLOMERATES_TRIGGER){
                i++;
            }
            agglomerated_points[i].push_back(*it);
        }
        it++;
    }
    //TODO : check if last agglomerate and first one are the same and if so merge them
    return agglomerated_points;
}


pair<int, int> Localization::getMostProbableAgglomerateAveragePos(vector<list<pair<double, double>>> &agglomerated_points) {
    pair<int ,int> last_enemy_pos = this->enemyPosition;
    pair<int, int> most_probable_average = Localization::getAveragePosition(agglomerated_points[0]);
    int best_distance = Localization::distanceBetween(most_probable_average, last_enemy_pos);
    for(unsigned int i=1; i<agglomerated_points.size(); i++){
        pair<int, int>agglomerate_average = Localization::getAveragePosition(agglomerated_points[i]);
        int distance = Localization::distanceBetween(agglomerate_average, last_enemy_pos);
        if(distance < best_distance){
            best_distance = distance;
            most_probable_average = agglomerate_average;
        }
    }
    return most_probable_average;
}


void Localization::processPoints(sl_lidar_response_measurement_node_hq_t nodes[NODES_LEN], size_t count) {
    list<pair<double, double>> points_inside;
    list<pair<double, double>> beacons_points[3];
    for (int pos = 0; pos < count; ++pos) {
        //Checking for direct proximity
        if(nodes[pos].dist_mm_q2 < PROXIMITY_ALERT_RANGE){
            double angle_degrees = (double)nodes[pos].angle_z_q14 / 16384.f;
            double obstructed_angles_degrees[3] = OBSTRUCTIONS_DEGREES;
            double precision = (double)OBSTRUCTIONS_WIDTH_DEGREES/2;
            //Removing obstructed angles from proximity alert
            if(angle_degrees < obstructed_angles_degrees[0]-precision && angle_degrees > obstructed_angles_degrees[0]+precision && angle_degrees < obstructed_angles_degrees[1]-precision && angle_degrees > obstructed_angles_degrees[1]+precision && angle_degrees < obstructed_angles_degrees[2]-precision && angle_degrees > obstructed_angles_degrees[2]+precision){
                this->sendProximityAlert((int)nodes[pos].dist_mm_q2, (int)nodes[pos].angle_z_q14 / 16384);
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
    //Get agglomerates without solo points
    vector<list<pair<double, double>>> agglomerated_points = Localization::getAgglomerates(points_inside);
    //Get most probable agglomerate average position
    pair<int, int> averageDetection = Localization::getMostProbableAgglomerateAveragePos(agglomerated_points);
    //Measure beacons shift
    int xBeaconsShift = 0;
    int yBeaconsShift = 0;
    for (int i = 0; i < 3; i++) {
        pair<int, int> beaconDetectedPosition = Localization::getCircleCenter(beacons_points[i], this->beaconsRadius);
        int xShift = beaconDetectedPosition.first - this->beaconsPositions[i].first;
        int yShift = beaconDetectedPosition.second - this->beaconsPositions[i].second;
        xBeaconsShift = xBeaconsShift + xShift / 3;
        yBeaconsShift = yBeaconsShift + yShift / 3;
    }

    int maxGap = Localization::getMaxGap(points_inside, averageDetection);
    //All writes at the same time to prevent inconsistent data
    this->x_robot = this->x_robot + xBeaconsShift;
    this->y_robot = this->y_robot + yBeaconsShift;
    this->enemyPosition = averageDetection;
    this->enemyPositionGap = maxGap;
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
        if (contains(verb, "set position")) {
            vector<string> position = split(data, ",");
            this->setRobotPosition(stoi(position[0]), stoi(position[1]), stoi(position[2]));
        }
    }
}


void Localization::sendProximityAlert(int distance, int angle) {
    this->sendMessage("all", "stop proximity", to_string(distance) + "," + to_string(angle));
}