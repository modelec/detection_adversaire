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


void Localization::setBeaconsPosition(pair<int, int> positions[3]){
    for(unsigned int i = 0; i < 3; i++){
        this->beaconsPositions[i] = positions[i];
    }
}


void Localization::setBeaconsMode(bool state){
    this->beaconsMode = state;
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


bool Localization::isTriangulating() const {
    return this->triangulationMode;
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

int Localization::rplidarToTrigoRadians(double rplidarDegrees){
    int angle_cent_radians = (int)(rplidarDegrees * 200.f * M_PI / 360.f);
    angle_cent_radians = angle_cent_radians + (int)(100 * M_PI);
    if(angle_cent_radians > 200 * M_PI){
        angle_cent_radians = (int)(angle_cent_radians - 200 * M_PI);
    }
    return (int)(200 * M_PI - angle_cent_radians);
}


pair<double, double> Localization::lineEquationFromPoints(pair<double, double> p1, pair<double, double> p2){
    double a = (p2.second - p1.second) / (p2.first - p1.first);
    double b = p1.second - a * p1.first;
    return make_pair(a, b);
}


vector<pair<double,double>> Localization::intersectionBetweenCircles(pair<double,double> p0, double r0, pair<double,double> p1, double r1){
    //https://paulbourke.net/geometry/circlesphere/
    double a, dx, dy, d, h, rx, ry;
    double x2, y2;
    dx = p1.first - p0.first;
    dy = p1.second - p0.second;
    d = hypot(dx,dy);
    if (d > (r0 + r1))
    {
        return {make_pair(-1, -1), make_pair(-1, -1)};
    }
    if (d < fabs(r0 - r1))
    {
        return {make_pair(-1, -1), make_pair(-1, -1)};
    }
    a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;
    x2 = p0.first + (dx * a/d);
    y2 = p0.second + (dy * a/d);
    h = sqrt((r0*r0) - (a*a));
    rx = -dy * (h/d);
    ry = dx * (h/d);
    return {make_pair(x2 + rx, y2 + ry), make_pair(x2 - rx, y2 - ry)};
}


pair<double,double> Localization::intersectionBetweenLines(pair<double,double> l1, pair<double,double> l2){
    double x_intersect = (l2.second - l1.second) / (l1.first - l2.first);
    double y_intersect = l1.first * x_intersect + l1.second;
    return make_pair(x_intersect, y_intersect);
}


int Localization::determineRobotOrientation(pair<int,int> robotPos, pair<int,int> beaconPos, int beaconOrientation){
    int angle = -1;
    if(beaconPos.first <= robotPos.first && beaconPos.second <= robotPos.second){
        angle = (int)(100*(double)atan(robotPos.first - beaconPos.first));
    } else if(beaconPos.first <= robotPos.first && beaconPos.second >= robotPos.second){
        angle = (int)(100*(M_PI/2 + (double)atan(beaconPos.second - robotPos.second)));
    } else if(beaconPos.first >= robotPos.first && beaconPos.second >= robotPos.second){
        angle = (int)(100*(M_PI+(double)atan(beaconPos.first - robotPos.first)));
    } else if(beaconPos.first >= robotPos.first && beaconPos.second <= robotPos.second){
        angle = (int)(100*(3*M_PI/2+(double)atan(robotPos.second - beaconPos.second)));
    }
    int alpha = angle - beaconOrientation;
    if(alpha < 0){
        alpha = (int)(alpha + 200*M_PI);
    }
    return alpha;
}


list<pair<double, double>> Localization::getMostProbableAgglomerate(vector<list<pair<double, double>>> &agglomerated_points) {
    pair<int ,int> last_enemy_pos = this->enemyPosition;
    unsigned int most_probable_index = 0;
    int best_distance = Localization::distanceBetween(Localization::getAveragePosition(agglomerated_points[0]), last_enemy_pos);
    int n = (int)agglomerated_points.size();
    for(unsigned int i=1; i<n; i++){
        pair<int, int>agglomerate_average = Localization::getAveragePosition(agglomerated_points[i]);
        int distance = Localization::distanceBetween(agglomerate_average, last_enemy_pos);
        if(distance < best_distance){
            best_distance = distance;
            most_probable_index = i;
        }
    }
    return agglomerated_points[most_probable_index];
}


vector<pair<double, int>> Localization::extractBeaconsMeasuredPoints(sl_lidar_response_measurement_node_hq_t nodes[NODES_LEN], size_t count) {
    vector<pair<double, int>> nearestBeaconDetectedPointRelative(3, make_pair(-1, -1));
    for (int pos = 0; pos < count; ++pos) {
        //checking measurement quality
        if(nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0){
            pair<int, int> position = Localization::robotToCartesian(nodes[pos], this->x_robot, this->y_robot, this->alpha_robot);
            //Select points next to beacons
            if(this->getBeaconsMode()) {
                int beaconNumber = this->getBeaconNumber(position);
                if(beaconNumber != -1){
                    if ((double) nodes[pos].dist_mm_q2 / 4.0 < nearestBeaconDetectedPointRelative[beaconNumber].first || nearestBeaconDetectedPointRelative[beaconNumber].first == -1) {
                        int angle_radians = Localization::rplidarToTrigoRadians(((double)nodes[pos].angle_z_q14 * 90.0) / 16384.0);
                        nearestBeaconDetectedPointRelative[beaconNumber] = make_pair((double) nodes[pos].dist_mm_q2 / 4.0, angle_radians);
                    }
                }
            }
        }
    }
    return nearestBeaconDetectedPointRelative;
}


vector<int> Localization::determineRobotPosition(vector<pair<double, int>> beaconsDistanceAndAngleRelative, vector<bool> beaconsDetected) {
    pair<int, int>* beaconsPos = this->beaconsPositions;
    list<pair<double, double>> circlesIntersectionsEquations;
    if(beaconsDetected[0] && beaconsDetected[1]){
        vector<pair<double,double>> intersectCircles =  Localization::intersectionBetweenCircles(beaconsPos[0], beaconsDistanceAndAngleRelative[0].first + (double)BEACONS_RADIUS, beaconsPos[1], beaconsDistanceAndAngleRelative[1].first + (double)BEACONS_RADIUS);
        pair<double, double> lineEquation =  Localization::lineEquationFromPoints(intersectCircles[0], intersectCircles[1]);
        circlesIntersectionsEquations.emplace_back(lineEquation);
    }
    if(beaconsDetected[1] && beaconsDetected[2]){
        vector<pair<double,double>> intersectCircles =  Localization::intersectionBetweenCircles(beaconsPos[1], beaconsDistanceAndAngleRelative[1].first + (double)BEACONS_RADIUS, beaconsPos[2], beaconsDistanceAndAngleRelative[2].first + (double)BEACONS_RADIUS);
        pair<double, double> lineEquation =  Localization::lineEquationFromPoints(intersectCircles[0], intersectCircles[1]);
        circlesIntersectionsEquations.emplace_back(lineEquation);
    }
    if(beaconsDetected[2] && beaconsDetected[0]){
        vector<pair<double,double>> intersectCircles =  Localization::intersectionBetweenCircles(beaconsPos[2], beaconsDistanceAndAngleRelative[2].first + (double)BEACONS_RADIUS, beaconsPos[0], beaconsDistanceAndAngleRelative[0].first + (double)BEACONS_RADIUS);
        pair<double, double> lineEquation =  Localization::lineEquationFromPoints(intersectCircles[0], intersectCircles[1]);
        circlesIntersectionsEquations.emplace_back(lineEquation);
    }
    list<pair<double, double>> intersections;
    for (auto i = circlesIntersectionsEquations.begin(); i != prev(circlesIntersectionsEquations.end()); i++) {
        for (auto j = next(i); j != circlesIntersectionsEquations.end(); j++) {
            pair<double, double> intersection = Localization::intersectionBetweenLines(*i, *j);
            intersections.emplace_back(intersection);
        }
    }
    pair<int, int> robotPos = Localization::getAveragePosition(intersections);
    int n = 0;
    int angle = 0;
    for(unsigned int i=0; i<3; i++){
        if(beaconsDetected[i]){
            n++;
            angle += Localization::determineRobotOrientation(robotPos, beaconsPos[i], beaconsDistanceAndAngleRelative[i].second);
        }
    }
    if(n != 0){
        angle = angle/n;
    }
    vector<int> result{robotPos.first, robotPos.second, angle};
    return result;
}


void Localization::processPoints(sl_lidar_response_measurement_node_hq_t nodes[NODES_LEN], size_t count) {
    list<pair<double, double>> points_inside;
    vector<pair<double, int>> nearestBeaconDetectedPointRelative(3, make_pair(-1, -1));
    vector<bool> beaconsDetected(3, false);
    bool proximityAlert = false;
    bool positionIncorrect = false;
    for (int pos = 0; pos < count; ++pos) {
        //checking measurement quality
        if(nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0){
            //Checking for direct proximity
            if((double)nodes[pos].dist_mm_q2/4.0f < PROXIMITY_ALERT_RANGE) {
                proximityAlert = true;
                if (this->proximityLastRound) {
                    int angle_radians = Localization::rplidarToTrigoRadians(((double) nodes[pos].angle_z_q14 * 90.f) / 16384.f);
                    this->sendProximityAlert((int) ((double) nodes[pos].dist_mm_q2 / 4.0f), angle_radians);
                }
            }
            //Select points inside map and next to beacons
            pair<int, int> position = Localization::robotToCartesian(nodes[pos], this->x_robot, this->y_robot,this->alpha_robot);
            if (Localization::isInsideMap(position)) {
                points_inside.emplace_back(position);
            } else {
                if (this->getBeaconsMode()) {
                    int beaconNumber = this->getBeaconNumber(position);
                    if ((double) nodes[pos].dist_mm_q2 / 4.0f < nearestBeaconDetectedPointRelative[beaconNumber].first || nearestBeaconDetectedPointRelative[beaconNumber].first == -1) {
                        int angle_radians = Localization::rplidarToTrigoRadians(((double) nodes[pos].angle_z_q14 * 90.f) / 16384.f);
                        nearestBeaconDetectedPointRelative[beaconNumber] = make_pair((double) nodes[pos].dist_mm_q2 / 4.0f, angle_radians);
                        beaconsDetected[beaconNumber] = true;
                    }
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
    this->enemyPosition = averageDetection;
    this->enemyPositionGap = maxGap;
    //Determine approximative robot position from beacons
    if(this->getBeaconsMode()){
        vector<int> robotPos = this->determineRobotPosition(nearestBeaconDetectedPointRelative, beaconsDetected);
        pair<int, int> robotMeasuredPos{robotPos[0], robotPos[1]};
        pair<int, int> robotOdometryPos{this->x_robot, this->y_robot};
        int positionSwitch = Localization::distanceBetween(robotMeasuredPos, robotOdometryPos);
        if(positionSwitch > POSITION_CORRECT_RANGE){
            positionIncorrect = true;
            if(this->positionIncorrectLastRound){
                this->sendMessage("strat", "stop recalibrate", to_string(positionSwitch));
            }
        }
    }
    this->proximityLastRound = proximityAlert;
    this->positionIncorrectLastRound = positionIncorrect;
}


void Localization::processTriangulation(const vector<pair<double, int>>& overallNearestBeaconDetectedPointRelative) {
    vector<bool> beaconsDetected(3, true);
    for(unsigned int i = 0; i < 3; i++){
        if(overallNearestBeaconDetectedPointRelative[i].first ==  -1){
            beaconsDetected[i] = false;
        }
    }
    vector<int> robotPos = this->determineRobotPosition(overallNearestBeaconDetectedPointRelative, beaconsDetected);
    this->sendMessage("strat", "set pos", to_string(robotPos[0]) + ',' + to_string(robotPos[1]) + ',' + to_string(robotPos[2]));
    this->triangulationMode = false;
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
            this->sendMessage(issuer, "pong", "1");
        }
        if (contains(verb, "get health")) {
            if (this->getLidarHealth()) {
                this->sendMessage(issuer, "set health", "1");
            } else {
                this->sendMessage(issuer, "set health", "0");
            }
        }
        if (contains(verb, "get data")) {
            //Get Data from robot
            vector<int> avoidance = this->getAvoidance();
            this->sendMessage(issuer, "set avoidance",to_string(avoidance[0]) + "," + to_string(avoidance[1]) + "," + to_string(avoidance[2]));
        }
        if (contains(verb, "start")) {
            this->started = true;
        }
        if (contains(verb, "stop")) {
            this->started = false;
        }
        if (contains(verb, "set pos")) {
            //Update robot position and orientation
            vector<string> position = split(data, ",");
            this->setRobotPosition(stoi(position[0]), stoi(position[1]), stoi(position[2]));
        }
        if (contains(verb, "set beacon")) {
            //Enable or disable beacons triangulation. This must be put to 0 if no beacons are on the table
            this->setBeaconsMode(stoi(data));
        }
        if (contains(verb, "set team")) {
            //Update beacons position
            if(stoi(data) == 0){
                pair<int, int> positions[3] = BLUE_TEAM_BEACONS_POS;
                this->setBeaconsPosition(positions);
            } else if(stoi(data) == 1){
                pair<int, int> positions[3] = YELLOW_TEAM_BEACONS_POS;
                this->setBeaconsPosition(positions);
            }
        }
        if (contains(verb, "get pos")) {
            //ONLY IF ROBOT IS STOPPED : measure robot position from multiple lidar runs.
            this->setBeaconsMode(true);
            this->triangulationMode = true;
        }
    }
}


void Localization::sendProximityAlert(int distance, int angle) {
    this->sendMessage("all", "stop proximity", to_string(distance) + "," + to_string(angle));
}
