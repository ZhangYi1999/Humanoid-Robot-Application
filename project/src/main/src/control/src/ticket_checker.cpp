#include<ticket_checker.h>

//********** definition of self defined function **********//
std::string str_split(std:string input, char delim){
    // set the start and end point of sub string
    std::size_t previous = 0;
    // position of first char delim
    std::size_t current = input.find(delim); 
    // cut out the sub string
    return input.substr(previous, current - previous); 
}

//********** definition of functions of Date and Time **********//
Date dateFromString(std::string date_str){
    // create a Date container
    Date result;

    // split the date string using char '.'
    result.day = atoi(str_split(date_str,'.').c_str());
    result.month = atoi(str_split(date_str,'.').c_str());
    result.year = atoi(date_str.c_str());

    return result;
}

bool date_compare(Date date1, Date date2){
    bool same_year = date1.year == date2.year; 
    bool same_month = date1.month == date2.month; 
    bool same_day = date1.day == date2.day; 

    return same_year && same_month && same_day;
}

Time timeFromString(std::string time_str){
    // create a time container
    Time result;

    // split the time string using char ':'
    result.hour = atoi(str_split(time_str,':').c_str());
    result.min = atoi(time_str.c_str());

    return result;
}

bool time_compare(Time time1, Time time2){
    bool same_hour = time1.hour == time2.hour;
    bool same_min = time1.min == time2.min;

    return same_hour && same_min;
}

//********** definition of functions of Train Station **********//
bool station_compare(Train_station staion1,Train_station staion2){
    bool same_name = staion1.Name.compare(staion2.Name) == 0;
    bool same_date = date_compare(staion1.arrival_date,staion2.arrival_date);

    return same_name && same_date;
}

//********** definition of functions of class Train **********//
Train::Train(){
    ros::param::get("~station_count",this->station_num);
    ros::param::get("~train_name",this->name);

    std::stringstream s;
    for(int i = 0; i < this->station_num; i++){
        Train_station temp_station;

        s.str("");
        s<<"~station_"<<i+1<<"/Name";
        ros::param::get(s.str(),temp_station.Name);

        std::string date_str;
        s.str("");
        s<<"~station_"<<i+1<<"/Date";
        ros::param::get(s.str(),date_str);
        temp_station.arrival_date = dateFromString(date_str);

        std::string time_str;
        s.str("");
        s<<"~station_"<<i+1<<"/Time";
        ros::param::get(s.str(),time_str);
        temp_station.arrival_time = timeFromString(time_str);

        this->stations.push_back(temp_station);
    }

    this->departure_date = this->stations[0].arrival_date;
    this->departure_time = this->stations[0].arrival_time;
    this->arrival_date = this->stations[this->station_num-1].arrival_date;
    this->arrival_time = this->stations[this->station_num-1].arrival_time;

    this->current_station_id = 0;
}

bool Train::moveToNextStation(){
    // The train moves and arrives at the next station
    this->current_station_id ++; 

    // Determine if the current station is final staion
    if(this->current_station_id == this->station_num-1) {
        // Arrive at final staion
        return true;
    }
    else {
        // Not final staion
        return false;
    }
}

void Train::delay(Date new_arrival_date, Time new_arrival_time){
    this->arrival_date = new_arrival_date;
    this->arrival_time = new_arrival_time;
}

Train_station Train::getStationByName(std::string name){
    for(int i = 0; i < this-> station_num; i++) {
        if(this->stations[i].Name.compare(name)==0){
            return this->station[i];
        }
    }
    Train_station dummy_station;
    dummy_station.Name = "no such station";
    return dummy_station;
}

Train_station Train::getCurrentStation(){
    return this->stations[this->current_station_id];
}

int Train::getStationIdByName(std::string name){
    for(int i = 0; i < this-> station_num; i++) {
        if(this->stations[i].Name.compare(name)==0){
            return i;
        }
    }
    return -1;
}

int Train::stationDirection(std::string station_name){
    int id = getStationIdByName(station_name);
    if(id!=-1){
        if(id > this->current_id)
            return 1;
        else 
            return -1;
    }
    else{
        return 0;
    }
}


//********** definition of functions of class Ticket **********//
Ticket::Ticket(std::string ticket_msg){
    this->id = str_split(ticket_msg,',');
    this->passenger_Name = str_split(ticket_msg,',');
    this->date = dateFromString(str_split(ticket_msg,','));
    this->departure_station = str_split(ticket_msg,',');
    this->destination_station = str_split(ticket_msg,',');
    this->train_name = ticket_msg;
}

bool Ticket::compare(Ticket ticket){
    bool same_id = this->id == ticket.id;
    bool same_name = this->passenger_Name.compare(ticket.passenger_Name)==0;
    bool same_date = date_compare(this->date,ticket.date);
    bool same_to = this->departure_station.compare(ticket.departure_station)==0;
    bool same_from = this->destination_station.compare(ticket.destination_station)==0;
    bool same_train = this->train_name.compare(ticket.train_name) == 0;

    return same_id && same_name && same_date && same_to && same_from && same_train;
}

//********** definition of functions of class TicketChecker **********//

TicketChecker::TicketChecker(){
    this->detector = cv::QRCodeDetector();	
    this->message = "";
   
    ros::param::get("~ticket_count",this->ticket_num);

    std::stringstream s;
    for(int i = 0; i < this->ticket_num; i++){
        std::string id;
        // std::string passenger_Name;
        // std::string date;
        // std::string departure_station;
        // std::string destination_station;
        // std::string train_name;

        s.str("");
        s<<"~ticket_"<<i+1<<"/ID";
        ros::param::get(s.str(),id);
        // s.str("");
        // s<<"~ticket_"<<i+1<<"/Name";
        // ros::param::get(s.str(),passenger_Name);
        // s.str("");
        // s<<"~ticket_"<<i+1<<"/Date";
        // ros::param::get(s.str(),date);
        // s.str("");
        // s<<"~ticket_"<<i+1<<"/From";
        // ros::param::get(s.str(),departure_station);
        // s.str("");
        // s<<"~ticket_"<<i+1<<"/To";
        // ros::param::get(s.str(),destination_station);
        // s.str("");
        // s<<"~ticket_"<<i+1<<"/Train";
        // ros::param::get(s.str(),train_name);

        ticket_ids.push_back(id);   
    }
    
    this->ticket_detected = std::vector<int>(this->ticket_num,0);
    this->found_qr_code_flag = false;
    this->check_face_flag = false;
}
    
TicketChecker::~TicketChecker(){
    // Delete the ticket checker
}

bool TicketChecker::checkQRcode(cv::Mat img){
    cv::Mat points;
    if(this->detector.detect(img,points)){
        // if QR code is detected in the img
        this->message = this->detector.decode(img,points);
        this->found_qr_code_flag = true;
        return true;
    }
    else{
         // if no QR code in the img
        return false;
    }
}

bool TicketChecker::checkValid(Train_station current_station, bool &direction){
    if(this->found_qr_code_flag){
        // reset the found qr code flag
        this->found_qr_code_flag = false;

        // check if the message is in right regular
        if((this->message.substr(0,3)).compare("NAO") == 0) {
            this->current_ticket = Ticket(this->message);
            // check if the ticket is in same direction
            if(this->train.stationDirection(this->current_ticket.destination_station)>0)
                return true;
            else
                return false;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

// face detection prograss
bool TicketChecker::check_face(cv::Mat img){
    // use face detection detect face from camera frame and compare it with "ticket_name"
    // if they are the same, return true, 
    return true;
}

bool TicketChecker::check_attention(cv::Mat img){
    // if face detection can detect a face, return true
    return true;
}

std::string TicketChecker::getMessage(){
    return this->message;
}

