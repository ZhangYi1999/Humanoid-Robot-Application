#include<ticket_checker.h>

//********** definition of self defined function **********//
std::string str_split(std::string &input,char delim){
    // set the start and end point of sub string
    std::size_t previous = 0;
    // position of first char delim
    std::size_t current = input.find(delim);

    std::string result = input.substr(previous, current - previous);
    input =  input.substr(current+1);
    // cut out the sub string
    return result;
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

    // split the time string using char '-'
    result.hour = atoi(str_split(time_str,'-').c_str());
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
std::string Train::getName(){
    return this->name;
}


void Train::load_data(ros::NodeHandle nh_){
    nh_.getParam("/station_count",this->station_num);
    nh_.getParam("/train_name",this->name);
    std::cout<<"Train ID: "<<this->name<<std::endl;
    std::cout<<"Station Num: "<<this->station_num<<std::endl;

    std::stringstream s;
    for(int i = 0; i < this->station_num; i++){
        Train_station temp_station;

        s.str("");
        s<<"/station_"<<i+1<<"/Name";
        nh_.getParam(s.str(),temp_station.Name);

        std::string time_str;
        s.str("");
        s<<"/station_"<<i+1<<"/Time";
        nh_.getParam(s.str(),time_str);
        temp_station.arrival_time = timeFromString(time_str);
        std::cout<<"time: "<<time_str<<std::endl;

        std::string date_str;
        s.str("");
        s<<"/station_"<<i+1<<"/Date";
        nh_.getParam(s.str(),date_str);
        temp_station.arrival_date = dateFromString(date_str);

        std::cout<<"Station "<<i+1<<std::endl;
        std::cout<<"  Name: "<<temp_station.Name<<std::endl;
        std::cout<<"  Date: year:"<<temp_station.arrival_date.year \
                 <<" month:" <<temp_station.arrival_date.month \
                 <<" day:" <<temp_station.arrival_date.day \
                 <<std::endl;
        std::cout<<"  Time: hour:"<<temp_station.arrival_time.hour \
                 <<" min:" <<temp_station.arrival_time.min \
                 <<std::endl;

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
            return this->stations[i];
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
        if(id > this->current_station_id)
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
    // this->id = str_split(ticket_msg,',');
    // this->passenger_Name = str_split(ticket_msg,',');
    // this->date = dateFromString(str_split(ticket_msg,','));
    // this->departure_station = str_split(ticket_msg,',');
    // this->destination_station = str_split(ticket_msg,',');
    // this->train_name = ticket_msg;
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

void TicketChecker::load_data(ros::NodeHandle nh_){
    int ticket_count;
    nh_.getParam("/ticket_count",ticket_count);


    this->train.load_data(nh_);

    this->detector = cv::QRCodeDetector();	
    this->message = "";
   
    nh_.getParam("/ticket_count",this->ticket_num);
    std::cout<<"load "<<ticket_count<<" tickets."<<std::endl;

    std::stringstream s;
    for(int i = 0; i < this->ticket_num; i++){
        Ticket temp_ticket;

        temp_ticket.used = 0;

        temp_ticket.ticket_id = i;
        s.str("");
        s<<"/ticket_"<<i+1<<"/ID";
        nh_.getParam(s.str(),temp_ticket.id);
        std::cout<<"load "<<s.str()<<" "<<temp_ticket.id<<std::endl;

        s.str("");
        s<<"/ticket_"<<i+1<<"/Name";
        nh_.getParam(s.str(),temp_ticket.passenger_Name);

        std::string date_str;
        s.str("");
        s<<"/ticket_"<<i+1<<"/Date";
        nh_.getParam(s.str(),date_str);
        temp_ticket.date = dateFromString(date_str);

        s.str("");
        s<<"/ticket_"<<i+1<<"/From";
        nh_.getParam(s.str(),temp_ticket.departure_station);

        std::string departure_time_str;
        s.str("");
        s<<"/ticket_"<<i+1<<"/DepartureTime";
        nh_.getParam(s.str(),departure_time_str);
        temp_ticket.departure_time = timeFromString(departure_time_str);

        s.str("");
        s<<"/ticket_"<<i+1<<"/To";
        nh_.getParam(s.str(),temp_ticket.destination_station);

        std::string arrival_time_str;
        s.str("");
        s<<"/ticket_"<<i+1<<"/ArrivalTime";
        nh_.getParam(s.str(),arrival_time_str);
        temp_ticket.arrival_time = timeFromString(arrival_time_str);

        s.str("");
        s<<"/ticket_"<<i+1<<"/Train";
        nh_.getParam(s.str(),temp_ticket.train_name);

        this->tickets.push_back(temp_ticket);  
    }

    this->found_qr_code_flag = false;
    this->check_face_flag = false;
}

bool TicketChecker::checkQRcode(cv::Mat img){
    std::vector<cv::Point> points;
    if(this->detector.detect(img,points)){
        // if QR code is detected in the img
        if(!points.empty()){
            std::string msg = this->detector.decode(img,points);
            if(msg.empty()) {
                return false;
            }
            else{
                this->message = msg;
                this->found_qr_code_flag = true;
                return true;
            } 
        }
        else {
            return false;
        }
         
    }
    else{
         // if no QR code in the img
        return false;
    }
}

bool TicketChecker::checkValid(){
    if(this->found_qr_code_flag){
        // reset the found qr code flag
        this->found_qr_code_flag = false;

        // check if the message is in right regular
        if((this->message.substr(0,3)).compare("NAO") == 0) {
            this->current_ticket = getFullTicketfromMsg(this->message);

            Train_station a = this->train.getStationByName(this->current_ticket.departure_station);
            if(!station_compare(a,this->train.getCurrentStation())){
                if(this->train.stationDirection(this->current_ticket.departure_station)<0)
                    return true;
                else
                    return false;
            }
            else{
                return true;
            }


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
bool TicketChecker::check_face(){
    // use face detection detect face from camera frame and compare it with "ticket_name"
    // if they are the same, return true, 
    return true;
}

/* bool TicketChecker::check_attention(cv::Mat img){
    // if face detection can detect a face, return true
    return true;
} */

Ticket TicketChecker::getFullTicketfromMsg(std::string msg){
    for(int i = 0; i < this->ticket_num; i++){
        if(this->tickets[i].id.compare(msg)==0){
            
            return this->tickets[i];
        }
            
    }
}

std::string TicketChecker::getMessage(){
    return this->message;
}

std::string TicketChecker::getError(){
    return "wrong station."; // just for test
}
