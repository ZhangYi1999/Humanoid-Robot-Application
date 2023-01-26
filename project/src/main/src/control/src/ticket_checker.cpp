#include<ticket_checker.h>

// translation station from string to number
int station_num(std::string station){
    if(station == "Hamburg")
        return 0;
    else if (station == "Hannover")
        return 1;
    else
        return 2;
}

TicketChecker::TicketChecker(){
    this->detector = cv::QRCodeDetector();	
    this->message = "";
   
    ros::param::get("~count",this->ticket_num);

    std::stringstream s;
    for(int i = 0; i < ticket_num; i++){
        int id;
        std::string Name;
        std::string From;
        std::string Destination;

        s.str("");
        s<<"~ticket_"<<i+1<<"/ID";
        ros::param::get(s.str(),id);
        s.str("");
        s<<"~ticket_"<<i+1<<"/Name";
        ros::param::get(s.str(),Name);
        s.str("");
        s<<"~ticket_"<<i+1<<"/From";
        ros::param::get(s.str(),From);
        s.str("");
        s<<"~ticket_"<<i+1<<"/To";
        ros::param::get(s.str(),Destination);

        tickets.push_back(Ticket(id,Name,From,Destination));

        check_face_flag = false;

        ticket_detected = std::vector<int>(5,0);
    }
}
    
TicketChecker::~TicketChecker(){

}

bool TicketChecker::checkQRcode(cv::Mat img){
    cv::Mat points;
    if(this->detector.detect(img,points)){
        this->message = this->detector.decode(img,points);
        return true;
    }
    else{
        return false;
    }
}

bool TicketChecker::checkValid(Train_station current_station, bool &direction){
    if(this->message!=""){
        std::string str=this->message;

        char delim = ',';
        std::size_t previous = 0;
        std::size_t current = str.find(delim);
        std::string temp = str.substr(previous, current - previous);
        int id = atoi(temp.c_str());
        previous = current + 1;
        current = str.find(delim, previous);
        std::string Name = str.substr(previous, current - previous);
        previous = current + 1;
        current = str.find(delim, previous);
        std::string From = str.substr(previous, current - previous);
        previous = current + 1;
        current = str.find(delim, previous);
        std::string Destination = str.substr(previous, current - previous);

        // std::cout<<"id: "<<id<<" name: "<<Name<<" From: "<<From<<" To: "<<Destination<<std::endl;
        // translate station to number
        int from_num = station_num(From); 
        int des_num = station_num(Destination);

        for(int i = 0; i < ticket_num; i++){
            bool same_id = id == this->tickets[i].Id;
            bool same_name = Name.compare(this->tickets[i].Name)==0;
            bool same_from = From.compare(this->tickets[i].From)==0;
            bool same_to = Destination.compare(this->tickets[i].Destination)==0;
            std::cout<<"id: "<<this->tickets[i].Id<<" name: "<<this->tickets[i].Name<<" From: "<<this->tickets[i].From<<" To: "<<this->tickets[i].Destination<<std::endl;
            if(same_id & same_name & same_from & same_to){
                // task 3
                ticket_name = Name;
                // task 4
                ticket_id = id;
                // destination has already arrived or passed
                if (static_cast<int>(current_station) <= des_num) return false;
                // passenger traveling in wrong direction
                if (static_cast<int>(current_station) <= from_num) {
                    direction = false;
                    return false;
                }
                
                return true;
            }
        }
        return false;
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

