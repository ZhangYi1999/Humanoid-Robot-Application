#include<ticket_checker.h>

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

bool TicketChecker::checkValid(){
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

        for(int i = 0; i < ticket_num; i++){
            bool same_id = id == this->tickets[i].Id;
            bool same_name = Name.compare(this->tickets[i].Name)==0;
            bool same_from = From.compare(this->tickets[i].From)==0;
            bool same_to = Destination.compare(this->tickets[i].Destination)==0;
            std::cout<<"id: "<<this->tickets[i].Id<<" name: "<<this->tickets[i].Name<<" From: "<<this->tickets[i].From<<" To: "<<this->tickets[i].Destination<<std::endl;
            if(same_id & same_name & same_from & same_to){
                return true;
            }
        }
        return false;
    }
    else{
        return false;
    }
}

std::string TicketChecker::getMessage(){
    return this->message;
}
