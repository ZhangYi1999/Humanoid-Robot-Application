#include<qr_code.h>

TicketChecker::TicketChecker(){
    this.detector = cv::QRCodeDetector::QRCodeDetector();	
    this.message = null;
   
    ros::param::get("count",this.ticket_num);

    std::stringstream s;
    for(int i = 0; i < ticket_num; i++){
        int id;
        string Name;
        string From;
        string Destination;

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

        tickets.push_back(new Ticket(id,Name,From,Destination));
    }
}
    
TicketChecker::~TicketChecker(){

}

bool TicketChecker::checkQRcode(cv::Mat img){
    cv::Mat points;
    if(this.detector.detect(img,points)){
        this.message = this.detector.decode(img,points);
        return true;
    }
    else{
        return false;
    }
}

bool TicketChecker::checkValid(){
    if(this.message!=null){
        string str;
        this.message.copy(str,this.message.length());

        char delim = ',';
        std::size_t previous = 0;
        std::size_t current = str.find(delim);
        string temp = str.substr(previous, current - previous);
        int id = atoi(temp.c_str());
        previous = current + 1;
        current = str.find(delim, previous);
        string Name = str.substr(previous, current - previous);
        previous = current + 1;
        current = str.find(delim, previous);
        string From = str.substr(previous, current - previous);
        previous = current + 1;
        current = str.find(delim, previous);
        string Destination = str.substr(previous, current - previous);

        for(int i = 0; i < ticket_num; i++){
            bool same_id = id == this.tickets.Id;
            bool same_name = Name.compare(this.tickets[i].Name)==0;
            bool same_from = From.compare(this.tickets[i].From)==0;
            bool same_to = Destination.compare(this.tickets[i].Destination)==0;
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