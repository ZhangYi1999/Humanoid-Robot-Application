#include <ros/ros.h>
#include <vector>
#include <sstream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

class Ticket
{
public:
    int Id;
    std::string Name;
    std::string From;
    std::string Destination;

    Ticket(int id,std::string name,std::string from,std::string dest) : Id(id), Name(name), From(from), Destination(dest) {};
};


class TicketChecker
{
private:
    cv::QRCodeDetector detector;
    std::string message;
    std::vector<Ticket> tickets;
    int ticket_num;

    bool check_face_flag; // if detected face
    std::string ticket_name; // name from ticket

public:
    TicketChecker();
    ~TicketChecker();

    bool checkQRcode(cv::Mat img);

    bool checkValid(); // check if the ticket is vaild
    void check_face(cv::Mat img); // find the face and get the name of face
    
    bool get_check_face_flag(){return check_face_flag;};
    std::string get_ticket_name(){return ticket_name;};
    //bool check_station();

    std::string getMessage();

    // task 3
    bool check_attention(cv::Mat img);

};