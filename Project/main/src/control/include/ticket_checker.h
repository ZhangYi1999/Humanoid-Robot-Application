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
public:
    TicketChecker();
    ~TicketChecker();

    bool checkQRcode(cv::Mat img);
    bool checkValid();
    std::string getMessage();

};