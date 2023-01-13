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
    string Name;
    string From;
    string Destination;

    Ticket(int id,string name,string from,string dest) : Id(id), Name(name), From(from), Destination(dest) {};
}


class TicketChecker
{
private:
    cv::QRCodeDetector detector;
    string message;
    vector<Ticket> tickets;
    int ticket_num;
public:
    TicketChecker();
    ~TicketChecker();

    bool checkQRcode(cv::Mat img);
    bool checkValid();

};