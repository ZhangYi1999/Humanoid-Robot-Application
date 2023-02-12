#include <ros/ros.h>
#include <vector>
#include <sstream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

std::string str_split(std::string &input, char delim);

struct Date{
    int year;
    int month;
    int day;
};

Date dateFromString(std::string date_str);
bool date_compare(Date date1, Date date2);

struct Time{
    int hour;
    int min;
};

Time timeFromString(std::string time_str);
bool time_compare(Time time1, Time time2);

struct Train_station{
    std::string Name;
    Date arrival_date;
    Time arrival_time;
};

bool station_compare(Train_station staion1,Train_station staion2);

// class for the train
class Train{
public:
    std::string name;
    std::vector<Train_station> stations;
    int current_station_id;             
    int station_num;
    Date departure_date;
    Time departure_time;
    Date arrival_date;
    Time arrival_time;

    Train(){}
    ~Train(){}

    std::string getName();
    void load_data(ros::NodeHandle nh_);
    bool moveToNextStation();
    void delay(Date new_arrival_date, Time new_arrival_time);
    Train_station getStationByName(std::string name);
    Train_station getCurrentStation();
    int getStationIdByName(std::string name);
    int stationDirection(std::string station_name);
};

class Ticket
{
public:
    std::string id;
    std::string passenger_Name;
    Date date;
    std::string departure_station;
    Time departure_time;
    std::string destination_station;
    Time arrival_time;
    std::string train_name;
    int ticket_id;
    int used; // check ticket if used

    // blank constructor
    Ticket(){}
    // from string msg to ticket
    Ticket(std::string ticket_msg);

    bool compare(Ticket ticket);
};

// main class
class TicketChecker
{
public:
    cv::QRCodeDetector detector;
    std::string message;
    std::vector<Ticket> tickets;
    int ticket_num;

    Ticket current_ticket;

    bool found_qr_code_flag;
    bool check_face_flag; // if detected face
    
    Train train;

    TicketChecker(){}
    ~TicketChecker(){}

    void load_data(ros::NodeHandle nh_);
    bool checkQRcode(cv::Mat img); // check if there is valid QR code in the img
    bool checkValid(); // check if the ticket is vaild

    Ticket getFullTicketfromMsg(std::string msg);

    bool get_check_face_flag(){return check_face_flag;};

    std::string getMessage();

    std::string getError();
};