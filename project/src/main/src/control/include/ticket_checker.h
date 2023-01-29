#include <ros/ros.h>
#include <vector>
#include <sstream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

std::string str_split(std:string input, char delim);

struct Date{
    int year;
    int month;
    int day;
}

Date dateFromString(std::string date_str);
bool date_compare(Date date1, Date date2);

struct Time{
    int hour;
    int min;
}

Time timeFromString(std::string time_str);
bool time_compare(Time time1, Time time2);

struct Train_station{
    std::string Name;
    Date arrival_date;
    Time arrival_time;
};

bool station_compare(Train_station staion1,Train_station staion2);

class Train{
private:
    std::string name;
    std::vector<Train_station> stations;
    int current_station_id;
    int station_num;
    Date departure_date;
    Time departure_time;
    Date arrival_date;
    Time arrival_time;

public:
    Train();
    bool moveToNextStation();
    void delay(Date new_arrival_date, Time new_arrival_time);
    Train_station getStationByName(std::string name);
    Train_station getCurrentStation();
    int getStationIdByName(std::string name);
    int stationDirection(Train_station station);
};

class Ticket
{
public:
    std::string id;
    std::string passenger_Name;
    Date date;
    std::string departure_station;
    std::string destination_station;
    std::string train_name;

    Ticket(std::string ticket_msg);

    bool compare(Ticket ticket);
};


class TicketChecker
{
private:
    Train train;

    cv::QRCodeDetector detector;
    std::string message;
    std::vector<std::string> ticket_ids;
    std::vector<int> ticket_checked; // used for count the passagers, so they never use the ticket twice
    int ticket_num;

    Ticket current_ticket;

    bool found_qr_code_flag;
    bool check_face_flag; // if detected face
    
public:
    TicketChecker();
    ~TicketChecker();

    bool checkQRcode(cv::Mat img);

    bool checkValid(Train_station current_station, bool &direction); // check if the ticket is vaild
    bool check_face(cv::Mat img); // find the face and get the name of face
    
    bool get_check_face_flag(){return check_face_flag;};
    std::string get_ticket_name(){return ticket_name;};
    //bool check_station();

    std::string getMessage();

    // task 3
    bool check_attention(cv::Mat img);

    // task 4
    void change_ticket(){ticket_detected[ticket_id]++;};
    int get_ticket_valid(){return ticket_detected[ticket_id];};

};