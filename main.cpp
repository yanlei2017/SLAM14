#include <iostream>
using namespace std;
<<<<<<< HEAD
int main(int, char**) {
    cout << "Hello, world!\n";
    cout<<sizeof(long double  )<<endl;    
=======
class Clock {
 private:
  int hour, minute, second;

 public:
  Clock(int hour, int minute, int second);
  ~Clock();
};

Clock::Clock(int hour, int minute, int second) {
  if (0 <= hour <= 24 && 0 <= minute <= 60 && 0 <= second <= 60) {
    this->hour = hour;
    this->minute = minute;
    this->second = second;
  } else {
    while (!(0 <= hour <= 24)) {
      hour = (hour - 24) % 24;
    }
    this->hour = hour;

    while (!(0 <= minute <= 60)) {
      minute = (minute - 60) % 60;
    }
    this->minute = minute;

    while (!(0 <= second <= 60)) {
      second = (second - 60) % 60;
    }
    this->second = second;
  }
  cout << "Time initialized at  " << this->hour << ":" << this->minute << ":"
       << this->second << endl;
>>>>>>> 57342546f4de485a313951a128a513abe2754109
}

Clock::~Clock() {}

int main(int, char**) { Clock a(25, 61, 99); }
