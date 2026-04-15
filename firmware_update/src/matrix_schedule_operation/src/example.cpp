#include <iostream>
#include <ctime>

using namespace std;
tm getTimeNow(void);
int main() {
   tm TimeNow = getTimeNow();
   // print various components of tm structure.
   cout << "Year:" << TimeNow.tm_year<<endl;
   cout << "Month: "<< TimeNow.tm_mon<< endl;
   cout << "Day: "<< TimeNow.tm_mday << endl;
   cout << "Time: "<< TimeNow.tm_hour << ":";
   cout << TimeNow.tm_min << ":";
   cout << TimeNow.tm_sec << endl;
   
   time_t a = mktime(&TimeNow);
   cout << a << endl;
   
   tm *form_user;
   form_user->tm_year = TimeNow.tm_year;
   form_user->tm_mon = TimeNow.tm_mon;
   form_user->tm_mday = TimeNow.tm_mday;
   form_user->tm_hour = 22;
   form_user->tm_min = 50;
   form_user->tm_sec = 50;
   time_t a_user = mktime(form_user);
   
  
   cout << a_user << endl;
   
   cout << difftime(a_user, a) << endl;
}

tm getTimeNow(void)
{
     // current date/time based on current system
   time_t now = time(0);

   cout << "Number of sec since January 1,1970 is:: " << now << endl;
   

   tm *ltm = localtime(&now);

   
   tm aa;
   
   aa.tm_year = 1900 + ltm->tm_year;
   aa.tm_mon = 1 + ltm->tm_mon;
   aa.tm_mday = ltm->tm_mday;
   aa.tm_hour = 7+ltm->tm_hour;
   aa.tm_min = ltm->tm_min;
   aa.tm_sec = ltm->tm_sec;
   return aa;
}