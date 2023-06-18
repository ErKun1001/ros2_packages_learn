#include <iostream>
#include <thread>
#include <chrono>

using namespace std;
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  int cnt = 0;
  while(1)
  {
    cout<<"hello word!!"<<cnt++<<endl;
    std::this_thread::sleep_for(chrono::seconds(1));
  }
  
  return 0;
}
