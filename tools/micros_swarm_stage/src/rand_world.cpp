#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <stdlib.h>
#include <time.h>

using namespace std;

string world_head = 
"include \"swarm.inc\"\n\
include \"turtlebot_intruder.inc\"\n\
\n\
define floorplan model \n\
(\n\
  # sombre, sensible, artistic\n\
  color \"gray30\"\n\
  \n\
  # most maps will need a bounding box \n\
  boundary 1\n\
  \n\
  gui_nose 0\n\
  gui_grid 0\n\
  gui_outline 0\n\
  gripper_return 0\n\
  fiducial_return 0\n\
  laser_return 1\n\
)\n\
\n\
resolution 0.02\n\
interval_sim 10  # simulation timestep in milliseconds\n\
\n\
window\n\
(\n\
  size [ 600.0 700.0 ]\n\
  center [ 0.0 0.0 ]\n\
  rotate [ 0.0 0.0 ]\n\
  scale 60\n\
)\n\
\n\
floorplan\n\
(\n\
  name \"rand\"\n\
  bitmap \"../maps/rand.png\"\n\
  size [ 100.0 100.0 2.0 ]\n\
  pose [  50.0  50.0 0.0 0.0 ]\n\
  #size [ 40.0 40.0 2.0 ]\n\
  #pose [ 20.0 20.0 0.0 0.0]\n\
)\n\
\n\
define block model\n\
(\n\
size [0.5 7 0.5]\n\
gui_nose 1\n\
)\n\
\n";

float random_float(float min, float max)
{
    float tmp_rand = rand() / float(RAND_MAX);
    return min + max * tmp_rand;
}

int main()
{
    srand(time(NULL));
    int num = 10;
    float min_pos = 0;
    float max_pos = 20;
    ofstream file;
	file.open("rand.world");
    stringstream ss;
    
    ss<<world_head;
    
    for(int i = 0; i < num; i++) {
        ss<<"swarm( pose ["<<random_float(min_pos+5, max_pos-5)<<" "<<random_float(min_pos+5, max_pos-5)<<" 0 0 ] name \"era"<<i<<"\" "<<"color \"red\")"<<endl;
    }
    file<<ss.str();
    
    return 0;
}
