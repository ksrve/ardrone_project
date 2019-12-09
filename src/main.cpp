#include <ardrone.h>


using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ardrone_node");

    cout.flush();
    cout << "Starting ardrone node...\n";
    Ardrone ardrone;
    
ifstream file("2point.txt");

while(ros::ok()){
        ardrone.takeoff();
        sleep(1.0);
        ardrone.hover();
        sleep(0.5);

        ROS_INFO("Trying to open file...");

        if (file.is_open()){
            
            ROS_INFO("File is open. Starting...");
            sleep(1.0);

            string line;

            while(getline(file, line)){
                double x;
                double y;
                double z;

                istringstream iss(line);

                iss >> x >> y >> z;
            
                ardrone.updatePIDControl(x, y, z);
                ROS_INFO("-------------REACHED---TARGET----------------");
            }
        }
        
        else
        {
            ROS_INFO(" Can not open file :( ");
        } 

        sleep(1.0);
        ardrone.hover();
        sleep(1.0);
        ardrone.land();
        sleep(5);
        break;
    }
}

  

