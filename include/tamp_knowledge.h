#include "tamp_msgs/sceneobjects.h"
#include"world.h"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "tamp_msgs/sceneobjects.h"
#include "tamp_msgs/knowledge.h"
#include <boost/algorithm/string.hpp>
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose.h"
#include "tamp_msgs/ackquest.h"
#include <math.h>


#define RST "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYLW  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KCYN  "\x1B[36m"

#define FRED(x)  KRED x RST
#define FGRN(x)  KGRN x RST
#define FBLU(x)  KBLU x RST
#define FYLW(x)  KYLW x RST
#define FCYN(x)  KCYN x RST
#define BOLD(x)  "\x1B[1m" x RST

using namespace std;


class tamp_knowledge
{
public:
	tamp_knowledge();
	~tamp_knowledge();
	void readDataBase();
	bool knowledgeQuery(tamp_msgs::knowledge::Request& request, tamp_msgs::knowledge::Response& response);
	bool sceneQuery();
	bool robotStateQuery();
	void findClosestObject();
	void lookUpInDatabase();
	void updateScene();
	void updateDataBase(std::vector<std_msgs::String> objtypes,std::vector<geometry_msgs::Transform> objposes);
    void updatemetrics();
    int minEucleadan(std::vector<std::vector<float>> objs,std::vector<float> des);
    void findClosestObjects();
    void updatesceneItself();
    void eliminateCB(std_msgs::String::ConstPtr msg);

private:
	std::vector<world> dataBase_;
	ros::NodeHandle nh;
	ros::Publisher pubToRobotDisplay;
	ros::Subscriber objectelominate;
	ros::ServiceServer knowledgeServer;
	ros::ServiceClient sceneClient;
	ros::ServiceClient robotStateClient;
	bool sceneupdated_;
	int nuofobj_;
	
};
