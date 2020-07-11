#include "tamp_knowledge.h"



tamp_knowledge::tamp_knowledge(){

    pubToRobotDisplay=nh.advertise<std_msgs::String>("robotDisplayText",20);
   
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	
	

	knowledgeServer = nh.advertiseService("tamp_knowledge_service",&tamp_knowledge::knowledgeQuery,this);
	sceneClient= nh.serviceClient<tamp_msgs::sceneobjects>("tamp_scene_service");
	robotStateClient=nh.serviceClient<tamp_msgs::ackquest>("tamp_ack_service");
	
	objectelominate = nh.subscribe("eliminate_object",80,&tamp_knowledge::eliminateCB,this);
    readDataBase();


	cout<<BOLD("******************************************************")<<endl;
	



}

bool tamp_knowledge::knowledgeQuery(tamp_msgs::knowledge::Request& request, tamp_msgs::knowledge::Response& res){
    ROS_INFO("knowledge receieved Query");
   
    if(request.updatescene){
    	//tamp_msgs::knowledge::Response resp;
     //response =resp;
        
        updateScene();
    }
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string type=request.reqType;
	string name=request.Name;
	string requestInfo=request.requestInfo;

	cout<<"MSG:: type: "<<type<<", requestInfo: "<<requestInfo<<endl;
	vector<string> typeVec,requestInfoVec;
	boost::split(typeVec, type, boost::is_any_of("")); // Exmpl: Point-Point1, Point1, Cylinder-Cylinder1, Cylinder, Cylinder-Cylinder1-graspingPose1, Cylinder-Cylinder1-centerFrame
	boost::split(requestInfoVec, requestInfo, boost::is_any_of("-"));// Pose, Pose-Name, Center, Center-Name, boundingBox, boundingBall
	bool ret_name=false; // trturn name (if false: value)

	if(requestInfo.find("Name") != std::string::npos)
	{
		ret_name=true;
		cout<<"return name"<<endl;
	}
	else
	{
		cout<<"return values"<<endl;
	}

	for(int i=0;i<dataBase_.size();i++)
	{
		int Occurence=0;
		for(int j=0; j<dataBase_[i].name.size();j++)// cylinder cylinder1 graspingPose1, point4 Pose,
		{
			for(int k=0;k<typeVec.size();k++) // cylinder1 graspingPose1
			{
				//				cout<<"worldVec["<<i<<"].name["<<j<<"]: "<<worldVec[i].name[j]<<", typeVec["<<k<<"]: "<<typeVec[k]<<endl;
				if(dataBase_[i].name[j]==typeVec[k])
				{
					Occurence++;
					//					cout<<Occurence<<endl;
				}
			}
			if(Occurence==typeVec.size() && dataBase_[i].name.back().find(requestInfoVec[0]) != std::string::npos)
			{
				if(ret_name==true)
				{
					string KB_name;
					KB_name+=dataBase_[i].name[0];
					for(int l=1; l<dataBase_[i].name.size();l++)
						KB_name+="-"+dataBase_[i].name[l];

					cout<<"Res: "<<KB_name<<endl;
					res.names.push_back(KB_name);
				}
				else
				{   if(dataBase_[i].name.size()>1){
					for(size_t k=1;k<dataBase_[i].name.size();k++){

						res.names.push_back(dataBase_[i].name[k]);
					}
					
				}
					cout<<"Res: ";
					for(int m=0;m<dataBase_[i].value.size();m++)
					{
						res.pose.push_back(dataBase_[i].value[m]);

						cout<<dataBase_[i].value[m]<<" ";
					}
					
					
					cout<<endl;
					cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
					return true;// normally when ask for a vector value, it is just one vector value
				}
				break; // if a index of world vec happened, the same index should be passed and not happen again.
			}
		}
	}
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	return true;

	

}
tamp_knowledge::~tamp_knowledge(){


}

void tamp_knowledge::readDataBase(){


    const char* home=getenv("HOME");
	string pointPath(home);
	string pointsPath=pointPath+"/catkin_ws/src/TAMP/tamp_knowledge/files/poses.txt";
	ifstream file_path_ifStr(pointsPath.c_str());
	string line;
	vector<string> line_list;

	string delim_type=" ";
	if (file_path_ifStr.is_open())
	{
		while(getline(file_path_ifStr,line))
		{
			boost::split(line_list, line, boost::is_any_of(delim_type));
			if(line_list[0]!="#")
			{
				vector<float> Pose;
				vector<string> Name;
				for(int i=1;i<line_list.size();i++)
				{
					Pose.push_back( stof(line_list[i]) );
				}

				Name.push_back(line_list[0]);
				//Name.push_back("Pose");
				world temp_point(Name,Pose);
				dataBase_.push_back(temp_point);
			}
		}
	}


	int right_ee_pose=dataBase_.size();
    int left_ee_pose = right_ee_pose+1;
    int closest_object_to_rightee = left_ee_pose+1;
    int closest_object_to_leftee = closest_object_to_rightee+1;
    int closest_object_to_base = closest_object_to_leftee+1;
    int num_of_objects = closest_object_to_base+1;
    int closest_object_to_target =num_of_objects+1;
    int largest_object = closest_object_to_target+1;
    dataBase_.resize(largest_object+1);
    dataBase_[right_ee_pose].name.push_back("righteepose");
    dataBase_[left_ee_pose].name.push_back("lefteepose");
    dataBase_[closest_object_to_rightee].name.push_back("clobree");
    dataBase_[closest_object_to_leftee].name.push_back("cloblee");
    dataBase_[closest_object_to_base].name.push_back("clobb");
    dataBase_[num_of_objects].name.push_back("num_of_objects");
    dataBase_[closest_object_to_target].name.push_back("clobt");
    dataBase_[largest_object].name.push_back("largestobject");
    for(int i=0;i<dataBase_.size();i++)
		dataBase_[i].print();
     updatesceneItself();






	

}

void tamp_knowledge::updatesceneItself(){
	ROS_INFO("Updating Scene ItSelf Database");
	//tamp_msgs::sceneobjects msg;
	//msg.request.update=true;
	std::vector<std_msgs::String> objtypes;
	std::vector<geometry_msgs::Transform> objposes;
    std::vector<float> boundboxvec;
    std::vector<string> objecttype;
    std::vector<string> grasptype;
    std::vector<string> objid;
    for(std::size_t i=0;i<dataBase_.size();i++){
       boost::split(objid, dataBase_[i].name[0], boost::is_any_of("-"));
       boost::split(objecttype, dataBase_[i].name[0], boost::is_any_of("_"));
       ROS_INFO("Updating Scene ItSelf Database1");

       if(objecttype[0]=="cylinder" ||objecttype[0]=="cube"){
       	ROS_INFO("Updating Scene ItSelf Database2");
              boost::split(grasptype, objecttype[1], boost::is_any_of("-"));
              ROS_INFO("Updating Scene ItSelf Database3");
              if(grasptype.size()>1){
	              if(grasptype[1]=="grasp"){
	              	ROS_INFO("Updating Scene ItSelf Database4");

	                      std_msgs::String str;
			              str.data=objid[0];
			              objtypes.push_back(str);
			              geometry_msgs::Transform msg;
			              msg.translation.x=dataBase_[i].value[0];
			              msg.translation.y=dataBase_[i].value[1];
			              msg.translation.z=dataBase_[i].value[2];
			              msg.rotation.x = dataBase_[i].value[3];
			              msg.rotation.y = dataBase_[i].value[4];
			              msg.rotation.z = dataBase_[i].value[5];
			              objposes.push_back(msg);

              }
          }
          }
      }
            
              
    

   updateDataBase(objtypes,objposes);


}
       	
       	











void tamp_knowledge::updateScene(){
    ROS_INFO("Updating Scene Database");
	tamp_msgs::sceneobjects msg;
	msg.request.update=true;
	std::vector<std_msgs::String> objtypes;
	std::vector<geometry_msgs::Transform> objposes;
    std::vector<float> boundboxvec;
   if(sceneClient.call(msg)){

       objtypes=msg.response.types;
       objposes = msg.response.baseposes;
       boundboxvec = msg.response.boundbox;
       ROS_INFO("Updated Database");
   
   }

   else{

   	ROS_INFO("Couldn't Update Database");
   }

   updateDataBase(objtypes,objposes);

    


}

void tamp_knowledge::updateDataBase(std::vector<std_msgs::String> objtypes,std::vector<geometry_msgs::Transform> objposes){
    


    for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){
    	 std::vector<string> requestInfoVec;
        boost::split(requestInfoVec, it->name[0], boost::is_any_of("_"));
    	if(it->name[0]=="num_of_objects"){
    		//it->value.clear();
          //  it->value.push_back(objtypes.size());
           
            dataBase_.erase(it);
    	}
    	else if(requestInfoVec[0]=="object"){
           
    		dataBase_.erase(it);
    	}
    	else if(it->name[0]=="boundbox")
    	{

    		dataBase_.erase(it);
    	}

		else
			{it++;}
	}

    world inst,instbound;
    inst.name.push_back("num_of_objects");
    inst.value.push_back(objtypes.size());
    nuofobj_=objtypes.size();
    //inst.name.resize(objtypes.size());
    for(std::size_t i=0;i<objtypes.size();i++){
    	inst.name.push_back(objtypes[i].data);

    }
    dataBase_.push_back(inst);
    
    //instbound.name.push_back("boundbox");
    //instbound.value = boundbouxing;
   // dataBase_.push_back(instbound);
    for(std::size_t i=0;i<objtypes.size();i++){
		world inst;
		ROS_INFO("Inserting scene objects");
		//string objname = "object_"+to_string(i+1);
		//inst.name.push_back(objname);
		//inst.name.push_back("object");
		inst.name.push_back(objtypes[i].data);
		inst.value.push_back(objposes[i].translation.x);
		inst.value.push_back(objposes[i].translation.y);
		inst.value.push_back(objposes[i].translation.z);
		//inst.value.push_back(0.0);
		inst.value.push_back(objposes[i].rotation.x);
		inst.value.push_back(objposes[i].rotation.y);
		inst.value.push_back(objposes[i].rotation.z);
		dataBase_.push_back(inst);
	  

    }

     updatemetrics();

    ROS_INFO("New Database Values");
    for(int i=0;i<dataBase_.size();i++)
		dataBase_[i].print();
	


}


void tamp_knowledge::updatemetrics(){
   
    tamp_msgs::ackquest msgr;
    geometry_msgs::PoseStamped posr;
    tamp_msgs::ackquest msgl;
    geometry_msgs::PoseStamped posl;
    msgr.request.arm ="right";
    msgl.request.arm ="left";
    ROS_INFO("msgl.request.arm");
    tamp_msgs::ackquest msg;
    //ros::spinOnce();
    if(robotStateClient.call(msgr))
    {
        posr =  msgr.response.eepos;
        cout<<"***********ree x:"<<posr.pose.position.x<<endl;
        cout<<"***********ree y:"<<posr.pose.position.y<<endl;
        cout<<"***********ree z:"<<posr.pose.position.z<<endl;
        
        
    }
    else{
         ROS_INFO("Couldn't Call end effector Service right");

    }
     if(robotStateClient.call(msgl))
    {
        posl =  msgl.response.eepos;
        
        
    }
    else{
         ROS_INFO("Couldn't Call end effector Service left");

    }

for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){
    	
    	
    	if(it->name[0]=="righteepose" || it->name[0]=="lefteepose"){

    		dataBase_.erase(it);
    	

    	}
    	

		else
			{it++;}
	}
	world instr,instl;
	instr.name.push_back("righteepose");
	instr.value.push_back(posr.pose.position.x);
	instr.value.push_back(posr.pose.position.y);
	instr.value.push_back(posr.pose.position.z);
	instr.value.push_back(posr.pose.orientation.x);
	instr.value.push_back(posr.pose.orientation.y);
	instr.value.push_back(posr.pose.orientation.z);
	instr.value.push_back(posr.pose.orientation.w);
    dataBase_.push_back(instr);

    instl.name.push_back("lefteepose");

	instl.value.clear();
    instl.value.push_back(posl.pose.position.x);
    instl.value.push_back(posl.pose.position.y);
    instl.value.push_back(posl.pose.position.z);
    instl.value.push_back(posl.pose.orientation.x);
    instl.value.push_back(posl.pose.orientation.y);
    instl.value.push_back(posl.pose.orientation.z);
    instl.value.push_back(posl.pose.orientation.w);
    dataBase_.push_back(instl);

   findClosestObjects();

    




}


void tamp_knowledge::findClosestObjects(){

     
    ROS_INFO("findClosestObjects");
    std::vector<std::vector<float>> posobjs;
    std::vector<float> targetpos;
    std::vector<float> righeepos;
    std::vector<float> lefteepos;
    std::vector<float> basepos={0.0,0.0,0.0};
    std::vector<string> objids;


    for(std::size_t i=0;i<dataBase_.size();i++){
        std::vector<string> requestInfoVec,grasptype;
        boost::split(requestInfoVec, dataBase_[i].name[0], boost::is_any_of("_"));
      if(requestInfoVec[0]=="cylinder" ||requestInfoVec[0]=="cube"){
      	
        boost::split(grasptype, dataBase_[i].name[0], boost::is_any_of("-"));

         if(grasptype.size()>1){

             if(grasptype[1]=="grasp"){
         
             		if(nuofobj_>1){
             			   if(grasptype[0]!="cylinder_target"){
					             std::vector<float> v{dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]
					         	,dataBase_[i].value[3],dataBase_[i].value[4],dataBase_[i].value[5]};
					             posobjs.push_back(v);
					             objids.push_back(grasptype[0]);

      						}

             		}
             		else if(nuofobj_==1){
             			std::vector<float> v{dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]
					    ,dataBase_[i].value[3],dataBase_[i].value[4],dataBase_[i].value[5]};
					    posobjs.push_back(v);
					    objids.push_back(grasptype[0]);


             		}
              }
      	
         }

      }
      if(dataBase_[i].name[0]=="righteepose"){

         righeepos ={dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]
         	,dataBase_[i].value[3],dataBase_[i].value[4],dataBase_[i].value[5]};

      }
     if(dataBase_[i].name[0]=="lefteepose"){

         lefteepos ={dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]
         	,dataBase_[i].value[3],dataBase_[i].value[4],dataBase_[i].value[5]};
      }
      if(dataBase_[i].name[0]=="cylinder_target"){
          targetpos ={dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]
         	,dataBase_[i].value[3],dataBase_[i].value[4],dataBase_[i].value[5]};
      }


    }


  ROS_INFO("findClosestObjects2");

int mindistarget = minEucleadan(posobjs,targetpos);
int mindistrightee = minEucleadan(posobjs,righeepos);
int mindistleftee = minEucleadan(posobjs,lefteepos);
int mindistbase = minEucleadan(posobjs,basepos);
  ROS_INFO("findClosestObjects3");
 for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){
        std::vector<string> metricvec;
    	boost::split(metricvec,it->name[0], boost::is_any_of("-"));

    	if(metricvec[0]=="clobree"||metricvec[0]=="cloblee"||metricvec[0]=="clobb"
    		||metricvec[0]=="clobt"||metricvec[0]=="larob"){
            
             dataBase_.erase(it);

            }
    	

		else
			{it++;}
	}


world instlar;
instlar.name.push_back("larob");
instlar.name.push_back("cube_1");

dataBase_.push_back(instlar);




for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]=="cube_1-pregrasp"){
        instlar.name.clear();
        instlar.value.clear();
        instlar.name.push_back("larob-pregrasp");
		instlar.value=dataBase_[i].value;
		dataBase_.push_back(instlar);

	}
	if(dataBase_[i].name[0]=="cube_1-postgrasp"){

		instlar.name.clear();
        instlar.value.clear();
        instlar.name.push_back("larob-postgrasp");
		instlar.value=dataBase_[i].value;
		dataBase_.push_back(instlar);
	}
	if(dataBase_[i].name[0]=="cube_1-grasp"){

		instlar.name.clear();
        instlar.value.clear();
        instlar.name.push_back("larob-grasp");
		instlar.value=dataBase_[i].value;
		dataBase_.push_back(instlar);
	}
}





















world insteer,insteel,instb,instt;

insteer.name.push_back("clobree");
insteer.name.push_back(objids[mindistrightee]);
insteer.value.push_back(posobjs[mindistrightee][0]);
insteer.value.push_back(posobjs[mindistrightee][1]);
insteer.value.push_back(posobjs[mindistrightee][2]);
insteer.value.push_back(posobjs[mindistrightee][3]);
insteer.value.push_back(posobjs[mindistrightee][4]);
insteer.value.push_back(posobjs[mindistrightee][5]);
dataBase_.push_back(insteer);

  ROS_INFO("findClosestObjects5");
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobree-grasp");
insteer.value.push_back(posobjs[mindistrightee][0]);
insteer.value.push_back(posobjs[mindistrightee][1]);
insteer.value.push_back(posobjs[mindistrightee][2]);
insteer.value.push_back(posobjs[mindistrightee][3]);
insteer.value.push_back(posobjs[mindistrightee][4]);
insteer.value.push_back(posobjs[mindistrightee][5]);
dataBase_.push_back(insteer);
  ROS_INFO("findClosestObjects6");
std::vector<float> valuespre,valuespost;
for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]==objids[mindistrightee]+"-pregrasp"){

		valuespre=dataBase_[i].value;
	}
	if(dataBase_[i].name[0]==objids[mindistrightee]+"-postgrasp"){

		valuespost=dataBase_[i].value;
	}
}
  ROS_INFO("findClosestObjects7");
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobree-pregrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespre[i]);

}
  ROS_INFO("findClosestObjects8");
dataBase_.push_back(insteer);
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobree-postgrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespost[i]);

}
dataBase_.push_back(insteer);

  ROS_INFO("findClosestObjects9");




insteel.name.push_back("cloblee");
insteel.name.push_back(objids[mindistleftee]);
insteel.value.push_back(posobjs[mindistleftee][0]);
insteel.value.push_back(posobjs[mindistleftee][1]);
insteel.value.push_back(posobjs[mindistleftee][2]);
insteel.value.push_back(posobjs[mindistleftee][3]);
insteel.value.push_back(posobjs[mindistleftee][4]);
insteel.value.push_back(posobjs[mindistleftee][5]);
dataBase_.push_back(insteel);
  ROS_INFO("findClosestObjects10");

insteel.name.clear();
insteel.value.clear();
insteel.name.push_back("cloblee-grasp");
insteel.value.push_back(posobjs[mindistleftee][0]);
insteel.value.push_back(posobjs[mindistleftee][1]);
insteel.value.push_back(posobjs[mindistleftee][2]);
insteel.value.push_back(posobjs[mindistleftee][3]);
insteel.value.push_back(posobjs[mindistleftee][4]);
insteel.value.push_back(posobjs[mindistleftee][5]);
dataBase_.push_back(insteel);
  ROS_INFO("findClosestObjects11");
valuespre.clear();
valuespost.clear();
for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]==objids[mindistleftee]+"-pregrasp"){

		valuespre=dataBase_[i].value;
	}
	if(dataBase_[i].name[0]==objids[mindistleftee]+"-postgrasp"){

		valuespost=dataBase_[i].value;
	}
}
  ROS_INFO("findClosestObjects12");
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("cloblee-pregrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespre[i]);

}
dataBase_.push_back(insteer);
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("cloblee-postgrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespost[i]);

}
dataBase_.push_back(insteer);





  ROS_INFO("findClosestObjects13");
instb.name.push_back("clobb");
instb.name.push_back(objids[mindistbase]);
instb.value.push_back(posobjs[mindistbase][0]);
instb.value.push_back(posobjs[mindistbase][1]);
instb.value.push_back(posobjs[mindistbase][2]);
instb.value.push_back(posobjs[mindistbase][3]);
instb.value.push_back(posobjs[mindistbase][4]);
instb.value.push_back(posobjs[mindistbase][5]);
dataBase_.push_back(instb);


instb.name.clear();
instb.value.clear();
instb.name.push_back("clobb-grasp");
instb.value.push_back(posobjs[mindistbase][0]);
instb.value.push_back(posobjs[mindistbase][1]);
instb.value.push_back(posobjs[mindistbase][2]);
instb.value.push_back(posobjs[mindistbase][3]);
instb.value.push_back(posobjs[mindistbase][4]);
instb.value.push_back(posobjs[mindistbase][5]);
dataBase_.push_back(instb);




valuespre.clear();
valuespost.clear();
for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]==objids[mindistbase]+"-pregrasp"){

		valuespre=dataBase_[i].value;
	}
	if(dataBase_[i].name[0]==objids[mindistbase]+"-postgrasp"){

		valuespost=dataBase_[i].value;
	}
}
  ROS_INFO("findClosestObjects14");
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobb-pregrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespre[i]);

}
dataBase_.push_back(insteer);
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobb-postgrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespost[i]);

}
dataBase_.push_back(insteer);




instt.name.push_back("clobt");
instt.name.push_back(objids[mindistarget]);
instt.value.push_back(posobjs[mindistarget][0]);
instt.value.push_back(posobjs[mindistarget][1]);
instt.value.push_back(posobjs[mindistarget][2]);
instt.value.push_back(posobjs[mindistarget][3]);
instt.value.push_back(posobjs[mindistarget][4]);
instt.value.push_back(posobjs[mindistarget][5]);
dataBase_.push_back(instt);


instt.name.clear();
instt.value.clear();
instt.name.push_back("clobt-grasp");
instt.value.push_back(posobjs[mindistarget][0]);
instt.value.push_back(posobjs[mindistarget][1]);
instt.value.push_back(posobjs[mindistarget][2]);
instt.value.push_back(posobjs[mindistarget][3]);
instt.value.push_back(posobjs[mindistarget][4]);
instt.value.push_back(posobjs[mindistarget][5]);
dataBase_.push_back(instt);






valuespre.clear();
valuespost.clear();
for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]==objids[mindistarget]+"-pregrasp"){

		valuespre=dataBase_[i].value;
	}
	if(dataBase_[i].name[0]==objids[mindistarget]+"-postgrasp"){

		valuespost=dataBase_[i].value;
	}
}

insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobt-pregrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespre[i]);

}
dataBase_.push_back(insteer);
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobt-postgrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespost[i]);

}
dataBase_.push_back(insteer);








for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]=="cylinder_target-grasp"){
       world insttarget;
       insttarget.name.push_back("target-grasp");
	   insttarget.value =dataBase_[i].value ;
	   dataBase_.push_back(insttarget);

	}
	else if(dataBase_[i].name[0]=="cylinder_target-pregrasp"){

		world insttarget;
       insttarget.name.push_back("target-pregrasp");
	   insttarget.value =dataBase_[i].value ;
	    dataBase_.push_back(insttarget);
	}
	else if(dataBase_[i].name[0]=="cylinder_target-postgrasp"){

		world insttarget;
       insttarget.name.push_back("target-postgrasp");
	   insttarget.value =dataBase_[i].value ;
	    dataBase_.push_back(insttarget);
	}
}
       world insttarget;
       insttarget.name.push_back("target");
       insttarget.name.push_back("cylinder_target");
	    dataBase_.push_back(insttarget);



}

int tamp_knowledge::minEucleadan(std::vector<std::vector<float>> objs,std::vector<float> des){
  ROS_INFO("minEucleadan");
  int j=0;
  double min = 50;
  for(std::size_t i=0;i<objs.size();i++){

  	double dx = objs[i][0] - des[0];
  	double dy = objs[i][1] - des[1];
  	double dz = objs[i][2] - des[2];
    
    double dis = sqrt(dx*dx+dy*dy+dz*dz);
   // if(dis<0.02){
   // 	i++;
   // }
    if(dis<min){

    	min = dis;
    	j++;
    }




  }

  return j-1;


}

void tamp_knowledge::eliminateCB(std_msgs::String::ConstPtr msg){
   ROS_INFO("Elimitating %s from scene",msg->data.c_str());
    string obj =msg->data;
    string itobj;

    for(size_t i=0;i<dataBase_.size();i++){

        if(dataBase_[i].name[0]==obj){

        	itobj= dataBase_[i].name[1];
        }
          


    }
   

  ROS_INFO("Elimitating %s from scene",itobj.c_str());
  for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){
    	
    	if(it->name[0]==itobj || it->name[0]==itobj+"-grasp" ||it->name[0]==itobj+"-pregrasp" ||it->name[0]==itobj+"-postgrasp"){
            
           
           dataBase_.erase(it);

    	}
    	else
			{it++;}

}
 
updatesceneItself();
}