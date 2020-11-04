//these are all header file declaration
//#define _GLIBCXX_USE_CXX11_ABI 0
//sensor_msgs::msgs::Range
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h> 
#include <nodelet/nodelet.h>
#include <fstream>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <gtec_msgs/Ranging.h>
#include <sensor_msgs/Range.h>
//#include <mrs_msgs/TrackerPoint.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_srvs/SetBool.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/RtkGps.h>
#include <mrs_lib/param_loader.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Trigger.h>
#include <math.h>
#include <string.h>
#include <map> 
#include <iterator> 
#include <std_msgs/String.h> 
#include <Eigen/Dense>

namespace localization
{

  class uwb_start : public nodelet::Nodelet
  {
  public:
	virtual void onInit();
	ros::NodeHandle nh;
	struct locate{
	//bool localization_enable;
	//geometry_msgs::Point anchor[4];
	//float dist[3];
	std::map<std::string, float>		tag;
	};

  	void callbackOdomGt(const nav_msgs::OdometryConstPtr& msg);
	void callbackOdomUav(const nav_msgs::OdometryConstPtr& msg);
	void callbackTrackerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr msg, const std::string& topic);

	void collisionavoidance(std::string uav_name);
  	void activate(void);
	void goal(std::string uav_name, float x, float y, float z, float yaw);
	void callbackOtheruavcoordinates(const mrs_msgs::RtkGpsConstPtr msg, const std::string& topic);
	void callbackuwbranging(const gtec_msgs::RangingConstPtr msg, const std::string& topic);
	void callbacksonar(const sensor_msgs::RangeConstPtr msg, const std::string& topic);
	void callbackimudata(const sensor_msgs::ImuConstPtr msg, const std::string& topic);
	void callbackstatedata(const mavros_msgs::StateConstPtr msg, const std::string& topic);
	double dist3d(const double ax, const double ay, const double az, const double bx, const double by, const double bz);
	int neighbourtimer(void);
	void callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te);
	void callbackTimerUwbLocate(const ros::TimerEvent& te);
	void takeoff(int client_id, float height);
	void movement(void);
	//void uwblocate();
	//vector for subcription 
	std::vector<ros::Subscriber>                            sub_uav_rtk_gps;
  	std::vector<ros::Subscriber>                            sub_uav_diag;
  	std::vector<ros::Subscriber>                            sub_uav_sonar;
  	std::vector<ros::Subscriber>                            sub_uav_uwb_range;
  	std::vector<ros::Subscriber>                            sub_uav_imu;
  	std::vector<ros::Subscriber>                            sub_uav_state;
  	std::vector<ros::ServiceClient>                         arm_client;
  	std::vector<ros::ServiceClient>                         motor_client;
  	std::vector<ros::ServiceClient>                         land_client;
  	std::vector<ros::ServiceClient>                         set_mode_client;
  	std::vector<ros::ServiceClient>                         takeoff_client;

	//drones name
  	std::vector<std::string>                                other_drone_names_;
  	std::map<std::string, bool>  				other_drones_diagnostics;
	//drone localization
  	std::map<std::string, mrs_msgs::RtkGps> 		drones_gps_locate;
  	std::map<std::string, float>	 			drones_sonar_locate;
  	std::map<std::string, geometry_msgs::Point> 	        drones_imu_locate;
  	std::map<std::string, geometry_msgs::Point> 		drones_uwb_locate;
  	std::map<std::string, geometry_msgs::Point> 		drones_uwb_data;
  	std::map<std::string, struct locate> 	        	anchor;
  	std::map<std::string, geometry_msgs::Point> 	        drones_final_locate;
  	std::map<std::string, mavros_msgs::State> 	        current_state;

	std::string 						rtk_gps;
	std::string 						global;
	std::string						control_manager;
	std::string						mpc_tracker;
	std::string						diagnostics;
	std::string						tracker_diagnostics_in;
	//nav_msgs::Odometry					odom_gt_;
  	//nav_msgs::Odometry 					odom_uav_;
	std::string						_frame_id_;
	bool 							_simulation_;
	std::vector<ros::Publisher>				pub_reference_;
	std::vector<ros::Publisher>				pub_reference_mavros;
	ros::Timer 						timer_publish_dist_to_waypoint_;
	ros::Timer 						timer_publish_uwb_locate;
	std::map<std::string, mrs_msgs::ReferenceStamped>	new_waypoints;
	bool 							path_set=false;
	bool 							goal_set=false;
	mavros_msgs::CommandBool 				arm_request;
	mavros_msgs::CommandTOL 				srv_takeoff;
	mavros_msgs::SetMode 					srv_setMode;
	std_srvs::SetBool					motor_request;
	bool							got_imu_data=false;
	bool							got_sonar_data=false;
	bool							got_uwb_data=false;
  };
}




//init function
namespace localization
{	
	//ros::NodeHandle nh("~");
	//ros::NodeHandle nh = getMTPrivateNodeHandle();
	void uwb_start::onInit()
	{
	ros::NodeHandle nh = getMTPrivateNodeHandle();
	//load parameter rom launch file
	mrs_lib::ParamLoader param_loader(nh, "uwb_start");
	//param_loader.load_param("uav_name", _uav_name_);
	param_loader.loadParam("simulation", _simulation_);
	param_loader.loadParam("frame_id", _frame_id_);
	//param_loader.load_param("network/robot_names", other_drone_names_);
	//param_loader.loadParam("tracker_diagnostics_in",tracker_diagnostics_in);
	other_drone_names_ = {"uav1"};
	//subscrbing and publishing to respective topic 
	for (unsigned long i = 0; i < other_drone_names_.size(); i++) {
	//subscribe gps topic
	std::string prediction_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+"rtk_gps"+std::string("/")+"global";
	//sub_uav_rtk_gps.push_back(nh.subscribe <mrs_msgs::RtkGps> (prediction_topic_name, 10,boost::bind(&uwb_start::callbackOtheruavcoordinates, this, _1, prediction_topic_name)));
 	ROS_INFO("[uwb_start]: subscribing to %s", prediction_topic_name.c_str());
	//subscribe diagnostics topic
	std::string diag_topic_name = std::string("/") + other_drone_names_[i] + std::string("/") +"control_manager"+std::string("/")+"diagnostics";    
	sub_uav_diag.push_back(nh.subscribe <mrs_msgs::ControlManagerDiagnostics> (diag_topic_name, 10, boost::bind(&uwb_start::callbackTrackerDiag, this, _1, diag_topic_name)));
	ROS_INFO("[uwb_start]: subscribing to %s", diag_topic_name.c_str());				
	//advertise reference topic
	std::string neha_uav = "/"+other_drone_names_[i]+"/control_manager/position_cmd";
	pub_reference_.push_back(nh.advertise<mrs_msgs::ReferenceStamped>(neha_uav,1));
	ROS_INFO("[uwb_start]:publishing to %s",neha_uav.c_str());

	//advertise to px4 mavros
	std::string setpoint_topic_name = "/"+other_drone_names_[i]+"mavros/setpoint_position/local";
	pub_reference_mavros.push_back(nh.advertise<geometry_msgs::PoseStamped>(setpoint_topic_name,1));
	//subscribe sonar topic
	std::string sonar_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+"sensor"+std::string("/")+"sonar_front";
	sub_uav_sonar.push_back(nh.subscribe <sensor_msgs::Range> (sonar_topic_name, 1, boost::bind(&uwb_start::callbacksonar, this, _1, sonar_topic_name)));
 	ROS_INFO("[uwb_start]: subscribing to %s", sonar_topic_name.c_str());

	//suscribe imu topic  
	std::string imu_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+"mavros"+std::string("/")+"imu"+std::string("/")+"data_raw";
	sub_uav_imu.push_back(nh.subscribe <sensor_msgs::Imu> (imu_topic_name, 1, boost::bind(&uwb_start::callbackimudata, this, _1, imu_topic_name)));
 	ROS_INFO("[uwb_start]: subscribing to %s", imu_topic_name.c_str());

	//subscribe current state
	std::string state_topic_name = std::string("/")+other_drone_names_[i]+ "/mavros/state";
	sub_uav_state.push_back(nh.subscribe <mavros_msgs::State> (state_topic_name, 1, boost::bind(&uwb_start::callbackstatedata, this, _1, state_topic_name)));
 	ROS_INFO("[uwb_start]: subscribing to %s", state_topic_name.c_str());
	//service call
	std::string arv_service_name = std::string("/")+other_drone_names_[i]+ "/mavros/cmd/arming";
	arm_client.push_back(nh.serviceClient<mavros_msgs::CommandBool>(arv_service_name));
	std::string land_service_name = std::string("/")+other_drone_names_[i]+ "/mavros/cmd/land";
	land_client.push_back(nh.serviceClient<mavros_msgs::CommandTOL>(land_service_name));
	std::string mode_service_name = std::string("/")+other_drone_names_[i]+ "/mavros/set_mode";
	set_mode_client.push_back(nh.serviceClient<mavros_msgs::SetMode>(mode_service_name));
	std::string takeoff_service_name = std::string("/")+other_drone_names_[i]+ "/mavros/cmd/takeoff";
	takeoff_client.push_back(nh.serviceClient<mavros_msgs::CommandTOL>(takeoff_service_name));


	}
	//subscribe uwb topic  
	//std::string uwb_topic_name=std::string("/")+"gtec"+std::string("/")+"toa"+std::string("/")+"ranging";
		//sub_uav_uwb_range.push_back(nh.subscribe <gtec_msgs::Ranging> (uwb_topic_name, 1, boost::bind(&uwb_start::callbackuwbranging, this, _1, uwb_topic_name)));
	 	//ROS_INFO("[uwb_start]: subscribing to %s", uwb_topic_name.c_str());
	//subscrobe to pose topic 


//------------subsriber---------------
	//ros::Subscriber sub_odom_gt_=nh.subscribe("odom_gt_in",1,&uwb_start::callbackOdomGt,this,ros::TransportHints().tcpNoDelay());
	//ros::Subscriber sub_odom_uav_=nh.subscribe("odom_uav_in", 1, &uwb_start::callbackOdomUav, this, ros::TransportHints().tcpNoDelay());

        
//---------------------timer------------------

	//timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(30), &uwb_start::callbackTimerPublishDistToWaypoint, this);
	//timer_publish_uwb_locate = nh.createTimer(ros::Rate(10), &uwb_start::callbackTimerUwbLocate, this);
//------------------------service--------------

	




	
	ROS_INFO_ONCE("m here in init");
	//while((!got_sonar_data)||(!got_imu_data)||(!got_uwb_data)){}
		activate();	

	}



//activate fun this is fun to form a traingle of uav
void uwb_start::activate(void)
{	 
	std::cout << __FILE__ << ":" << __LINE__ << "activate function started "  <<std::endl; 
	while(ros::ok()){
	if((got_sonar_data) &&(got_imu_data)){
		int n=1;
		  std::cout << __FILE__ << ":" << __LINE__ << "activate function reached "  <<std::endl; 
			uwb_start::takeoff(0,2);
		  std::cout << __FILE__ << ":" << __LINE__ << "takeoff for uav1 complete sonar data" <<drones_sonar_locate["uav1"]<<"imu data is"<<drones_imu_locate["uav1"] <<std::endl; 
			std::cout << __FILE__ << ":" << __LINE__ << "z reading of final locate is "<<drones_final_locate["uav1"].z<<"reading from sonar is"<<drones_sonar_locate["uav1"]<<std::endl;
		  	//path_set=true;
			//uwb_start::goal("uav1",0,1,0,0);
			//while(!other_drones_diagnostics["uav2"]){}
			
	}

  }
	
}

void uwb_start::takeoff(int client_id, float height)
{
	std::cout << __FILE__ << ":" << __LINE__ << "i am at takeoff start "  <<std::endl; 
	

	  std::cout << __FILE__ << ":" << __LINE__ << "motor on"  <<std::endl; 
	//set mode
	std::string uav_name = "uav"+client_id;
	ros::Duration(5).sleep();
	srv_setMode.request.base_mode = 0;
	srv_setMode.request.custom_mode = "OFFBOARD";
	set_mode_client[client_id].call(srv_setMode);
		set_mode_client[client_id].call(srv_setMode);	
		if(set_mode_client[client_id].call(srv_setMode)){
		ROS_INFO("setmode send ok offboard enabled");
		}else{
		      ROS_ERROR("Failed SetMode");
		}
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
	

	//set arm
	ros::Duration(5).sleep();
	arm_request.request.value = true;
	arm_client[client_id].call(arm_request);

	  std::cout << __FILE__ << ":" << __LINE__ << "i am getting in while loop"  <<std::endl; 
		ros::Duration(.1).sleep();
		arm_client[client_id].call(arm_request);
		if(arm_request.response.success)
		{
	  std::cout << __FILE__ << ":" << __LINE__ << "i am getting in if loop"  <<std::endl; 
			ROS_INFO("Arming Successful");	
		}
	
	
	while(!other_drones_diagnostics[uav_name]){
        pub_reference_mavros[client_id].publish(pose);
	}
	  std::cout << __FILE__ << ":" << __LINE__ << "i am at takeoff end "  <<std::endl; 

}
/*
void uwb_start::callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te)
{
	if(!path_set)
	return;

	std::map<std::string, mrs_msgs::ReferenceStamped>::iterator itr=new_waypoints.begin();

	if(goal_set&&itr!=new_waypoints.end()){
	int l = *((itr->first).c_str()+3);
	pub_reference_[l].publish(itr->second);
	itr++;
	}
	else
	{
	itr=new_waypoints.begin();
	}
}



//goto function
void uwb_start::goal(std::string uav_name, float x, float y, float z, float yaw){
	mrs_msgs::ReferenceStamped new_waypoint;
	new_waypoint.header.frame_id = uav_name +"/"+ _frame_id_;
	ROS_INFO("hii neha its me %s",uav_name.c_str());	
	new_waypoint.header.stamp         = ros::Time::now();
	new_waypoint.reference.position.x = x;
	new_waypoint.reference.position.y = y;
	new_waypoint.reference.position.z = z;
	new_waypoint.reference.heading    = yaw;
	new_waypoints[uav_name]=new_waypoint;
	ROS_INFO("[uwb_start]: Flying to waypoint : x: %2.2f y: %2.2f z: %2.2f yaw: %2.2f uav name: %s",new_waypoint.reference.position.x, new_waypoint.reference.position.y, new_waypoint.reference.position.z, new_waypoint.reference.heading, uav_name.c_str() );
	goal_set =true;
}
*/
int sonar_count[3];
//callback function for sonar 
void uwb_start::callbacksonar(const sensor_msgs::RangeConstPtr msg, const std::string& topic){
  
  int uav_no = *(topic.c_str()+4); 
  uav_no = uav_no-48;
  std::string uav_name="uav"+std::to_string(uav_no);
  uav_no--;
  sonar_count[uav_no] = 1;
  got_sonar_data=true;
  drones_sonar_locate[uav_name] = msg->range;
  drones_final_locate[uav_name].z=msg->range;
  if(uav_name == "uav1")
  std::cout << __FILE__ << ":" << __LINE__  << "callback sonar data uav name is " << uav_name <<"and range is "<<msg->range<<"data seen as sonar is"<<drones_sonar_locate["uav1"]<<"data seen as final locate is"<<drones_final_locate["uav1"].z<<std::endl; 
}

//callback function for imu sensor
// see this algo 
int c;
double x_0 =0,y_0 =0,z_0 =0;
int imu_count[3];
void uwb_start::callbackimudata(const sensor_msgs::ImuConstPtr msg, const std::string& topic){
  double t;
  std::cout << __FILE__ << ":" << __LINE__  << "[uwb_start]: m here in callbackimudata x_0 is " << x_0 <<" y_0 is "<<y_0<<"z_0 is "<<z_0<<std::endl; 
  int uav_no = *(topic.c_str()+4); 
  uav_no = uav_no-48;
  std::string uav_name="uav"+std::to_string(uav_no);
  uav_no--;
  imu_count[uav_no] = 1;
  got_imu_data=true;
  double dt = (msg->header.stamp.sec + (msg->header.stamp.nsec/pow(10,9))) - t ;
  t = msg->header.stamp.sec + (msg->header.stamp.nsec/pow(10,9));
  //std::cout << __FILE__ << ":" << __LINE__  << "time is " << t <<"time difference is "<<dt<<std::endl;
	if(c>0 && dt<1)  
	{ double x = x_0 + 0.5*pow(dt,2)*(msg->linear_acceleration.x);
	  double y = y_0 + 0.5*pow(dt,2)*(msg->linear_acceleration.y);
	  double z = z_0 + 0.5*pow(dt,2)*(msg->linear_acceleration.z-9.7);
          //std::cout << __FILE__ << ":" << __LINE__  << "difference in x is " << 0.5*pow(dt,2)*(msg->linear_acceleration.x) <<"difference in y is "<<0.5*pow(dt,2)*(msg->linear_acceleration.y)<<"difference in z is "<<0.5*pow(dt,2)*(msg->linear_acceleration.z-9.7)<<std::endl;
	  x_0 = x;
	  y_0 = y;
	  z_0 = z;
	  geometry_msgs::Point X;
	  X.x=x;
	  X.y=y;
	  X.z=z;
	  drones_imu_locate[uav_name]=X;
	  drones_final_locate[uav_name]=X;
	  //std::cout << __FILE__ << ":" << __LINE__  << "callback imu data uav name is " << uav_name <<"and data is "<<X<<std::endl; 
	}
c++;
}

void uwb_start::callbackstatedata(const mavros_msgs::StateConstPtr msg, const std::string& topic){

int uav_no = *(topic.c_str()+4); 
  uav_no = uav_no-48;
  std::string uav_name="uav"+std::to_string(uav_no);
current_state[uav_name] = *msg;
}
void uwb_start::callbackTrackerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr msg, const std::string& topic){
  ROS_INFO_ONCE("m here in callbackTrackerDiag");
  int uav_no = *(topic.c_str()+4); 
  uav_no = uav_no-48;
  std::string uav_name="uav"+std::to_string(uav_no);

  other_drones_diagnostics[uav_name] = msg->tracker_status.have_goal;  
  if (!msg->tracker_status.have_goal){
  //std::cout << __FILE__ << ":" << __LINE__ << uav_name << "waypoint reached "  <<std::endl; 
  }
}

}
PLUGINLIB_EXPORT_CLASS(localization::uwb_start, nodelet::Nodelet);
