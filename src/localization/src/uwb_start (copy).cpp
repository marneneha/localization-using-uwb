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

	struct locate{
	bool localization_enable;
	geometry_msgs::Point anchor[4];
	float dist[3];
	};

  	void callbackOdomGt(const nav_msgs::OdometryConstPtr& msg);
	void callbackOdomUav(const nav_msgs::OdometryConstPtr& msg);
	void callbackTrackerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);

	void collisionavoidance(std::string uav_name);
  	void activate(void);
	void neha_goto(float x,float y, std::string uav_name);
	void callbackOtheruavcoordinates(const mrs_msgs::RtkGpsConstPtr msg, const std::string& topic);
	void callbackuwbranging(const gtec_msgs::RangingConstPtr msg, const std::string& topic);
	//void callbacksonar(const sensor_msgs::RangeConstPtr msg, const std::string& topic);
	void callbackimudata(const sensor_msgs::ImuConstPtr msg, const std::string& topic);
	double dist3d(const double ax, const double ay, const double az, const double bx, const double by, const double bz);
	int neighbourtimer(void);
	//void callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te);
	void callbackTimerUwbLocate(const ros::TimerEvent& te);
	void uwblocate(locate* tag_pose,std::string* uav);
	std::vector<ros::Subscriber>                            other_uav_coordinates;
  	std::vector<ros::Subscriber>                            other_uav_diag_subscribers;
  	//std::vector<ros::Subscriber>                            other_uav_sonar;
  	std::vector<ros::Subscriber>                            other_uav_uwb_range;
  	std::vector<ros::Subscriber>                            other_uav_imu;
  	std::vector<std::string>                                other_drone_names_;
  	std::map<std::string, mrs_msgs::RtkGps> 		other_drones_location;
  	//std::map<std::string, float>	 			other_drones_sonar_locate;
  	std::map<std::string, geometry_msgs::Point> 	        other_drones_imu_locate;
  	std::map<std::string, geometry_msgs::Point> 		other_drones_uwb_locate;
  	std::map<std::string, struct locate> 	        	tag;
  	std::map<std::string, sensor_msgs::Imu> 		other_drones_imu;
	std::map<std::string, int>				other_drones_neighbour;
	std::map<std::string, int>				other_drones_preneighbour;
  	std::map<std::string, mrs_msgs::MpcTrackerDiagnostics>  other_drones_diagnostics;
	std::string 						rtk_gps;
	std::string 						global;
	std::string						control_manager;
	std::string						mpc_tracker;
	std::string						diagnostics;
	std::string						tracker_diagnostics_in;
	//nav_msgs::Odometry					odom_gt_;
  	//nav_msgs::Odometry 					odom_uav_;
	std::string _frame_id_;
	bool _simulation_;
	std::vector<ros::Publisher>				pub_reference_;
	ros::Timer 						timer_publish_dist_to_waypoint_;
	ros::Timer 						timer_publish_uwb_locate;
	std::vector<mrs_msgs::ReferenceStamped>			new_waypoints;
	bool path_set=false;
  };
}




//init function
namespace localization
{	ros::NodeHandle nh("~");
	void uwb_start::onInit()
	{
	//load parameter rom launch file
	mrs_lib::ParamLoader param_loader(nh, "uwb_start");
	//param_loader.load_param("uav_name", _uav_name_);
	param_loader.loadParam("simulation", _simulation_);
	param_loader.loadParam("frame_id", _frame_id_);
	//param_loader.load_param("network/robot_names", other_drone_names_);
	param_loader.loadParam("tracker_diagnostics_in",tracker_diagnostics_in);
	other_drone_names_ = {"uav1", "uav2", "uav3", "uav4", "uav5", "uav6","uav7", "uav8", "uav9"};
	//subscrbing and publishing to respective topic 
	for (unsigned long i = 0; i < other_drone_names_.size(); i++) {
	//subscribe gps topic
	std::string prediction_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+"rtk_gps"+std::string("/")+"global";
	other_uav_coordinates.push_back(nh.subscribe <mrs_msgs::RtkGps> (prediction_topic_name, 10, 						boost::bind(&uwb_start::callbackOtheruavcoordinates, this, _1, prediction_topic_name)));
 	ROS_INFO("[uwb_start]: subscribing to %s", prediction_topic_name.c_str());
	//subscribe diagnostics topic
	std::string diag_topic_name = std::string("/") + other_drone_names_[i] + std::string("/") +"control_manager"+std::string("/")	+"mpc_tracker"+std::string("/")+"diagnostics";    
	other_uav_diag_subscribers.push_back(nh.subscribe(diag_topic_name, 1, &uwb_start::callbackTrackerDiag, this, ros::TransportHints().tcpNoDelay()));
	ROS_INFO("[uwb_start]: subscribing to %s", diag_topic_name.c_str());				
	//advertise reference topic
	std::string neha_uav = "/"+other_drone_names_[i]+"/control_manager/reference";
	pub_reference_.push_back(nh.advertise<mrs_msgs::ReferenceStamped>(neha_uav,1));
	ROS_INFO("[uwb_start]:publishing to %s",neha_uav.c_str());
	//subscribe sonar topic
	//std::string sonar_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+"sonar";
	//other_uav_sonar.push_back(nh.subscribe <sensor_msgs::Range> (sonar_topic_name, 10, boost::bind(&uwb_start::callbacksonar, this, _1, sonar_topic_name)));
 	//ROS_INFO("[uwb_start]: subscribing to %s", sonar_topic_name.c_str());

	//suscribe imu topic  
	std::string imu_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+"mavros"+std::string("/")+"imu"+std::string("/")+"data_raw"/*or data*/;
	other_uav_imu.push_back(nh.subscribe <sensor_msgs::Imu> (imu_topic_name, 10, boost::bind(&uwb_start::callbackimudata, this, _1, imu_topic_name)));
 	ROS_INFO("[uwb_start]: subscribing to %s", imu_topic_name.c_str());
	}
	//subscribe uwb topic  
	std::string uwb_topic_name=std::string("/")+"gtec"+std::string("/")+"toa"+std::string("/")+"ranging";
		other_uav_uwb_range.push_back(nh.subscribe <gtec_msgs::Ranging> (uwb_topic_name, 10, boost::bind(&uwb_start::callbackuwbranging, this, _1, uwb_topic_name)));
	 	ROS_INFO("[uwb_start]: subscribing to %s", uwb_topic_name.c_str());

//------------subsriber---------------
	//ros::Subscriber sub_odom_gt_=nh.subscribe("odom_gt_in",1,&uwb_start::callbackOdomGt,this,ros::TransportHints().tcpNoDelay());
	//ros::Subscriber sub_odom_uav_=nh.subscribe("odom_uav_in", 1, &uwb_start::callbackOdomUav, this, ros::TransportHints().tcpNoDelay());

        
//---------------------timer------------------

//timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(20), &uwb_start::callbackTimerPublishDistToWaypoint, this);
timer_publish_uwb_locate = nh.createTimer(ros::Rate(20), &uwb_start::callbackTimerUwbLocate, this);
//------------------------publisher--------------

	
ROS_INFO_ONCE("m here in init");
		//activate();	

		ros::spin();
	}



//activate fun this is fun to form a traingle of uav
void uwb_start::activate(void)
{
int x=0,y=0,i=0,j=0,N=3;
float z=3,yaw=3.14;
int n = (sqrt(8*N+1)-1)/2;
int k=0;
mrs_msgs::ReferenceStamped new_waypoint;
  for (i=0;i<n;i++){
    for (j=0;j<=i;j++){
	std::string uav_name=other_drone_names_[k];

	float x=10*(-1*0.2*i+0.8*j);
	float y=10*(-1*0.8*i);

	float z=5,yaw=0.23;

	{
	new_waypoint.header.frame_id = uav_name +"/"+ _frame_id_;
	ROS_INFO("hii neha its me %s",uav_name.c_str());	
	new_waypoint.header.stamp    = ros::Time::now();
	new_waypoint.reference.position.x = x;
	new_waypoint.reference.position.y = y;
	new_waypoint.reference.position.z = z;
	new_waypoint.reference.heading    = yaw;
	new_waypoints.push_back(new_waypoint);
	}

	  ROS_INFO("[uwb_start]: Flying to waypoint : x: %2.2f y: %2.2f z: %2.2f yaw: %2.2f uav name: %s",new_waypoints[k].reference.position.x, new_waypoints[k].reference.position.y, new_waypoints[k].reference.position.z, new_waypoints[k].reference.heading, uav_name.c_str() );
       k++;
	//ROS_INFO("m here in activate1 and value are n:%d,i:%d,j:%d ",n ,i, j);
    }
  }
	path_set=true;
	ROS_INFO("path has been set");
}
int l=0;
/*void uwb_start::callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te)
{
	
	if(path_set&&l<=2){
	pub_reference_[l].publish(new_waypoints[l]);
	l++;
	}
	else
	l=0;
}*/



//goto function

//localization algo 

//clear
//or mrs_msgs::RtkGps::ConstPtr&

void uwb_start::callbackTimerUwbLocate(const ros::TimerEvent& te)
{
ROS_INFO("[uwb_start]: m here in callbackTimerUwbLocate");
static int i=0;
  std::string uav_name="uav"+i;
  if(tag[uav_name].localization_enable){ 
    struct locate *ptr;
	ROS_INFO("[uwb_start]: m here in callbackTimerUwbLocate if with l as %d",i);
    ptr = &tag[uav_name];
    uwb_start::uwblocate(ptr, &uav_name);
      if(i==8)
      i=0;
  }
i++;
}


void uwb_start::callbackOtheruavcoordinates(const mrs_msgs::RtkGpsConstPtr msg, const std::string& topic){
  ROS_INFO("[uwb_start]: m here in callbackTimerUwbLocate");
  int uav_no = *(topic.c_str()+3); 
  //std::string uav_name="uav"+uav_name-1;
  std::string uav_name="uav"+uav_no;
  other_drones_location[uav_name]=*msg;
}

//callback function for sonar 
/*void uwb_start::callbacksonar(const sensor_msgs::RangeConstPtr msg, const std::string& topic){
  int uav_no = *(topic.c_str()+3); 
  std::string uav_name="uav"+uav_no;
  //std::string uav_name="uav";
  other_drones_sonar_locate[uav_name] = msg->range;
  tag[uav_name].anchor[0].z = msg->range;
}*/

//callback function for imu sensor
void uwb_start::callbackimudata(const sensor_msgs::ImuConstPtr msg, const std::string& topic){
  ROS_INFO("[uwb_start]: m here in callbackimudata");
  int uav_no = *(topic.c_str()+3); 
  std::string uav_name="uav"+uav_no;
  //std::string uav_name="uav";
  int x_0,y_0,z_0;
  int t= msg->header.stamp.nsec;
  int x = x_0 + 0.5*pow(t,2)*(msg->linear_acceleration.x);
  int y = y_0 + 0.5*pow(t,2)*(msg->linear_acceleration.y);
  int z = z_0 + 0.5*pow(t,2)*(msg->linear_acceleration.z);
  //float cov[9] = { msg->linear_acceleration_covariance };
  x_0 = x;
  y_0 = y;
  z_0 = z;
  ROS_INFO("[uwb_start]: imu location x_0= %d y_0= %d z_0= %d",x_0 ,y_0 ,z_0);
  geometry_msgs::Point X;
  X.x=x;
  X.y=y;
  X.z=z;
  other_drones_imu_locate[uav_name]=X;
}

//callback function for uwb sensor
void uwb_start::callbackuwbranging(const gtec_msgs::RangingConstPtr msg, const std::string& topic){
  ROS_INFO("[uwb_start]: m here in callbackuwbranging");
  //int uav_no = msg->tagId; 
  //std::string uav_name="uav"+uav_no;
  //std::string uav_name="uav";
  //put tagid and anchorid as uav name
  static int i=0;
//see this condition properly 
  if(!tag[msg->tagId].anchor[i].x){
  ROS_INFO("[uwb_start]: m here in callbackuwbranging anchorid is%s rage is %d",msg->anchorId.c_str() ,msg->range);
    tag[msg->tagId].anchor[i] = other_drones_uwb_locate[msg->anchorId];
    tag[msg->tagId].dist[i] = msg->range;
    i++;
    if(i==2){
      i=0;
      tag[msg->tagId].localization_enable = true;
      }
    }
}
void uwb_start::uwblocate(locate* tag_pose, std::string* uav){
  ROS_INFO("[uwb_start]: m here in uwblocate");
Eigen::MatrixXd A (2,2);
A<< (tag_pose->anchor[2].x-tag_pose->anchor[1].x), (tag_pose->anchor[2].y-tag_pose->anchor[1].y), (tag_pose->anchor[3].x-tag_pose->anchor[2].x), (tag_pose->anchor[3].y-tag_pose->anchor[2].y);

Eigen::MatrixXd B(2,1);
B<< ((pow(tag_pose->dist[0],2))-(pow((tag_pose->anchor[0].z-tag_pose->anchor[1].z),2))-(pow(tag_pose->anchor[1].x,2))-(pow(tag_pose->anchor[1].y,2)))-((pow(tag_pose->dist[1],2))-(pow((tag_pose->anchor[0].z-tag_pose->anchor[2].z),2))-(pow(tag_pose->anchor[2].x,2))-(pow(tag_pose->anchor[2].y,2))),
((pow(tag_pose->dist[1],2))-(pow((tag_pose->anchor[0].z-tag_pose->anchor[2].z),2))-(pow(tag_pose->anchor[2].x,2))-(pow(tag_pose->anchor[2].y,2)))-((pow(tag_pose->dist[2],2))-(pow((tag_pose->anchor[0].z-tag_pose->anchor[3].z),2))-(pow(tag_pose->anchor[3].x,2))-(pow(tag_pose->anchor[3].y,2)));

geometry_msgs::Point X;

X.x = (A.inverse()*B)(0);
X.y = (A.inverse()*B)(1);
X.z = tag_pose->anchor[0].z;
other_drones_uwb_locate[*uav] = X;
}






//more time lag
//check no. of neighbour
//unnecesarry function
/*void uwb_start::callbackOdomGt(const nav_msgs::OdometryConstPtr& msg){
  ROS_INFO_ONCE("m here in callbackOdomGt");
  odom_gt_ = *msg;		
}
void uwb_start::callbackOdomUav(const nav_msgs::OdometryConstPtr& msg){
  ROS_INFO_ONCE("m here in callbackOdomUav");
  odom_uav_ = *msg;		
  }
*/
void uwb_start::callbackTrackerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg){
  ROS_INFO_ONCE("m here in callbackTrackerDiag");	
  //other_drones_diagnostics[msg->uav_name] = *msg;  
  if (!msg->tracker_status.have_goal){
    //ROS_INFO("[uwb_start]: %s Waypoint reached." ,other_drones_diagnostics[msg->uav_name]);
  }
}

double dist3d(const double ax, const double ay, const double az, const double bx, const double by, const double bz) {

  return sqrt(pow(ax - bx, 2) + pow(ay - by, 2) + pow(az - bz, 2));
}
}
PLUGINLIB_EXPORT_CLASS(localization::uwb_start, nodelet::Nodelet);
