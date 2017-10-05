#include "lab2pkg/lab2.h" 
#define PI 3.14159265359
#define SPIN_RATE 20  /* Hz */

//arrays defining Waypoints
double home[]={120*PI/180,-90*PI/180,90*PI/180,-90*PI/180,-90*PI/180,0*PI/180};

double arr11[]={131.03*PI/180,-71.62*PI/180,117.02*PI/180,-132.05*PI/180,-90.42*PI/180,0*PI/180};
double arr12[]={143.47*PI/180,-53.17*PI/180,87.09*PI/180,-123.08*PI/180,-90.05*PI/180,0*PI/180};
double arr13[]={158.28*PI/180,-73.63*PI/180,122.96*PI/180,-140.65*PI/180,-88.82*PI/180,0*PI/180};
double arr14[]={131.06*PI/180,-65.39*PI/180,120.81*PI/180,-144.33*PI/180,-90.6*PI/180,0*PI/180};
double arr15[]={143.8*PI/180,-48.38*PI/180,88.64*PI/180,-130.35*PI/180,-89.47*PI/180,0*PI/180};
double arr16[]={158.43*PI/180,-66.47*PI/180,124.49*PI/180,-148.52*PI/180,-88.79*PI/180,0*PI/180};
double arr17[]={130.57*PI/180,-57.56*PI/180,123.68*PI/180,-156.1*PI/180,-89.88*PI/180,0*PI/180};
double arr18[]={143.5*PI/180,-43.43*PI/180,90.47*PI/180,-137.07*PI/180,-89.4*PI/180,0*PI/180};
double arr19[]={158.07*PI/180,-59.12*PI/180,122.18*PI/180,-147.99*PI/180,-88.81*PI/180,0*PI/180};

double arr21[]={130.47*PI/180,-73.99*PI/180,117.35*PI/180,-134.47*PI/180,-89.69*PI/180,0*PI/180};
double arr22[]={143.11*PI/180,-55.27*PI/180,86.78*PI/180,-121*PI/180,-87.34*PI/180,0*PI/180};
double arr23[]={157.78*PI/180,-76.33*PI/180,122.52*PI/180,-137.93*PI/180,-88.86*PI/180,0*PI/180};
double arr24[]={130.89*PI/180,-67.24*PI/180,118.35*PI/180,-140.1*PI/180,-90.5*PI/180,0*PI/180};
double arr25[]={144.03*PI/180,-50.97*PI/180,86.93*PI/180,-124.34*PI/180,-89.92*PI/180,0*PI/180};
double arr26[]={158.45*PI/180,-69.35*PI/180,122.71*PI/180,-142.95*PI/180,-88.77*PI/180,0*PI/180};
double arr27[]={130.44*PI/180,-60.19*PI/180,121.56*PI/180,-151.29*PI/180,-89.73*PI/180,0*PI/180};
double arr28[]={143.61*PI/180,-45.45*PI/180,86.94*PI/180,-127.88*PI/180,-89.41*PI/180,0*PI/180};
double arr29[]={158.24*PI/180,-61.47*PI/180,122.38*PI/180,-147.89*PI/180,-88.81*PI/180,0*PI/180};

// array to define final velocity of point to point moves.  For now slow down to zero once 
// each point is reached
double arrv[]={0,0,0,0,0,0};

//vectors to be used to publish commands to UR3 ROS Driver (ece470_ur3_driver)
std::vector<double> QH (home,home+sizeof(home) / sizeof(home[0]));

std::vector<double> Q11 (arr11,arr11+sizeof(arr11) / sizeof(arr11[0]));
std::vector<double> Q12 (arr12,arr12+sizeof(arr12) / sizeof(arr12[0]));
std::vector<double> Q13 (arr13,arr13+sizeof(arr13) / sizeof(arr13[0]));
std::vector<double> Q14 (arr14,arr14+sizeof(arr14) / sizeof(arr14[0]));
std::vector<double> Q15 (arr15,arr15+sizeof(arr15) / sizeof(arr15[0]));
std::vector<double> Q16 (arr16,arr16+sizeof(arr16) / sizeof(arr16[0]));
std::vector<double> Q17 (arr17,arr17+sizeof(arr17) / sizeof(arr17[0]));
std::vector<double> Q18 (arr18,arr18+sizeof(arr18) / sizeof(arr18[0]));
std::vector<double> Q19 (arr19,arr19+sizeof(arr19) / sizeof(arr19[0]));

std::vector<double> Q21 (arr21,arr21+sizeof(arr21) / sizeof(arr21[0]));
std::vector<double> Q22 (arr22,arr22+sizeof(arr22) / sizeof(arr22[0]));
std::vector<double> Q23 (arr23,arr23+sizeof(arr23) / sizeof(arr23[0]));
std::vector<double> Q24 (arr24,arr24+sizeof(arr24) / sizeof(arr24[0]));
std::vector<double> Q25 (arr25,arr25+sizeof(arr25) / sizeof(arr25[0]));
std::vector<double> Q26 (arr26,arr26+sizeof(arr26) / sizeof(arr26[0]));
std::vector<double> Q27 (arr27,arr27+sizeof(arr27) / sizeof(arr27[0]));
std::vector<double> Q28 (arr28,arr28+sizeof(arr28) / sizeof(arr28[0]));
std::vector<double> Q29 (arr29,arr29+sizeof(arr29) / sizeof(arr29[0]));

std::vector<double> v (arrv,arrv+sizeof(arrv) / sizeof(arrv[0]));

// creating an array of these vectors allows us to iterate through them
// and programatically choose where to go.
std::vector<double> Q1 [4][3] = {
	{Q21, Q22, Q23},
  {Q11, Q12, Q13},
  {Q14, Q15, Q16},
  {Q17, Q18, Q19}
};

int stack[3]={0,0,0};
ece470_ur3_driver::command driver_msg;

// Global bool variables that are assigned in the callback associated when subscribed 
// to the "ur3/position" and ur_driver/io_states topic
bool isReady=1;
bool pending=0;
bool suction=0;

// Whenever ur3/position publishes info this callback function is run.
void position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady; // When isReady is True the robot arm has made it to its desired position
						  // and is ready to be told to go to another point if desired.
	pending=msg->pending; // pending is the opposite of isReady, pending is true until a new position is reached
//	ROS_INFO("Debug isRdy = %d, pending = %d",(int)isReady,(int)pending);
}

// Whenever ur_driver/io_states publishes info this callback function is run
void io_callback(const ur_msgs::IOStates::ConstPtr& msg)
{
	suction=msg->digital_in_states[0].state;
}


int move_arm(	ros::Publisher pub_command , ros::Rate loop_rate, std::vector<double> dest, float duration)
{
    ROS_INFO("sending Goals");
		driver_msg.destination=dest;
												  // the subscriber will not receive this message.
	    driver_msg.duration = duration;
		pub_command.publish(driver_msg);  // publish command, but note that is possible that

		int spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
		
    int error = 0;
    return error;
}

int move_block(ros::Publisher pub_command ,
                ros::Rate loop_rate,float duration,
                std::vector<double> Q)
{
    move_arm(pub_command,loop_rate,Q,duration);
    int error = 0;
    return error;
}

int move(ros::Publisher pub_command ,
                ros::Rate loop_rate,
                ros::ServiceClient srv_SetIO,
                ur_msgs::SetIO srv,int start_pos, int end_pos)
{
	float duration=1;
	if (stack[start_pos]==0)
	{
		return -1;//no block at start position
	}
	move_block(pub_command, loop_rate, duration, Q1[0][start_pos]);
	move_block(pub_command, loop_rate, duration,Q1[4-stack[start_pos]][start_pos]);
	srv.request.fun = 1;
	srv.request.pin = 0;  //Digital Output 0
	srv.request.state = 1.0; //Set DO0 on
	if (srv_SetIO.call(srv)) 
	{
		ROS_INFO("True: Switched Suction ON");
	} else 
		{
			ROS_INFO("False");
		}
	
	// Wait one second before checking suction
	int spincount = 0;
	while (spincount < 20)
	{
		ros::spinOnce();
		loop_rate.sleep();
		spincount++;
	}
	
	// Check for suction
	if (suction == 0)
	{
		return -1;
	}
	
	stack[start_pos]=stack[start_pos]-1;
	stack[end_pos]=stack[end_pos]+1;

	move_block(pub_command, loop_rate, duration,Q1[0][start_pos]);
	move_block(pub_command, loop_rate, duration,Q1[0][end_pos]);
	move_block(pub_command, loop_rate, duration,Q1[4-stack[end_pos]][end_pos]);
	srv.request.fun = 1;
	srv.request.pin = 0; // Digital Output 0
	srv.request.state = 0.0; //Set DO0 off
	if (srv_SetIO.call(srv)) 
	{
		ROS_INFO("True: Switched Suction OFF");
	} else 
		{
			ROS_INFO("False");
		}
	move_block(pub_command, loop_rate, duration, Q1[0][end_pos]);
	return 0;

}


int move_tower(ros::Publisher pub_command ,
                ros::Rate loop_rate,
                ros::ServiceClient srv_SetIO,
                ur_msgs::SetIO srv,
                int block,
                int start,
                int end,
                int mid)
{
	int ret=0;
		if (block == 0) 
		{
			ret=move(pub_command ,loop_rate, srv_SetIO,srv, start, end);
			return ret;
    } else 
    	{
    		ret=move_tower(pub_command ,loop_rate, srv_SetIO,srv,block - 1, start, mid, end);
    		if (ret==-1)
    		{
    			return ret;
    		}
    		ret=move(pub_command ,loop_rate, srv_SetIO,srv, start, end)	;
    		if (ret==-1)
    		{
    			return ret;
    		}
    		ret=move_tower(pub_command ,loop_rate, srv_SetIO,srv,block - 1, mid, end, start);
    	  if (ret==-1)
    		{
    			return ret;
    		}
    		return 0;
    	}
}  


int main(int argc, char **argv)
{
  int inputdone=0;
	int start_loc = -1;
	int end_loc = -1;
	int mid_loc=-1;
//initialization & variable definition
	ros::init(argc, argv, "lab2node");	//initialzation of ros required for each Node.
	ros::NodeHandle nh;				//handler for this node.
	
	//initialized publisher ur3/command, buffer size of 10.
	ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
	// initialize subscriber to ur3/position and call function position_callback each time data is published
	ros::Subscriber sub_position=nh.subscribe("ur3/position",1,position_callback);
	ros::Subscriber sub_io=nh.subscribe("ur_driver/io_states",1,io_callback);
	
	ros::ServiceClient srv_SetIO = nh.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
	ur_msgs::SetIO srv;



	std::string inputString;
	while (!inputdone) {
		std::cout << "Enter start location <Either 0 1 or 2>"<< std::endl;
		std::getline(std::cin, inputString);
		std::cout << "You entered " << inputString << "\n";
		if (inputString == "1") {
			inputdone = 1;
			start_loc = 1;
		} else if (inputString == "2") {
			inputdone = 1;
			start_loc = 2;
		} else if (inputString == "0") {
			inputdone = 1;
			start_loc = 0;
		} else {
			std::cout << "Please just enter the character 0 1 or 2\n\n"<< std::endl;
		}
	}
		std::string s_loc_String=inputString;
inputString="";
    inputdone=0;

	stack[start_loc]=3;
///////We may have to pause for a second here
	while (!inputdone) {
		std::cout << "Enter end location <Either 0 1 or 2>";
		std::getline(std::cin, inputString);
		std::cout << "You entered " << inputString << "\n";
		if (inputString==s_loc_String)
			{std::cout << "Please use a value which is different from you start location.\n\n"<< std::endl;}
		else{
		if (inputString == "1") {
			inputdone = 1;
			end_loc = 1;
		} else if (inputString == "2") {
			inputdone = 1;
			end_loc = 2;
		} else if (inputString == "0") {
			inputdone = 1;
			end_loc = 0;
		} else {
			std::cout << "Please just enter the character 0 1 or 2\n\n"<< std::endl;
		}
	}
	}
	for (int qq=0; qq<3; qq++)
	{
	if (start_loc!=qq && end_loc!=qq)
		{mid_loc=qq;}
	}
	
	while(!ros::ok()){};
		
	ROS_INFO("Beginning Task");

	ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int results=move_tower(pub_command ,loop_rate,srv_SetIO,srv,2,start_loc,end_loc,mid_loc);
 	if (results==-1)
 	{
 		std::cout << "ERROR WHILE EXECUTING\n\n"<< std::endl;
 		srv.request.fun = 1;
		srv.request.pin = 0; // Digital Output 0
		srv.request.state = 0.0; //Set DO0 off
		if (srv_SetIO.call(srv)) 
		{
			ROS_INFO("True: Switched Suction OFF");
		} else 
			{
				ROS_INFO("False");
			}
	}
 	else
 	{std::cout << "Task complete"<< std::endl;}
	
	move_arm(pub_command,loop_rate,QH,1);
	return 0;
}
