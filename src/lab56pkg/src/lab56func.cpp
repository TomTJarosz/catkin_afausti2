#include "lab56pkg/lab56.h"
#include <iostream>
extern ImageConverter* ic_ptr; //global pointer from the lab56.cpp

#define SPIN_RATE 20  /* Hz */

bool isReady=1;
bool pending=0;

float SuctionValue = 0.0;
float xw = 0.0;
float yw = 0.0;

bool leftclickdone = 1;
bool rightclickdone = 1;

std::vector<int> r_cent;
std::vector<int> c_cent;
/*****************************************************
* Functions in class:
* **************************************************/	

//constructor(don't modify) 
ImageConverter::ImageConverter():it_(nh_)
{
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera_node/image_raw", 1, 
    	&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    namedWindow(OPENCV_WINDOW);   
    pub_command=nh_.advertise<ece470_ur3_driver::command>("ur3/command",10);
    sub_position=nh_.subscribe("ur3/position",1,&ImageConverter::position_callback,this); 

	sub_io_states=nh_.subscribe("ur_driver/io_states",1,&ImageConverter::suction_callback,this);
	
	srv_SetIO = nh_.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");


    driver_msg.destination=lab_invk(-0.3,-0.3,0.2,-45.0);

	//publish the point to the robot
    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
	driver_msg.duration = 3.0;
	pub_command.publish(driver_msg);  // publish command, but note that is possible that
										  // the subscriber will not receive this message.
	spincount = 0;
	while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
		ros::spinOnce();  // Allow other ROS functionallity to run
		loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
		if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
			pub_command.publish(driver_msg);
			ROS_INFO_STREAM("Just Published again driver_msg");
			spincount = 0;
		}
		spincount++;  // keep track of loop count
	}
	ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
	
	while(!isReady)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO_STREAM("Ready for new point");

}

//destructor(don't modify)
ImageConverter::~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady;
	pending=msg->pending;
}

void ImageConverter::suction_callback(const ur_msgs::IOStates::ConstPtr& msg)
{
	SuctionValue = msg->analog_in_states[0].state;
}


//subscriber callback function, will be called when there is a new image read by camera
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{  
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    } 
    // create an gray scale version of image
    Mat gray_image;
	cvtColor( cv_ptr->image, gray_image, CV_BGR2GRAY );  
    // convert to black and white img, then associate objects:  

// FUNCTION you will be completing
    Mat bw_image = thresholdImage(gray_image); // bw image from own function

// FUNCTION you will be completing
    Mat associate_image = associateObjects(bw_image); // find associated objects

    // Update GUI Window
    imshow("Image window", cv_ptr->image);
    imshow("gray_scale", gray_image);
    imshow("black and white", bw_image);
    imshow("associate objects", associate_image);
    waitKey(3);
    // Output some video stream
    image_pub_.publish(cv_ptr->toImageMsg());
} 

/*****************************************************
	 * Function for Lab 5
* **************************************************/	
// Take a grayscale image as input and return an thresholded image.
// You will implement your algorithm for calculating threshold here.
Mat ImageConverter::thresholdImage(Mat gray_img)
{
	int   totalpixels;
	Mat bw_img  = gray_img.clone(); // copy input image to a new image
	totalpixels	  = gray_img.rows*gray_img.cols;			// total number of pixels in image
	uchar graylevel; // use this variable to read the value of a pixel
	int zt=0; // threshold grayscale value 
	int H[256];
	int index;
	float p=0,q0=0,q1=0,mean=0,mean1=0,mean2=0,varb=0;
	float eps=.0000001;
	// Initialize histogram
	for(int i=0; i<256; i++)
	{
		H[i] = 0;
	}
						
	// Populate histogram
	for(int r=0; r<gray_img.rows; r++)
	{
		for(int c=0; c<gray_img.cols; c++)
		{
			graylevel = gray_img.data[r*c];
			index = (int)graylevel;
			H[index] += 1; 
		}
	}

	for(int i=0; i<256; i++)
	{
		mean += float(H[i]*i)/float(totalpixels);		
	}

		
	float newq0=0;
	float bestvarb=0.;
	zt = 0;  // you will be finding this automatically 

	for(int i=0; i<256; i++)
	{
		p = float(H[i])/float(totalpixels);
		newq0 = q0+p;
		mean1 = (float(i*p)/float(eps+newq0))+(float(mean1*q0)/float(eps+newq0));
		mean2 = float(mean-(newq0*mean1))/float(eps+1-newq0);
		q0 = newq0;
		varb = q0*float(1.0-q0)*float(mean1-mean2)*float(mean1-mean2);
		if (varb>bestvarb)
		{
			zt = i;
			bestvarb = varb;
		}
			
	}

	std::cout<<"Threshold is "<<zt<<std::endl;
	// threshold the image
	for(int i=0; i<totalpixels; i++)
	{
		graylevel = gray_img.data[i];	
		if(graylevel>zt) bw_img.data[i]= 255; // set rgb to 255 (white)
		else             bw_img.data[i]= 0; // set rgb to 0   (black)
	}	
	return bw_img;	
}
/*****************************************************
	 * Function for Lab 5
* **************************************************/
// Take an black and white image and find the object it it, returns an associated image with different color for each image
// You will implement your algorithm for rastering here
Mat ImageConverter::associateObjects(Mat bw_img)
{
    //initiallize the variables you will use
    int height,width; // number of rows and colums of image
    int red, green, blue; //used to assign color of each objects
    uchar pixel; //used to read pixel value of input image
    height = bw_img.rows;
    width = bw_img.cols;
    int num = 0;
    int sets[height*width];
    int sizes[height*width];

    int ** pixellabel = new int*[height];
    for (int i=0;i<height;i++)
    {
        pixellabel[i] = new int[width];
    }
   
    for(int row=0; row<height; row++)
    {
        for(int col=0; col<width; col++)
        {
            sets[col+(row*width)]=-1;
            sizes[col+(row*width)]=0;
        }
    }

    num = 1;
    bool cflag=false;

    // create associated image
    for(int row=0; row<height; row++)
    {
        for(int col=0; col<width; col++)
        {
            cflag=false;
            int val=bw_img.data[col+(row*width)];
            if (val==255)
            {
                pixellabel[row][col] = 0;
            }else
                {
                    if (row!=0)
                    {
                        if (pixellabel[row-1][col]!=0)
                        {
                            pixellabel[row][col]=pixellabel[row-1][col];
                            cflag=true;
                        }
                    }
                    if (col!=0)
                    {
                        if (pixellabel[row][col-1]!=0)
                        {
                            if (cflag==false)
                            {
                                pixellabel[row][col]=pixellabel[row][col-1];
                            }else
                                {
                                    if(pixellabel[row][col-1]!=pixellabel[row-1][col])
                                    {
                                        int s1=pixellabel[row][col-1];
                                        int s2=pixellabel[row-1][col];
                                        while (sets[s1]!=-1)
                                        {
                                            s1=sets[s1];
                                        }
                                        while (sets[s2]!=-1)
                                        {
                                            s2=sets[s2];
                                        }
                                        if (s1!=s2)
                                        {
                                            if (s2>s1)
                                            {
                                                sets[s2]=s1;
                                            }else
                                                {
                                                    sets[s1]=s2;
                                                }
                                        }

                                    }
                                }
                            cflag=true;
                        }
                    }
                    if(cflag==false)
                    {
                        pixellabel[row][col]=num;
                        num += 1;
                    }
                }
           
            }

        }

    for(int row=0; row<height; row++)
    {
        for(int col=0; col<width; col++)
        {
            int label=pixellabel[row][col];
            int new_label=label;
            while (sets[new_label]!=-1)
            {
                new_label=sets[new_label];
            }
            pixellabel[row][col]=new_label;
            sizes[new_label]+=1;
        }
    }

    for(int row=0; row<height; row++)
    {
        for(int col=0; col<width; col++)
        {
            if (sizes[pixellabel[row][col]] <95)
            {
                pixellabel[row][col] = -1;
            }
            if (sizes[pixellabel[row][col]] > 900)
            {
                pixellabel[row][col] = -1;
            }
        }
    }
    std::vector<int> objs;
    std::map<int,int> objnum2color;
    objnum2color[-1] = 0;
    int objnum = 1;
    for(int row=0; row<height; row++)
    {
        for(int col=0; col<width; col++)
        {
            if (pixellabel[row][col] !=0)
            {
                if(objnum2color.count(pixellabel[row][col]) != 1)
                {objs.push_back(pixellabel[row][col]);
                    objnum2color[pixellabel[row][col]] = objnum;
                    objnum++;
                    if(objnum == 10)
                    {
                        objnum = 1;
                    }
                }   
            }

        }
    }

    // assign UNIQUE color to each object
    Mat associate_img = Mat::zeros( bw_img.size(), CV_8UC3 ); // function will return this image
    Vec3b color;
    for(int row=0; row<height; row++)
    {
        for(int col=0; col<width; col++)
        {
            switch (objnum2color[pixellabel[row][col]])
            {
               
                case 0:
                    red    = 255; // you can change color of each objects here
                    green = 255;
                    blue   = 255;
                    break;
                case 1:
                    red    = 255; // you can change color of each objects here
                    green  = 0;
                    blue   = 0;
                    break;
                case 2:
                    red    = 0;
                    green  = 255;
                    blue   = 0;
                    break;
                case 3:
                    red    = 0;
                    green  = 0;
                    blue   = 255;
                    break;
                case 4:
                    red    = 255;
                    green  = 255;
                    blue   = 0;
                    break;
                case 5:
                    red    = 255;
                    green  = 0;
                    blue   = 255;
                    break;
                case 6:
                    red    = 0;
                    green  = 255;
                    blue   = 255;
                    break;
                case 7:
                    red    = 128;
                    green  = 128;
                    blue   = 0;
                    break;
                case 8:
                    red    = 128;
                    green  = 0;
                    blue   = 128;
                    break;
                case 9:
                    red    = 0;
                    green  = 128;
                    blue   = 128;
                     break;
                default:
                    red    = 0;
                    green = 0;
                    blue   = 0;
                    break;                   
            }

            color[0] = blue;
            color[1] = green;
            color[2] = red;
            associate_img.at<Vec3b>(Point(col,row)) = color;
        }
    }
    r_cent.clear();
    c_cent.clear();
    std::map<int,int> m00_map;
    std::map<int,int> m01_map;
    std::map<int,int> m10_map;
    std::map<int,float> theta_map;
    std::map<int,int> r_map;
    std::map<int,int> c_map;
    for (int i=0; i<int(objs.size());i++)
    {int o_num=objs[i];
    int m00=0;
    int m01=0;
    int m10=0;
    for(int row=0; row<height; row++)
    {
        for(int col=0; col<width; col++)
        {
        if (pixellabel[row][col]!=o_num)
        {continue;}
        m00=m00+1;
        m01=m01+col;
        m10=m10+row;
        }
    }
    m00_map[o_num]=m00;
    m01_map[o_num]=m01;
    m10_map[o_num]=m10;
    r_map[o_num]=m10/m00;
    c_map[o_num]=m01/m00;
    c_cent.push_back(m01/m00);
    r_cent.push_back(m10/m00);
    }
    for (int i=0; i<int(objs.size());i++)
    {int o_num=objs[i];
    float c11=0;
    float c02=0;
    float c20=0;
    int r=r_map[o_num];
    int c=c_map[o_num];
    for(int row=0; row<height; row++)
    {
        for(int col=0; col<width; col++)
        {
        if (pixellabel[row][col]!=o_num)
        {continue;}
        c11=c11+((row-r)*(col-c));
        c02=c02+((col-c)*(col-c));
        c20=c20+((row-r)*(row-r));
        }
    }
    theta_map[o_num]=atan(2.*c11/(c20-c02))/2.;
    }

Vec3b black;
black[0] = 0;
black[1] = 0;
black[2] = 0;
for (int i=0; i<int(objs.size());i++)
    {
    int r=r_map[objs[i]];
    int c=c_map[objs[i]];
    int nr=r;
    int nc=c;
    float theta=theta_map[objs[i]];
    for (int j=0; j<10; j++)
    {int x=cos(theta)*j;
    int y=sin(theta)*j;
    nr=r+y;
    nc=c+x;
    if (nr>-1&&nc>-1&&nr<height&&nc<width)
    {associate_img.at<Vec3b>(Point(nc,nr)) = black;}
    }

    for (float j=0; j<10; j++)
    {int x=cos(theta)*j;
    int y=sin(theta)*j;
    nr=r-y;
    nc=c-x;
    if (nr>-1&&nc>-1&&nr<height&&nc<width)
    {associate_img.at<Vec3b>(Point(nc,nr)) = black;}
    }

    for (int j=0; j<10; j++)
    {int x=cos(theta+(3.1416/2.))*j;
    int y=sin(theta+(3.1416/2.))*j;
    nr=r+y;
    nc=c+x;
    if (nr>-1&&nc>-1&&nr<height&&nc<width)
    {associate_img.at<Vec3b>(Point(nc,nr)) = black;}
    }

    for (int j=0; j<10; j++)
    {int x=cos(theta+(3.1416/2.))*j;
    int y=sin(theta+(3.1416/2.))*j;
    nr=r-y;
    nc=c-x;
    if (nr>-1&&nc>-1&&nr<height&&nc<width)
    {associate_img.at<Vec3b>(Point(nc,nr)) = black;}
    }

    }

    return associate_img;
}
/*****************************************************
	*Function for Lab 6
 * **************************************************/
 //This is a call back function of mouse click, it will be called when there's a click on the video window.
 //You will write your coordinate transformation in onClick function.
 //By calling onClick, you can use the variables calculated in the class function directly and use publisher
 //initialized in constructor to control the robot.
 //lab4 and lab3 functions can be used since it is included in the "lab4.h" 
void onMouse(int event, int x, int y, int flags, void* userdata)
{
		ic_ptr->onClick(event,x,y,flags,userdata);
}

void ImageConverter::CameraToWorld(int r, int c)
{
	// Extrinsic parameters CHANGE IF CAMERA IS MOVED!!!
    // Normal Robot
    double theta = 0.0;
    double Tx = 0.899;
	double Ty = 0.862;
	

    // Back corner robot
    /*
	double theta = 0.00713;
	double Tx = 0.185235;
	double Ty = 0.067681;
	*/

    // Intrinsic parameters 
    double beta = 82.644;
    double Or = 240.0;
    double Oc = 320.0;

    xw = (-cos(theta)*(r/beta - Or - Tx) + sin(theta)*(c/beta - Oc - Ty))/1000.0;
    yw = (-sin(theta)*(r/beta - Or - Tx) + cos(theta)*(c/beta - Oc - Ty))/1000.0;
}

void ImageConverter::moveArmTo(float x, float y, float z)
{
	ROS_INFO_STREAM("Move func world coords: " << x << "," << y << "," << z);
	driver_msg.destination = lab_invk(x, y, z, -45.0);
	cout<<"msg destination: \n"<<driver_msg<<endl;

	ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
	driver_msg.duration = 3.0;
	pub_command.publish(driver_msg);  // publish command, but note that is possible that
	
	spincount = 0;

	while (isReady) // Waiting for isReady to be false meaning that the driver has the new command 
	{ 	
		ros::spinOnce();  // Allow other ROS functionallity to run
		loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
		if (spincount > SPIN_RATE) // if isReady does not get set within 1 second re-publish 
		{  
			pub_command.publish(driver_msg);
			ROS_INFO_STREAM("Just Published again driver_msg");
			spincount = 0;
		}
		spincount++;  // keep track of loop count
	}
	ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		
	while(!isReady)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO_STREAM("Ready for new point");
}

void ImageConverter::pickUp(void){
	srv.request.fun = 1;
	srv.request.pin = 0;  //Digital Output 0
	srv.request.state = 1.0; //Set DO0 on
	if (srv_SetIO.call(srv)) {
		ROS_INFO("True: Switched Suction ON");
	} else {
		ROS_INFO("False");
	}
}

void ImageConverter::putDown(void){

	srv.request.fun = 1;
	srv.request.pin = 0;  //Digital Output 0
	srv.request.state = 0.0; //Set DO0 on
	if (srv_SetIO.call(srv)) {
		ROS_INFO("True: Switched Suction OFF");
		SuctionValue = 1;
	} else {
		ROS_INFO("False");
	}
}

void ImageConverter::onClick(int event,int x, int y, int flags, void* userdata)
{
	// For use with Lab 6
	// If the robot is holding a block, place it at the designated row and column. 
	double zLow,zHigh; // world coordinates for block heigth and maneuvering heigth

	zLow = 0.0275;
	zHigh = 0.2;

	if  ( event == EVENT_LBUTTONDOWN ) //if left click, do nothing other than printing the clicked point
	{  
		if (leftclickdone == 1) 
		{
			leftclickdone = 0;  // code started
			ROS_INFO_STREAM("left click:  (" << x << ", " << y << ")");  //the point you clicked

			// Calibrate
			CameraToWorld(y,x);
			ROS_INFO_STREAM("Calibration:  (" << x << ", " << y << ") to (" << xw << ", " << yw << ")");
			
			// Move above target
			moveArmTo(xw, yw, zHigh);

			// Move to target
			moveArmTo(xw, yw, zLow);
			
			// Grab block
			pickUp();

			// Move back up
			moveArmTo(xw, yw, zHigh);

			// Move to waiting point
			moveArmTo(-0.3, -0.3, zHigh);

			leftclickdone = 1; // code finished
		} else 
			{
				ROS_INFO_STREAM("Previous Left Click not finshed, IGNORING this Click"); 
			}
	}
	else if  ( event == EVENT_RBUTTONDOWN )//if right click, find nearest centroid,
	{
		if (rightclickdone == 1) 
		{  // if previous right click not finished ignore
			rightclickdone = 0;  // starting code
			ROS_INFO_STREAM("right click:  (" << x << ", " << y << ")");  //the point you clicked

			// Calibrate
			CameraToWorld(y,x);

			// Move above target
			moveArmTo(xw, yw, zHigh);

			// Move to target
			moveArmTo(xw, yw, zLow);
			
			// Grab block
			putDown();

			// Move back up
			moveArmTo(xw, yw, zHigh);

			// Move to waiting point
			moveArmTo(-0.3, -0.3, zHigh);

			rightclickdone = 1; // code finished
		} else 
			{
				ROS_INFO_STREAM("Previous Right Click not finshed, IGNORING this Click"); 
			}
	}
}

