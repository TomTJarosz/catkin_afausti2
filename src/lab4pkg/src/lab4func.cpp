#include "lab4pkg/lab4.h"

/** 
 * function that calculates an elbow up Inverse Kinematic solution for the UR3
 */
std::vector<double> lab_invk(float xWgrip, float yWgrip, float zWgrip, float yaw_WgripDegree)
{

	double xcen,ycen,zcen,theta6,theta5,theta4,theta3,theta2,theta1,x3end,y3end,z3end; 
	double xgrip,ygrip,zgrip;
	double a1,a2,a3,a4,a5,a6;
	double d1,d2,d3,d4,d5,d6;
	double yawGrip;
	double rcSqrd,lc,sc,le,reSqrd,se,de;

	a1 = 0;
	d1 = 0.152;
	a2 = 0.244;
	d2 = 0.120;
	a3 = 0.213;
	d3 = -0.093;
	a4 = 0;
	d4 = 0.083;
	a5 = 0;
	d5 = 0.083;
	a6 = 0.0535;
	d6 = (0.082+0.056);

	xgrip = xWgrip + 0.150;
	ygrip = yWgrip - 0.150;
	zgrip = zWgrip + 0.010;
	yawGrip = yaw_WgripDegree*PI/180.0;
	
	xcen = xgrip - a6*cos(yawGrip);
	ycen = ygrip - a6*sin(yawGrip);
	zcen = zgrip;

	rcSqrd = pow(xcen,2) + pow(ycen,2); //square of distance from origin to xcen,ycen,zcen
	lc = d2 + d3 + d4;
	sc = sqrt(rcSqrd - pow(lc,2.0)); 

	theta1 = atan2(ycen,xcen) - atan2(lc,sc); 
	theta6 = PI/2 + theta1 - yawGrip; 
 
  se = sc - d5;
	x3end = se*cos(theta1);
	y3end = se*sin(theta1);
	z3end = zcen + d6;

	de = z3end - d1;
	reSqrd = pow(se,2.0) + pow(de,2.0);
	
	theta2 = -acos((pow(a2,2.0) + reSqrd - pow(a3,2.0))/(2.0*a2*sqrt(reSqrd))) - atan2(de,se);
	theta3 = PI - acos((pow(a2,2.0) + pow(a3,2.0) - reSqrd)/(2.0*a2*a3));
	theta4 = -theta2 - theta3 - PI/2; 
	theta5 = -PI/2;  
	
	// View values
	//use cout
	
	cout<<"xcen: "<< xcen <<endl;
	cout<<"ycen: "<< ycen <<endl;
	cout<<"zcen: "<< zcen <<endl;
	
	cout<<"theta1: "<< theta1<<endl;
	cout<<"theta2: "<< theta2<<endl;
	cout<<"theta3: "<< theta3<<endl;
	cout<<"theta4: "<< theta4<<endl;
	cout<<"theta5: "<< theta5<<endl;
	cout<<"theta6: "<< theta6<<endl;

	// check that your values are good BEFORE sending commands to UR3
	//lab_fk calculates the forward kinematics and convert it to std::vector<double>
	return lab_fk((float)theta1,(float)theta2,(float)theta3,(float)theta4,(float)theta5,(float)theta6);
}
