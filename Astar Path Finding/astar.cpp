#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "stdlib.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "nav_msgs/MapMetaData.h"

//start
//ros::param::set("goalx", "4");
//ros::param::set("goaly", "9");

//end

typedef struct node
{
	int prob;		
	int f_cost;
	int visited;		
}vals;

typedef struct node1
{
	float i,j;
}PATH;

double piv = atan(1)*4,z=0;
float nac,oac;

void Odomcallback(nav_msgs::Odometry odom)
{
	z = 2*asin(odom.pose.pose.orientation.z);
	nac = z - oac;
}

void TRT(double dt)
{
	ros::NodeHandle n;
	ros::Publisher pubd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);	
      	ros::Rate loop_rate(10);

        geometry_msgs::Twist msg;
	float count=12;
	while(count>0)
	{
		msg.linear.x = (dt*2);
		pubd.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		count=count-1;
	}	
}

void RTT(double angle)
{
	int a1;
	a1 = fabs(angle)/angle;
	angle = fabs(angle);
	if(angle<=0.004)
	return;
 	ros::NodeHandle n;
	ros::Rate loop_rate(10);	
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	nac = 0.0;
       	float velocity ;
	while(angle - fabs(nac) > 0.004)
	{

		if(angle - fabs(nac) > 0.08)		
			velocity = 0.2;
		else
			velocity = 0.01;

		geometry_msgs::Twist dis;
		dis.angular.z = ((a1 > 0)? velocity : -velocity);	       
		pub.publish(dis);
	  	ros::spinOnce();
		loop_rate.sleep();
	}
	oac+=nac;

}



void Travel(PATH path[],int size)
{
	int i=0;
	float current_x = path[0].i;
	float current_y = path[0].j;
	double angle,dt;
	
	for(i=1;i<=size;i++)
	{
		if(path[i].j-current_y == -1)
		{	
			if(path[i].i-current_x == 0)
			{
				angle = piv;
				dt = 1;
			}
			else
			{
				if(path[i].i-current_x == -1)
				{
					angle = 3*(piv/4); 
					dt = 1.414214;
				}
				else
				{
					if(path[i].i-current_x == 1)
					{
						angle = -3*(piv/4);	
						dt = 1.414214;
					}
				}
			}
			angle = -(z-angle);
		}
		else if(path[i].j-current_y == 1)
		{
			if(path[i].i-current_x == 0)
			{
				angle = 0;
				dt = 1;
			}
			else 
			{
				if(path[i].i-current_x == -1)
				{
					angle = -(piv/4); 
					dt = 1.414214;
				}
				else 
				{
					if(path[i].i-current_x == 1)
					{
						angle = (piv/4);	
						dt = 1.414214;
					}
				}
			}

			angle = -(angle+z);

		}
		else if(path[i].j-current_y == 0)
		{
			if(path[i].i-current_x == -1)
			{
				angle = ((piv/2)-z); 
				dt = 1;
			}
			else
			{			
				if(path[i].i-current_x == 1)
				{
					angle = -((piv/2)+z);	
					dt = 1;
				}
			}
		}

		

		RTT(angle);
		TRT(dt);		

		current_x = path[i].i;
		current_y = path[i].j;
	}
}



void Pathfinder(int d[])
{
	int i=0,j=0,k=0;
	int sti,stj,gi,gj;	
	int counter = 0,CG = 0,min;

	vals **grid = (vals**)malloc(21*sizeof(vals*)); 
	for (i=0;i<21;i++)
    	grid[i] = (vals*)malloc(19*sizeof(vals));

	
	for(i=0;i<21;i++)				// converting 1-D array into 2-D array-Grid map
	{
		for(j=0;j<19;j++)
		{
			grid[i][j].prob = d[k];	
			grid[i][j].visited = 0;    // initialize all values to 0
			grid[i][j].f_cost = 0;
			k++;
		}
	}
	sti = 11;stj = 0;//initial path based on (-8,-2) to (4.5,9)
	gi = 0;gj = 16;
	//start
	
	int ls,ms;
	ros::NodeHandle nhx;
	std::string gx,gy;
	if (nhx.getParam("goalx", gx))
	{
	ls = atoi(gx.c_str());
	if(ls < 0)
	{
	gj = fabs(ls + 11); 
	}
	else
	{
	}
	}
	
	if (nhx.getParam("goaly", gy))
	{
	ms = atoi(gy.c_str());
	if(ms < 0)
	{
	gi = fabs(-ms + 9);
	}
	}
	//end


	
	if(grid[sti][stj].prob == 1)
	{
		grid[sti][stj].prob = 0;		
	}



	if(grid[gi][gj].prob == 1)
	{
		grid[gi][gj].prob = 0;
	}
	

	PATH path[399];
	
	path[0].i = sti;
	path[0].j = stj; 

	i=sti,j=stj;
	grid[i][j].prob = 5;
	
	while(i!=gi || j!=gj)
	{
		counter++;
		CG++;
		min = 1000000;

		if(i>=1)
		{
			if(grid[i-1][j].prob==0 && grid[i-1][j].visited == 0)
			{	
				grid[i-1][j].f_cost = CG + float(fabs(i-1 -gi) + fabs(j - gj));	
				if(grid[i-1][j].f_cost <= min)
				{
					min = grid[i-1][j].f_cost;
					path[counter].i = i-1;
					path[counter].j = j; 
				}	
			}
			if(j>=1)
			{
				if(grid[i-1][j-1].prob==0 && grid[i-1][j-1].visited == 0)
				{
					grid[i-1][j-1].f_cost = CG + float(fabs(i-1 -gi) + fabs(j - 1 - gj));	
					if(grid[i-1][j-1].f_cost <= min)
					{
						min = grid[i-1][j-1].f_cost;
						path[counter].i = i-1;
						path[counter].j = j-1; 
					}
				}
			}
			if(j<18)
			{
				if(grid[i-1][j+1].prob==0 && grid[i-1][j+1].visited == 0)
				{	
					grid[i-1][j+1].f_cost = CG + float(fabs(i-1 -gi) + fabs(j + 1 - gj));
					if(grid[i-1][j+1].f_cost <= min)
					{
						min = grid[i-1][j+1].f_cost;
						path[counter].i = i-1;
						path[counter].j = j+1; 
					}		
				}
			}
		}
		if(i<20)
		{
			if(grid[i+1][j].prob==0 && grid[i+1][j].visited == 0)
			{			
				grid[i+1][j].f_cost = CG + float(fabs(i+1 -gi) + fabs(j - gj));
				if(grid[i+1][j].f_cost <= min)
				{
					min = grid[i+1][j].f_cost;
					path[counter].i = i+1;
					path[counter].j = j; 
				}			
			}
			if(j>=1)
     			{
				if(grid[i+1][j-1].prob==0 && grid[i+1][j-1].visited == 0)
				{	
					grid[i+1][j-1].f_cost = CG + float(fabs(i+1 -gi) + fabs(j-1 - gj));
					if(grid[i+1][j-1].f_cost <= min)
					{
						min = grid[i+1][j-1].f_cost;
						path[counter].i = i+1;
						path[counter].j = j-1; 
					}		
				}
		   	}
		        if(j<18)
		       	{
				if(grid[i+1][j+1].prob==0 && grid[i+1][j+1].visited == 0)
				{				
					grid[i+1][j+1].f_cost = CG + float(fabs(i+1 -gi) + fabs(j+1 - gj));
					if(grid[i+1][j+1].f_cost <= min)
					{
						min = grid[i+1][j+1].f_cost;
						path[counter].i = i+1;
						path[counter].j = j+1; 
					}		
				}
			}
		}
		if(j>=1)
		{
			if(grid[i][j-1].prob==0 && grid[i][j-1].visited == 0)
			{
				grid[i][j-1].f_cost = CG + float(fabs(i -gi) + fabs(j-1 - gj));
				if(grid[i][j-1].f_cost <= min)
				{
					min = grid[i][j-1].f_cost;
					path[counter].i = i;
					path[counter].j = j-1; 
				}					
			}
		}
		if(j<18)
		{
			
			if(grid[i][j+1].prob==0 && grid[i][j+1].visited == 0)
			{
				grid[i][j+1].f_cost = CG+ float(fabs(i -gi) + fabs(j+1 - gj));
				if(grid[i][j+1].f_cost <= min)
				{
					min = grid[i][j+1].f_cost;
					path[counter].i = i;
					path[counter].j = j+1; 
				}	
			}
		}

		i = path[counter].i;
		j = path[counter].j;
		
		grid[i][j].visited = 1;
		grid[i][j].prob = 7;

	}
	
	
	for(i=0;i<21;i++)
	{
		vals* currentPtr = grid[i];
    		free(currentPtr);
	}

	free(grid);

	Travel(path,counter);

}




void Startastar()
{
ros::NodeHandle n;
ros::Subscriber path2 = n.subscribe("odom", 1000, Odomcallback);    

int vals[21*19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,0,0,0,0,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1};

Pathfinder(vals);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "astar");
	Startastar();
     	ros::spin();
}
