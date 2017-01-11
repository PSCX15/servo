#include "ros/ros.h"
#include "servo/servo_command.h"
#include "servo/servo_order.h"
#include "maestro.h"

void interpretCommand(const servo::servo_command& msg);
int getRawOrder(float value, std::map<string, int> deviceParam);

std::vector<string> devices;
std::map<string, map<string, int> > params;

ros::Publisher rawOrder_pub;

int main(int argc, char **argv)
{
  
  std::string deviceName, bilan;
  std::map<string, int> deviceParam;
 
  
  ros::init(argc, argv, "servoListener");
  ros::NodeHandle n;

  //déclare le publisher pour le topic raw_order
  rawOrder_pub = n.advertise<servo::servo_order>("servo_raw_order", 100);
  ros::Rate loop_rate(10);

  //declare le subscriber pour le monde exterieur
  ros::Subscriber command_sub = n.subscribe("servo_command", 100, interpretCommand);
  
  n.getParam("devices", devices);
  
  bilan = "[";
  for(int i=0; i<devices.size(); i++)
  {
    deviceName = devices[i];
    n.getParam(deviceName, deviceParam);
    deviceParam["dCommand"] = deviceParam["cmax"] - deviceParam["cmin"];
    deviceParam["dRawOrder"] = deviceParam["max"] - deviceParam["min"];
    params[deviceName] = deviceParam;
    bilan += deviceName + "  ";
  }
  bilan += "]";
  
  
  ROS_INFO("servo_listener configured with %s", bilan.c_str());
  
  //ros::Subscriber sub = n.subscribe("servo_command", 100, interpretCommand);
  ros::spin();

  return 0;
}

void interpretCommand(const servo::servo_command& msg)
{
  static std::map<string, int> deviceParam;
  static int rawOrder, numeroMoteur;
  ROS_INFO("servo_node: got command [%s, %lf]", msg.device.c_str(), msg.value);
  if(params.count(msg.device)>0){
    //device exists
    deviceParam = params[msg.device];
    rawOrder = getRawOrder(msg.value, deviceParam);
    if(rawOrder>0) //no errors
    {
      servo::servo_order order;
      order.position = rawOrder;
      order.numeroMoteur = deviceParam["port"];
      rawOrder_pub.publish(order);
      
      //dispatch raw Order
      
    }
    else
    {
	  ROS_ERROR("servo_listener : got incoherent order %lf  for '%s' while accepted range is (%d, %d)->(%d, %d)",msg.value, msg.device.c_str(), deviceParam["cmin"], deviceParam["cmax"] , deviceParam["min"], deviceParam["max"]);
	}
  }
}

int getRawOrder(float value, std::map<string, int> deviceParam)
{
  
  if(value >= deviceParam["cmin"] && value <= deviceParam["cmax"])
  {//legit command
    int rawOrder = (int) deviceParam["min"]+(value - deviceParam["cmin"])*deviceParam["dRawOrder"]/deviceParam["dCommand"];
    return rawOrder;
  }
  else
  {//non_valid command
    
    return -1;
  }
}
    
    
    
    
    
