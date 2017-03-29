
#include "ros/ros.h"
#include "servo/servo_order.h"
#include "maestro.h"

#include "std_msgs/Bool.h"

Maestro maestro;
ros::Time lastbeat;
std::vector<string> devices;
unsigned char portMotor;

bool autorisation = false;

void maestroCallback(const servo::servo_order& msg)
{
	if(autorisation){
  ROS_INFO("servo_maestro: executed order [%d, %lf]", msg.numeroMoteur, msg.position);
  maestro.fixePosition(msg.numeroMoteur, msg.position);
  }
}

void heartbeat(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data){lastbeat = ros::Time::now();}
}

void testTimeOut(const ros::TimerEvent&){
	if(ros::Time::now()-lastbeat>=ros::Duration(1.5)){
		autorisation = false;	
		maestro.fixePosition(portMotor, 6000.0);
		ros::Duration(0.2).sleep();
		maestro.fixePosition(portMotor, 6000.0);
	}
	else{
		autorisation=true;
	}
}


int main(int argc, char **argv)
{
  std::string port;
  ros::init(argc, argv, "servoListener");
  ros::NodeHandle n;
  n.getParam("maestroPath", port);
  ROS_INFO("servo_maestro: port given : %s", port.c_str());
  
  if(maestro.connect(port.c_str())>=0){
  	ROS_INFO("servo_maestro: ready");
  }
  else{
  	ROS_ERROR("servo_maestro: unnable to connect");
  }
  
  ros::Subscriber sub = n.subscribe("servo_raw_order", 100, maestroCallback);
  
  n.getParam("devices", devices);
  std::map<string, int> motorParam;
  
  n.getParam("motor", motorParam);
  
  portMotor = motorParam["port"];
  
  lastbeat = ros::Time::now();
  
  ros::Subscriber emergencyOn = n.subscribe("bondServo",100, heartbeat);
  
  ros::Timer timer = n.createTimer(ros::Duration(1.0), testTimeOut);
  
  ros::spin();

  return 0;
}
//---------------------définition de la classe Maestro------------------------

// Ouvre le port et construit l'objet à partir de l'adresse du port USB
Maestro::Maestro()
{
    //std::cout << "coucou je suis dans le constructeur" << std::endl;
    //definition des bornes
    borneMin = 4000;
    borneMax = 8000;
}

Maestro::~Maestro()
{
    //destructeur
}

int Maestro::connect (const char * adresse){
    numeroPort = open(adresse, O_RDWR | O_NONBLOCK | O_NOCTTY); //Si ce numéro vaut -1, il y a erreur d'ouverture
    int i =1;
   /* while(numeroPort==-1 && i<10){
			stringstream ss;
			ss << i;
			string str = ss.str();
    	string adressechelou = "/dev/ttyACM99";
    	string adresseComplete = "/dev/ttyACM" + str;
    	ROS_INFO("on teste %s", adresseComplete.c_str());
    	numeroPort = open(adresseComplete.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    	i++;	
    }
    */
    ROS_INFO("servo_maestro: numeroPort %d", numeroPort);
    return numeroPort;
}



bool Maestro::fixePosition(unsigned char numeroMoteur, float positionVisee) const
{

    return Maestro::fixePositionMaestro(numeroMoteur, convertirFloatEnMaestro(positionVisee));
}

/*Inutile
//Renvoie -1 en cas d'erreur
short Maestro::mesurePositionMaestro(unsigned char numeroMoteur) const
{
    unsigned char command[] = {0x90, numeroMoteur}; //Commande à envoyer pour faire la mesure
    if(write(numeroPort, command, sizeof(command)) == -1) // Erreur d'envoi de requête
    {
        perror("error writing");
        return static_cast<short>(-1);
    }
    unsigned char response[2];
    if(read(numeroPort,response,2) != 2) // Erreur de récupération de la valeur
    {
        perror("error reading");
        return static_cast<short>(-1);
    }
    return static_cast<short>(response[0] + 256*response[1]);
}
*/

bool Maestro::fixePositionMaestro(unsigned char numeroMoteur, unsigned short positionVisee) const
{
    if(positionVisee > borneMax || positionVisee < borneMin){
        //perror("position visée en dehors des bornes autorisées");
        //positionVisee = max((unsigned short int) borneMin, min((unsigned short int)borneMax, positionVisee));
        positionVisee=6000;
    
    }
    

    unsigned char command[] = {0x84, numeroMoteur, positionVisee & 0x7F, positionVisee >> 7 & 0x7F}; // Commande à envoyer
    if (write(numeroPort, command, sizeof(command)) == -1) // Erreur de commande
    {
        ROS_INFO("servo_maestro: failed order");
        return false;
    }
    ROS_INFO("servo_maestro: position set %d %d %d", command[0], command[1], command[2]);
    return true;
}

unsigned short Maestro::convertirFloatEnMaestro(float mesure) const
{
    return static_cast<unsigned short>(mesure);
}

float Maestro::convertirMaestroEnFloat(short mesure) const
{
    return static_cast<float>(mesure);
}



