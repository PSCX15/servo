#include "ros/ros.h"
#include "servo/servo_order.h"
#include "maestro.h"

Maestro maestro;

void maestroCallback(const servo::servo_order& msg)
{
  ROS_INFO("I heard: [%d, %lf]", msg.numeroMoteur, msg.position);
  maestro.fixePosition(msg.numeroMoteur, msg.position);
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "servoListener");
  ros::NodeHandle n;
  
  maestro.connect("/dev/ttyACM0");
  
  ros::Subscriber sub = n.subscribe("servo_order", 100, maestroCallback);
  ros::spin();

  return 0;
}

// Ouvre le port et construit l'objet à partir de l'adresse du port USB
Maestro::Maestro()
{
    std::cout << "coucou je suis dans le constructeur" << std::endl;
    borneMin = 4000;
    borneMax = 8000;
}

Maestro::~Maestro()
{
    //destructeur
}

void Maestro::connect (const char * adresse){
    int numeroPort = open(adresse, O_RDWR | O_NONBLOCK | O_NOCTTY); //Si ce numéro vaut -1, il y a erreur d'ouverture
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
        perror("position visée en dehors des bornes autorisées");
        positionVisee = max((unsigned short int) borneMin, min((unsigned short int)borneMax, positionVisee));
    }

    unsigned char command[] = {0x84, numeroMoteur, positionVisee & 0x7F, positionVisee >> 7 & 0x7F}; // Commande à envoyer
    if (write(numeroPort, command, sizeof(command)) == -1) // Erreur de commande
    {
        perror("error writing");
        return false;
    }
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



