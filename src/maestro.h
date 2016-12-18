#include <iostream>
#include <stdio.h> // Gestionnaire entrées sorties
#include <fcntl.h> // Autre gestionnaire de fichiers (moins courant), contient open, read, etc...
#include <unistd.h>
#include <algorithm>



using namespace std;

class Maestro
{
public:
/* Constructeur de l'objet Maestro. L'adresse du port est :
- Sous Windows : "\\\\.\\USBSER000"
- Sous Linux : "/dev/ttyACM0"
Les bornes sont toutes initialisées à 4000, 8000
*/
    Maestro();
    virtual ~Maestro(); // Destructeur
    void connect(const char * adresse);
    bool fixePosition(unsigned char numeroMoteur, float positionVisee) const;
private:
    bool fixePositionMaestro(unsigned char numeroMoteur, unsigned short positionVisee) const;
    unsigned short convertirFloatEnMaestro(float mesure) const;
    float convertirMaestroEnFloat(short mesure) const;
    int numeroPort;
    unsigned short borneMin;
    unsigned short borneMax;
};


