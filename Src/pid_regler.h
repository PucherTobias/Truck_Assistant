#include <main.h>

typedef enum {
    ON, OFF
}pid_freigabe;

typedef struct
{
    float x; //istwert
    float x_last; //letzter istwert
    float dx; //Änderung Istwert für D-Anteil

    float w; //sollwert
    float e; //reglerabweichung 
    float u; //stellwert
    float T; // zeitkonstante

    float kp; //Verstärkung kp
    float ki; //Integrationsbeiwert
    float kd; //Differenzierbeiwert

    float Kp; //umgerechnetes Kp
    float Ki; //umgerechnetes Ki
    float Kd; //umgerechnetes Kd


    float I; //Integral Rechenwert für I-Anteil
    float D; //Differenzierer Rechenwert für D-Anteil
    float u_max, u_min; //maximaler / minimaler Reglerausgang

} 
pid_regler_struct;


extern void pid_init(pid_regler_struct *pid);


float limit(float wert, float min, float max);
