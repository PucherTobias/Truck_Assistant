#include "pid_regler.h"



void pid_init(pid_regler_struct *pid) {
    pid->Kp = pid->kp;
    pid->Ki = pid->ki * pid->T; //angepasste Konstanten
    pid->Kd = pid->kd / pid->T; //angepasste Konstanten

}


void pid_calc(pid_regler_struct *pid) {
    pid->e = (pid->u) - (pid->x); //soll-istwert Vergleich, Reglerabweichung e berechnen

    pid->I += (pid->Ki) * pid->e; //Integral term im Voraus berechnen

    pid->I = limit(pid->I, pid->u_min, pid->u_max); // Integrierer begrenzen

    pid->dx = (pid->x) - (pid->x_last); //Änderung für D-Anteil berechnen
    
    pid->D = (pid->Kd) * (pid->dx);
    //Stellgröße u berechnen
    pid->u = (pid->Kp)*(pid->e) + (pid->I) - (pid->D); // u = Kp*e + Ki*e - Kd*dx

}


float limit(float wert, float min, float max) {
    if(wert < min)
        wert = min;
    if(wert > max)
        wert = max;
    return wert;
}