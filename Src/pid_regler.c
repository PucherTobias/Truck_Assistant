#include "pid_regler.h"



void pid_init(pid_regler_struct *pid) {
    pid->Kp = kp;
    pid->Ki = ki * pid->T; //angepasste Konstanten
    pid->Kd = kd / pid->T; //angepasste Konstanten

}


void pid_calc(pid_regler_struct *pid) {
    pid->e = (pid->u) - (pid-x) //soll-istwert Vergleich, Reglerabweichung e berechnen

    pid->I += (pid->Ki) * e; //Integral term im Voraus berechnen

    pid->I = limit(pid->I, pid->u_min, pid->u_max); // Integrierer begrenzen

    pid->dx = (pid->x) - (pid->x_last); //Änderung für D-Anteil berechnen
    
    pid->D = (pid->Kd) * (pid->dx);
    //Stellgröße u berechnen
    pid->u = (pid->Kp)*e + (pid->I) - (pid->D) // u = Kp*e + Ki*e - Kd*dx

}