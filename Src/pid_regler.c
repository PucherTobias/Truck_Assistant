#include "pid_regler.h"



void pid_init(pid_regler_struct *pid) {
    pid->Kp = pid->kp;
    pid->Ki = pid->ki * pid->T; //angepasste Konstanten
    pid->Kd = pid->kd / pid->T; //angepasste Konstanten
		
		pid->D=0;
		pid->dx=0;
		pid->e=0;
		pid->I=0;
		pid->w=0;
		pid->x=0;
		pid->x_last=0;

}


void pid_calc(pid_regler_struct *pid) {
  if(pid->freigabe == ON){
		pid->e = (pid->w) - (pid->x); //soll-istwert Vergleich, Reglerabweichung e berechnen

    pid->I += (pid->Ki) * pid->e; //Integral term im Voraus berechnen

    pid->I = limit(pid->I, pid->u_min, pid->u_max); // Integrierer begrenzen

    pid->dx = (pid->x) - (pid->x_last); //Änderung für D-Anteil berechnen
    
    pid->D = (pid->Kd) * (pid->dx);
    //Stellgröße u berechnen
    pid->u = (pid->Kp)*(pid->e) + (pid->I) - (pid->D); // u = Kp*e + Ki*e - Kd*dx
		pid->u = limit(pid->u, pid->u_min, pid->u_max);
	
		pid->x_last = pid->x; //Letzten istwert abspeichern
	}else if(pid->freigabe == OFF) {
		//nothing
		pid->u = 0;
	}
}


float limit(float wert, float min, float max) {
    if(wert < min)
        wert = min;
    if(wert > max)
        wert = max;
    return wert;
}