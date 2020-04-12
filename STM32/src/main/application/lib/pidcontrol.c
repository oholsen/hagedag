#include "pidcontrol.h"
#include <string.h>
#include <stdio.h>

#define clip1(x, max)       (x < -max ? -max : (x > max ? max : x))
#define clip2(x, min, max)  (x < min ? min : (x > max ? max : x))


// TODO: configure with sample time (if fixed). Saves some multiplications/divisions...


static void _config(PIDControl_t *c)
{
    // derived parameters
    c->inv_Tf = 0;
    if (c->Td > 0.0 && c->alpha > 0.0)
        c->inv_Tf = 1.0 / (c->alpha * c->Td);
}

void pidcontrol_config(PIDControl_t *c, float Kp, float Ti, float Td, float alpha, float I_min, float I_max)
{
  c->Kp = Kp;
  c->Ti = Ti;
  c->Td = Td;

  c->alpha = alpha;  

  c->I_min = I_min;
  c->I_max = I_max;

  _config(c);
}


void pidcontrol_init(PIDControl_t *c, float u_n)
{
  // set state
  c->I       = 0.0;
  c->edf_n_1 = 0.0;
  c->saturated = 0;
}

/*

ARM:

 y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
    A0 = Kp + Ki + Kd
    A1 = (-Kp ) - (2 * Kd )
    A2 = Kd  




en: Kp + Ki + Kd
en-1: -Kp -2Kd
en-2: Kd


ei = Ii - I0



*/


/*
 * Roy Vegard Ovesen:
 *
 * Ok! Here is the PID controller algorithm that I would like to see
 * implemented:
 *
 *   delta_u_n = Kp * [ (ep_n - ep_n-1) + ((Ts/Ti)*e_n)
 *               + (Td/Ts)*(edf_n - 2*edf_n-1 + edf_n-2) ]
 *
 *   u_n = u_n-1 + delta_u_n
 *
 * where:
 *
 * delta_u : The incremental output
 * Kp      : Proportional gain
 * ep      : Proportional error with reference weighing
 *           ep = beta * r - y
 *           where:
 *           beta : Weighing factor
 *           r    : Reference (setpoint)
 *           y    : Process value, measured
 * e       : Error
 *           e = r - y
 * Ts      : Sampling interval
 * Ti      : Integrator time
 * Td      : Derivator time
 * edf     : Derivate error with reference weighing and filtering
 *           edf_n = edf_n-1 / ((Ts/Tf) + 1) + ed_n * (Ts/Tf) / ((Ts/Tf) + 1)
 *           where:
 *           Tf : Filter time
 *           Tf = alpha * Td , where alpha usually is set to 0.1
 *           ed : Unfiltered derivate error with reference weighing
 *             ed = gamma * r - y
 *             where:
 *             gamma : Weighing factor
 * 
 * u       : absolute output
 * 
 * Index n means the n'th value.
 * 
 * Low-pass: a = dt/(RC+dt), 1-a = RC/(RC+dt). RC >> dt, a ~ 0 and y_n moves slowly
 * y_n = a*x_n + (1-a)*y_n-1
 * y_n = y_n-1 + a*(x_n - y_n-1)


 * Inputs:
 * enabled ,
 * y_n , r_n , beta=1 , gamma=0 , alpha=0.1 ,
 * Kp , Ti , Td , Ts (is the sampling time available?)
 * u_min , u_max
 * 
 * Output:
 * u_n
 *
 * Oystein Haug Olsen:
 * This control is the special case 
 *   gamma = 0 --- gamma = 1????
 *   beta = 1
 *   Ti, Td > 0 
 * ALSO, changed to clamping integrator only.
 */

float pidcontrol_update(PIDControl_t *c, float r_n, float y_n, float Ts)
{
  // Calculates error:
  float e_n = r_n - y_n;

  c->P = c->Kp * e_n;
  
  if (c->Ti > 0) {
      c->I += Ts * c->Kp * e_n / c->Ti;      
      if (c->I < c->I_min) {
          c->saturated = 1;
          c->I = c->I_min;
      } else if (c->I > c->I_max) {
          c->saturated = 1;
          c->I = c->I_max;
      } else {
          c->saturated = 0;
      }
  } else {
      c->I = 0;
  }
  
  // Filters the derivate error:
  c->D = 0;
  if (c->Td > 0.0 && c->inv_Tf > 0.0) {
    float r = Ts * c->inv_Tf;
    float edf_n = (c->edf_n_1 - y_n * r) / (r + 1);
    c->D = c->Kp * c->Td * (edf_n - c->edf_n_1) / Ts;
    c->edf_n_1 = edf_n;
  }
  
  return c->P + c->I + c->D;
}


// Update with explicit derivative (of y only, not when reference changes as gamma = 0)
// TODO: also at gamma > 0 to speed up step response
float pidcontrol_update2(PIDControl_t *c, float r_n, float y_n, float dydt, float Ts)
{
  float e_n;             // error

  // Calculates error:
  e_n = r_n - y_n;

  c->P = c->Kp * e_n;

  if (c->Ti > 0) {
      c->I += Ts * c->Kp * e_n / c->Ti;
      if (c->I < c->I_min) {
          c->saturated = 1;
          c->I = c->I_min;
      } else if (c->I > c->I_max) {
          c->saturated = 1;
          c->I = c->I_max;
      } else {
          c->saturated = 0;
      }
  } else {
      c->I = 0;
  }

  c->D = 0;
  if (c->Td > 0.0) {
    c->D = -c->Kp * c->Td * dydt;
  }
  
  return c->P + c->I + c->D;
}




void pidcontrol_set_config(PIDControl_t *c, const char *name, float value)
{
    if (strcmp(name, "p") == 0)
        c->Kp = value;
    else if (strcmp(name, "i") == 0)
        c->Ti = value;
    else if (strcmp(name, "d") == 0)
        c->Td = value;
    else if (strcmp(name, "alpha") == 0)
        c->alpha = value;
    else if (strcmp(name, "min") == 0)
        c->I_min = value;
    else if (strcmp(name, "max") == 0)
        c->I_max = value;
    else if (strcmp(name, "amp") == 0) {
        c->I_min = value;
        c->I_max = value;
    }    
    
    _config(c);
}

#if _DLIB_FILE_DESCRIPTOR

int pidcontrol_print_config(FILE *file, const PIDControl_t *c)
{
    return fprintf(file, "Kp = %g  Ti = %g  Td = %g  alpha = %g", c->Kp, c->Ti, c->Td, c->alpha);
}

#endif