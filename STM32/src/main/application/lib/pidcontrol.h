#pragma once
#include <stdio.h>

typedef struct {
  // parameters, input to pidcontrol_config()
  float Kp;                  // proportional gain
  float Ti;
  float Td;                  // Derivator time (sec)

  float I_min;               // Minimum integrator clamp
  float I_max;               // Maximum integrator clamp

  float P, I, D;
  
  
  // derived (cached) parameters, output from pidcontrol_config()
  // TODO: move out of here, external filter
  float alpha;               // low pass filter weighing factor (usually 0.1)
  float inv_Tf;              // 1/Tf, Tf = Filter time = alpha * Derivator time (sec)
  float edf_n_1;             // edf[n-1] (derivative error)  

  // status flags, only one...
  int saturated;
  
} PIDControl_t;


// inits derived control parameters
void pidcontrol_config(PIDControl_t *control, float Kp, float Ti, float Td, float alpha, float I_min, float I_max);

// inits control state
void pidcontrol_init(PIDControl_t *control, float u_n);

// dt is seconds since last update
// TODO: timestamp here!?
float pidcontrol_update(PIDControl_t *c, float r_n, float y_n, float dt);
float pidcontrol_update2(PIDControl_t *c, float r_n, float y_n, float dydt, float dt);
void pidcontrol_set_config(PIDControl_t *c, const char *name, float value);

#if _DLIB_FILE_DESCRIPTOR
int pidcontrol_print_config(FILE *file, const PIDControl_t *c);
#endif
