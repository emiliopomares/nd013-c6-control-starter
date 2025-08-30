/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   this->p = Kpi;
   this->i = Kii;
   this->d = Kdi;
   this->min = output_lim_mini;
   this->max = output_lim_maxi;
   this->total_cte = 0;
   this->last_cte = 0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   this->total_cte += cte * this->dt;
   this->last_cte = this->current_cte;
   this->current_cte = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control;
   control = this->p * this->current_cte + 
      this->d * (this->current_cte - this->last_cte) / this->dt +
      this->i * (this->total_cte);
   if(control>this->max) {
      control = this->max;
   }
   if(control<this->min) {
      control = this->min;
   }
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   this->dt = new_delta_time; 
}