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
   * Initialize PID coefficients (and errors, if needed)
   **/
   m_error_actual = 0.;
   m_error_total = 0.;
   m_error_last = 0.;
   m_Kp = Kpi;
   m_Ki = Kii;
   m_Kd = Kdi;
   m_output_lim_min = output_lim_mini;
   m_output_lim_max = output_lim_maxi;
   m_delta_time = 0.;
}

void PID::UpdateError(double cte) {
   /**
   * Update PID errors based on cte.
   **/
   m_error_last = m_error_actual;
   m_error_actual = cte;
   m_error_total += cte*m_delta_time; // proper integral over time for integral term
}

double PID::TotalError() {
   /**
   * Calculate and return the total error
   * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control = 0.;

   // calculate control (I prefer convention of error in form target-current, so I use a "+" sign here)
   if (m_delta_time > 1e-9) 
   {
      control += m_Kp*m_error_actual;
      control += m_Ki*m_error_total;
      control += m_Kd*(m_error_actual-m_error_last)/m_delta_time;
      std::cout << "control " << control << std::endl;
   }

   // limit control   
   if (control < m_output_lim_min) {control = m_output_lim_min;}
   if (control > m_output_lim_max) {control = m_output_lim_max;}

   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with new value
   */
   m_delta_time = new_delta_time;
}
