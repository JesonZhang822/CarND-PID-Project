#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {

	Kp_ = 0;
	Ki_ = 0;
	Kd_ = 0;
	
	p_error = 0;
	i_error = 0;
	d_error = 0;

}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	//twiddle param, kp-->kd-->ki
	p.push_back(Kp);
	p.push_back(Kd);
	p.push_back(Ki);

	dp.push_back(0.1 * Kp);
	dp.push_back(0.1 * Kd);
	dp.push_back(0.1 * Ki);


	// Init the twiddle
	error = 0;
	best_error = std::numeric_limits<double>::max();

	echoes = 40;//20
	num_step = 0;
	num_echo = 0;

	param_index = 0;
	try_positive = true;
	is_first = true;
	
}

void PID::UpdateError(double cte) {

	double pre_cte ;

	if (is_first) {

		pre_cte = cte;
		p_error = cte;
		i_error = cte;
		d_error = 0;

		is_first = false;

	} else {

		pre_cte = p_error;
		p_error = cte;
		d_error = cte - pre_cte;
		i_error = i_error + cte ;
	}

}

double PID::TotalError() {

	double total_error ;

	total_error = - Kp_ * p_error - Ki_ * i_error - Kd_ * d_error;

	return total_error ;

}

void PID::Twiddle(double cte,int tune_index = 5){


	num_step += 1;

	if (num_step == 1){

		// calculate the PID param index
		if (tune_index > 2) {

			param_index = num_echo % 3;

		} else {

			param_index = tune_index;

		}
		

		p[param_index] += dp[param_index];

		UpdateParam(p[0],p[2],p[1]);

		is_first = true ;
		error = 0;
		try_positive = true;

		std::cout << "The dp is :" << dp[0] << ", "<< dp[1] << ", " << dp[2] << " The param_index is :" << param_index <<endl;

	}else if (num_step >= echoes){

		// calculate the sum of error**2
		error += pow(cte,2);

		if (num_step == 2 * echoes) {

			num_step = 0;
			num_echo += 1;

			std::cout << "The number of echo is : " << num_echo << " The best error is :" << best_error << endl;

			// calculate the mean error
			error = error / echoes;

			if (num_echo == 1){

				best_error = error ;
				dp[param_index] *= 1.1;

			} else {

				// get the min mean error
				if (error <= best_error) {

					best_error = error ;
					dp[param_index] *= 1.1;

					try_positive = true;

					std::cout << "The positive is true ! the dp_p is: " << dp[0] << " the dp_d: " << dp[1] << " the dp_i : "<< dp[2] <<"!"<< std::endl;

				}else {
		
					if (try_positive) {

						p[param_index] -= 2*dp[param_index];
						UpdateParam(p[0],p[2],p[1]);

						//is_first = true;
						num_step = 1;
						error = 0;
						try_positive = false ;

						std::cout << "The positive is False ! the dp_p is : " << dp[0] << " the dp_d: "<< dp[1] <<" the dp_i: "<< dp[2] <<"!"<< std::endl;

					} else {

						dp[param_index] *= 0.9;
						p[param_index] += dp[param_index];


						std::cout << "The p_p : " << p[0] << " the p_d: " << p[1] << " the p_i : " << p[2] << endl;
						std::cout << "The dp_p : " << dp[0] << " the dp_d: " << dp[1] << " the dp_i : " << dp[2] << endl;

					}

				}


			}


		}

	}


	std::cout << "The number of step is : " << num_step << endl;

}


void PID::UpdateParam(double Kp,double Ki ,double Kd){

	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	std::cout << "PID :" << "Kp:"<<Kp_ <<" Ki:" << Ki_ << " Kd:" << Kd_ << std::endl;

}
