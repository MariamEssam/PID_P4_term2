#include "PID.h"
#include <iostream>
#include <numeric>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() 
{
	isInitialized = false;
	IsTwiddled = false;
	step = 0;
	cte_totalsquaredarea = 0;
	tolerance = .0001;
	cte_totalarea = 0;
	dp = { 1,1,1 };
}

PID::~PID() {}

void PID::Init(double _Kp, double _Ki, double _Kd) {
	Kd = _Kd; 
	Kp = _Kp;
	Ki = _Ki;
}

void PID::UpdateError(double cte) {
	double delta_t =1;
	if (!isInitialized)
	{
		Init(.15, .0030, 2.90);
		previous_cte = cte;
		cte_totalarea = 0;
		isInitialized = true;
		
		besterror = UpdateCTEError(cte);
		NextTwiddleState = STATE_0;
	}
	else if (!IsTwiddled)
	{
		vector<double> p = { Kp,Ki,Kd };
		double err = UpdateCTEError(cte);
		switch (NextTwiddleState)
		{
		case STATE_0:
			ExecuteOp(OP_1,p);
			NextTwiddleState = STATE_1;
			break;
		case STATE_1:
			if (err < besterror)
			{
				besterror = err;
				ExecuteOp(OP_2,p);
				if (!AddIndex())
				{
					ExecuteOp(OP_1, p);
					NextTwiddleState = STATE_1;
				}
			}
			else
			{
				ExecuteOp(OP_3, p);
				NextTwiddleState = STATE_2;
			}
			break;
		case STATE_2:
			if (err < besterror)
			{
				besterror = err;
				ExecuteOp(OP_1,p);
			
			}
			else
			{
				ExecuteOp(OP_1, p);
				ExecuteOp(OP_4, p);
			}
			if (!AddIndex())
			{
				ExecuteOp(OP_1, p);
				NextTwiddleState = STATE_1;
			}
			break;
		}
		Kp = p[0];
		Ki = p[1];
		Kd = p[2];
		std::cout << endl << "kp: " << Kp << "KI: " << Ki << "Kd" << Kd << endl;
	}
	cte_totalarea += cte;
	p_error = -Kp*cte;
	d_error = -Kd*(cte - previous_cte)/delta_t;//we assume delta_t=1 
	i_error = -Ki*cte_totalarea;
	previous_cte = cte;
}
void PID::ExecuteOp(Operation op, std::vector<double>& p)
{
	int _index = index % 3;
	switch (op)
	{
	case OP_1:
		p[_index] += dp[_index];
		break;
	case OP_2:
		dp[_index] *= 1.1;
		break;
	case OP_3:
		p[_index] -= 2 * dp[_index];
		break;
	case OP_4:
		dp[_index] *= 0.9;
		break;
	}
	

}
bool PID::AddIndex()
{
	index++;
	double sum= std::accumulate(dp.begin(), dp.end(), 0);
	if (/*index % 3 == 0 && */sum < tolerance)
	{
		IsTwiddled = true;
	}
	return IsTwiddled;
}
double PID::TotalError() {
	return p_error+d_error+i_error;
}
double PID::UpdateCTEError(double cte)
{
	cte_totalsquaredarea += pow(cte, 2);
	return cte_totalsquaredarea / step;
}
double PID::Findsteer(double cte, double speed)
{
	step++;
	UpdateError(cte);
	return TotalError();
}
double PID::Findthrottle()
{
	if (!IsTwiddled)
		return 0.01;
	return 0.3;
}

