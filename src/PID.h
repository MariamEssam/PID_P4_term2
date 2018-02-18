#ifndef PID_H
#define PID_H
#include  <ctime>
#include  <vector>
class PID {
public:
	enum TwiddleState {
		STATE_1 = 1,
		STATE_0 = 0,
		STATE_2 = 2,
	};
	enum Operation {
		OP_1,
		OP_2,
		OP_3,
		OP_4
	};
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  std::vector<double> dp;
  bool isInitialized;
  bool IsTwiddled;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double Findsteer(double cte, double speed);
  double Findthrottle();
  double previous_cte;
  double cte_totalarea;
  double cte_totalsquaredarea;
  double besterror;
  double step;
  int index;
  double tolerance;
  double UpdateCTEError(double cte);
  void ExecuteOp(Operation op,std::vector<double>& p);
  TwiddleState NextTwiddleState;
  bool AddIndex();
};
#endif /* PID_H */
