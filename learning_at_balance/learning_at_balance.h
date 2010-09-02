#include <math.h>
#include <lcmt_servotubeCommand.h>
#include <lcmt_servotubeState.h>

struct CP_State{
	double y;
	double yd;
	double theta;
	double thetad;
};

class LaBcontroller
{

public:
	LaBcontroller(){};
	LaBcontroller(double (*costFunc_in)(const double*, const double*),
		double* initialGains, double dt_in, unsigned int K_PARAM_DIM_IN) ; // Constructor
	~LaBcontroller(){}; // Destructor
	
	lcmt_servotubeCommand getBalancingCommand(CP_State state);
	lcmt_servotubeCommand getBalancingCommandwYLearning(CP_State state);
	double performUpdate();

	void useAlternativeUpdating();

	void setLearningParams(double baseline_in, double gamma_in, double eta_in, double* sigma_in);
	void setBaseline(double);
	void setGamma(double);
	void setEta(double);
	void setEta(double*);
	void setSigma(double*);
	void setCostChangeSaturation(double ccs_in);
	void useInternalCostFunc();

private:
	unsigned int  K_PARAM_DIM; //set up for cartpole
	double dt;
	double* curGains;
	double* gainPerturbs;

	double (*costFunc)(const double* x, const double* u);
	double internalCostFunction(const double* x, const double* u);
	void incrementCost(CP_State state, const double u);
	void perturbGains();

	void preUpdateOutput(double);
	void postUpdateOutput();

	double integratedCost;
	double costChangeSaturation;

	double baseline;
	double gamma;
	double* eta;
	double* sigma;

	//for learning with one baseline test per update (i.e., two episodes per update)
	bool isUsingAltUpdating;
	bool isBaselineEpisode;
	bool useInternalCostFunction;
	double beginBaselineEpisode();
	double beginUpdateEpisode();
};