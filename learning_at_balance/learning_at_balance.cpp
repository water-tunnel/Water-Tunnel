#include "learning_at_balance.h"
double randn_notrig(double , double );

//fixed gain state estimator
LaBcontroller::LaBcontroller (double (*costFunc_in)(const double*, const double*), 
							  double* initialGains, double dt_in, unsigned int K_PARAM_DIM_IN)
{
	costFunc=costFunc_in;
	integratedCost=0.0;

	K_PARAM_DIM=K_PARAM_DIM_IN;
	dt=dt_in;

	//set initial gains
	curGains=new double[ K_PARAM_DIM];
	gainPerturbs=new double[ K_PARAM_DIM];

	for(unsigned int i=0;i< K_PARAM_DIM;i++)
	{
		curGains[i]=initialGains[i];
		gainPerturbs[i] = 0.0;
	}

	costChangeSaturation = .5;

	isUsingAltUpdating = false;
	isBaselineEpisode = false;
	useInternalCostFunction=false;

	
	//these default values are tuned for our learning at balance
	double baseline_tmp = 5;
	double gamma_tmp = .95;
	double eta_tmp = 2;
	double* sigma_tmp=new double[ K_PARAM_DIM];

	for(unsigned int i=0;i< K_PARAM_DIM;i++)
		sigma_tmp[i]=.03;
	if(K_PARAM_DIM==5) //var on y position
		sigma_tmp[4] = .0035;

	sigma = new double[ K_PARAM_DIM];
	eta = new double[K_PARAM_DIM];

	setLearningParams(baseline_tmp, gamma_tmp, eta_tmp, sigma_tmp);
	//perturbGains();
}

lcmt_servotubeCommand LaBcontroller::getBalancingCommand(CP_State state)
{
	lcmt_servotubeCommand myComm;
	myComm.commandType = 1; //acceleration command

	myComm.commandValue = -curGains[0]*state.y -curGains[1]*state.theta
		-curGains[2]*state.yd -curGains[3]*state.thetad;

	incrementCost(state ,myComm.commandValue);

	return myComm;
}

lcmt_servotubeCommand LaBcontroller::getBalancingCommandwYLearning(CP_State state)
{
	lcmt_servotubeCommand myComm;
	myComm.commandType = 1; //acceleration command

	myComm.commandValue = -curGains[0]*(state.y - curGains[4]) -curGains[1]*state.theta
		-curGains[2]*state.yd -curGains[3]*state.thetad;

	incrementCost(state ,myComm.commandValue);

	return myComm;
}

void LaBcontroller::incrementCost(CP_State state, const double u)
{
	double x[] = {state.y, state.theta, state.yd, state.thetad};

	if(useInternalCostFunction)
		integratedCost += dt*internalCostFunction(x, &u);
	else
		integratedCost += dt*costFunc(x, &u);
}

void LaBcontroller::useInternalCostFunc()
{
	useInternalCostFunction = true;
}

double LaBcontroller::internalCostFunction(const double* x, const double* u)
{
	double R= 10;
	double diagQ[] ={20, 5, 2, .5};

	int STATE_DIM = 4;

	double cost=R*pow((*u),2);

	cost += diagQ[0]*pow(x[0]-curGains[4],2);
	for(int i=1; i<STATE_DIM; i++)
		cost +=diagQ[i]*pow(x[i],2);

	return cost;
}

void LaBcontroller::setLearningParams(double baseline_in, double gamma_in, double eta_in, double* sigma_in)
{
	setBaseline(baseline_in);
	setGamma(gamma_in);
	setEta(eta_in);
	setSigma(sigma_in);
}

void LaBcontroller::setBaseline(double baseline_in)
{
	baseline = baseline_in;
}

void LaBcontroller::setGamma(double gamma_in)
{
	gamma=gamma_in;
}

void LaBcontroller::setEta(double eta_in)
{
	for(unsigned int i=0;i< K_PARAM_DIM;i++)
		eta[i]=eta_in;
}

void LaBcontroller::setEta(double* eta_in)
{
	for(unsigned int i=0;i< K_PARAM_DIM;i++)
		eta[i]=eta_in[i];
}

void LaBcontroller::setSigma(double* sigma_in)
{
	for(unsigned int i=0;i< K_PARAM_DIM;i++)
		sigma[i]=sigma_in[i];
}

void LaBcontroller::setCostChangeSaturation(double ccs_in)
{
	costChangeSaturation=ccs_in;
}

double LaBcontroller::beginBaselineEpisode() //end update episode
{
	isBaselineEpisode = true;

	double costChange;
	double integratedCostOut = integratedCost;

	costChange = integratedCost - baseline;
	integratedCost = 0.0; //reset cost for new episode

	if(costChange > costChangeSaturation)
		costChange = costChangeSaturation;
	else if(costChange < -costChangeSaturation)
		costChange = -costChangeSaturation;

	preUpdateOutput(costChange);
	for(unsigned int i=0;i< K_PARAM_DIM;i++)
	{
		curGains[i]+= -(eta[i]*costChange + 1)*gainPerturbs[i];//extra 1 to undo original perturbation
	}
	postUpdateOutput();

	return integratedCostOut;
}

double LaBcontroller::beginUpdateEpisode() //end baseline episode
{
	isBaselineEpisode = false;

	double integratedCostOut = integratedCost;

	baseline = integratedCost; //update baseline
	integratedCost = 0.0; //reset cost for new episode

	perturbGains();

	return integratedCostOut;
}

void LaBcontroller::useAlternativeUpdating()
{
	isUsingAltUpdating = true;
	isBaselineEpisode = true;
}

double LaBcontroller::performUpdate()
{
	if(isUsingAltUpdating)
	{
		if(isBaselineEpisode)
			return beginUpdateEpisode();
		else
			return beginBaselineEpisode();
	}
	else
	{
		double costChange;
		double integratedCostOut = integratedCost;

		costChange = integratedCost - baseline;

		if(costChange > costChangeSaturation)
			costChange = costChangeSaturation;
		else if(costChange < -costChangeSaturation)
			costChange = -costChangeSaturation;


		baseline = integratedCost*(1-gamma) + gamma*baseline; //update baseline
		integratedCost = 0.0; //reset cost for new episode

		for(unsigned int i=0;i< K_PARAM_DIM;i++)
			curGains[i]+= -eta[i]*gainPerturbs[i]*costChange;

		perturbGains();

		return integratedCostOut;
	}
}


void LaBcontroller::perturbGains()
{
	for(unsigned int i=0;i< K_PARAM_DIM;i++)
	{
		gainPerturbs[i] = sigma[i]*randn_notrig(0.0, 1.0);
		curGains[i]+= gainPerturbs[i];
	}
}

void LaBcontroller::preUpdateOutput(double costChange)
{
	printf("Old nominal gains: ");
	for(unsigned int i=0;i< K_PARAM_DIM;i++)
		printf("K%d: %2.3f ",i,curGains[i]-gainPerturbs[i]);
	printf("\n");

	printf("Gain perturbation: ");
	for(unsigned int i=0;i< K_PARAM_DIM;i++)
		printf("K%d: %2.3f ",i,gainPerturbs[i]);
	printf("\n");

	printf("Baseline Cost: %3.3f  Perturbed Cost: %3.3f\n",baseline,baseline+costChange);
}

void LaBcontroller::postUpdateOutput()
{
	printf("New nominal gains: ");
	for(unsigned int i=0;i< K_PARAM_DIM;i++)
		printf("K%d: %2.3f ",i,curGains[i]);
	printf("\n\n");
}

double randn_notrig(double mu=0.0, double sigma=1.0) {
	static bool deviateAvailable=false;	//	flag
	static float storedDeviate;			//	deviate from previous calculation
	double polar, rsquared, var1, var2;
	
	//	If no deviate has been stored, the polar Box-Muller transformation is 
	//	performed, producing two independent normally-distributed random
	//	deviates.  One is stored for the next round, and one is returned.
	if (!deviateAvailable) {
		
		//	choose pairs of uniformly distributed deviates, discarding those 
		//	that don't fall within the unit circle
		do {
			var1=2.0*( double(rand())/double(RAND_MAX) ) - 1.0;
			var2=2.0*( double(rand())/double(RAND_MAX) ) - 1.0;
			rsquared=var1*var1+var2*var2;
		} while ( rsquared>=1.0 || rsquared == 0.0);
		
		//	calculate polar tranformation for each deviate
		polar=sqrt(-2.0*log(rsquared)/rsquared);
		
		//	store first deviate and set flag
		storedDeviate=var1*polar;
		deviateAvailable=true;
		
		//	return second deviate
		return var2*polar*sigma + mu;
	}
	
	//	If a deviate is available from a previous call to this function, it is
	//	returned, and the flag is set to false.
	else {
		deviateAvailable=false;
		return storedDeviate*sigma + mu;
	}
}