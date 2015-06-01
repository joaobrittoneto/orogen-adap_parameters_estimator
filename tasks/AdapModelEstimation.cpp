/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AdapModelEstimation.hpp"

using namespace adap_parameters_estimator;

AdapModelEstimation::AdapModelEstimation(std::string const& name)
    : AdapModelEstimationBase(name)
{
}

AdapModelEstimation::AdapModelEstimation(std::string const& name, RTT::ExecutionEngine* engine)
    : AdapModelEstimationBase(name, engine)
{
}

AdapModelEstimation::~AdapModelEstimation()
{
	delete adapParam;
}

void AdapModelEstimation::forces_samplesCallback(const base::Time &ts, const ::base::samples::Joints &forces_samples_sample)
{

	base::samples::Joints forces = forces_samples_sample;

	bool isvelnan = false;
	for (int j=0; j < forces.elements.size(); j++)
		{
			if( isnan((double)forces.elements[j].effort))
				isvelnan = true;
		}
	if(!isvelnan)
		{
			adapParam->Queue(sizeQueue, forces, queueForces);
		}
	//throw std::runtime_error("Transformer callback for forces_samples not implemented");
}

void AdapModelEstimation::pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
	base::samples::RigidBodyState transformed_rbs = pose_samples_sample;


	// Convert velocities to body-frame
    if(transformed_rbs.sourceFrame == "body" && transformed_rbs.targetFrame != "body")
    	{
    		transformed_rbs.velocity = transformed_rbs.orientation.inverse() * transformed_rbs.velocity;
    	}

    bool isvelnan = false;
	for (int j=0; j<3; j++)
	{
		if( isnan((double)transformed_rbs.velocity[j]) || isnan((double)transformed_rbs.angular_velocity[j]) )
			isvelnan = true;
	}
	if(!isvelnan)
	{
		adapParam->Queue(sizeQueue, transformed_rbs, queuePose);
	}

	//throw std::runtime_error("Transformer callback for pose_samples not implemented");
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AdapModelEstimatation.hpp for more detailed
// documentation about them.

bool AdapModelEstimation::configureHook()
{
    if (! AdapModelEstimationBase::configureHook())
        return false;

    Eigen::Matrix<double, 6, 4, Eigen::DontAlign> gainLambda = _gLambda.get();
	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> gainA		 = _gA.get();
	double sampTime											 = _sTime.get();
	double frequencyTau										 = _ftau.get();
	DOFS dof												 = _dofs.get();

	adapParam = new AdapParameters(gainLambda, gainA, dof, sampTime, frequencyTau);

	if (!adapParam->gainsOk)
	{
		std::cout << std::endl << " Gains are not OK. Verify gains values. "<< std::endl;
	}

	// Initialize output parameters
	for (int i=0; i<6; i++)
						{
						 modelParameters.inertiaCoeff[i].positive = 1;
						 modelParameters.inertiaCoeff[i].negative = 1;
						 modelParameters.quadraticDampingCoeff[i].positive = 0;
						 modelParameters.quadraticDampingCoeff[i].negative = 0;
						 modelParameters.linearDampingCoeff[i].positive = 0;
						 modelParameters.linearDampingCoeff[i].negative = 0;
						}
	modelParameters.gravityAndBuoyancy = base::VectorXd::Zero(6);
	modelParameters.coriolisCentripetalMatrix = base::MatrixXd::Zero(6,6);


	return true;

}
bool AdapModelEstimation::startHook()
{
    if (! AdapModelEstimationBase::startHook())
        return false;
    return true;
}
void AdapModelEstimation::updateHook()
{
    AdapModelEstimationBase::updateHook();

    bool doIt = false;

    // Update properties
    Eigen::Matrix<double, 6, 4, Eigen::DontAlign> gainLambda = _gLambda.get();
    Eigen::Matrix<double, 6, 1, Eigen::DontAlign> gainA		 = _gA.get();
    double sampTime											 = _sTime.get();
    double frequencyTau										 = _ftau.get();
    DOFS dof												 = _dofs.get();

   	adapParam->configure(gainLambda, gainA, dof, sampTime, frequencyTau);


   	//Inputs
    base::samples::RigidBodyState inputSpeed;
    base::samples::Joints inputForces;

    //Convert inputs into vectors
    base::Vector6d velocity;
    base::Vector6d forces;
    base::VectorXd thruster;

    //Outputs
    double deltaV;
    double normDeltaV;
    base::Vector4d parameters;

    if(!queuePose.empty() && !queueForces.empty())
    {
    	inputSpeed = queuePose.back();
    	inputForces = queueForces.back();

		// Checking forces and torques
		if(inputForces.elements.size()!=6)
		{
			std::cout<<std::endl<< "Make sure the input forces have the right dimention" <<std::endl;
		}
		else
		{
			for (int i = 0; i < 6; i++)

			{
				// Converting from base::samples::Joints to base::Vector6d
				forces[i] = inputForces.elements[i].effort;

				// Converting from base::samples::Joints to base::Vector6d
				if (i < 3)
					velocity[i] = inputSpeed.velocity[i];
				else
					velocity[i] = inputSpeed.angular_velocity[i-3];
			}

			adapParam->parameters_estimation(forces, velocity, parameters, deltaV, normDeltaV);


			modelParameters.inertiaCoeff[dof].positive           = parameters[0];
			modelParameters.quadraticDampingCoeff[dof].positive  = parameters[1];
			modelParameters.linearDampingCoeff[dof].positive     = parameters[2];
			modelParameters.gravityAndBuoyancy[dof]              = parameters[3];


			_parameters.write(modelParameters);
			_deltaV.write(deltaV);
			_normDeltaV.write(normDeltaV);
		}

    }


}
void AdapModelEstimation::errorHook()
{
    AdapModelEstimationBase::errorHook();
}
void AdapModelEstimation::stopHook()
{
    AdapModelEstimationBase::stopHook();
}
void AdapModelEstimation::cleanupHook()
{
    AdapModelEstimationBase::cleanupHook();
}
