/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace adap_parameters_estimator;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
	delete adapParam;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    else
    {
    	sampTime = _sTime.get();
    	frequencyTau = _ftau.get();
    	gainLambda = _gLambda.get();
    	gainA = _gA.get();
    	dof = _dofs.get();


    	adapParam = new AdapParameters(sampTime, frequencyTau, gainLambda, gainA, dof);

    	interaction = 0;
    	//deltaV = 0;

    	return true;
    }
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    Parameters modelParameters;
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

    base::samples::RigidBodyState inputSpeed;
    base::samples::Joints inputThruster;

    base::Vector6d velocity;
    base::Vector6d tau;

    double deltaV;

	if (_speed_samples.readNewest(inputSpeed) == RTT::NewData)
	{
		// Converting from base::samples::RigidBodyStates to base::Vector6d
		for (int i = 0; i < 6; i++)
		{
			if (i < 3)
				velocity[i] = inputSpeed.velocity[i];

			else
				velocity[i] = inputSpeed.angular_velocity[i-3];

		}
	}

	if (_thruster_samples.readNewest(inputThruster) == RTT::NewData)
		{
			// Converting from base::samples::Joints to base::Vector6d
			for (int i = 0; i < 6; i++)
				tau[i] = inputThruster.elements[i].effort;

		}


	adapParam->delta_velocity(velocity);
	adapParam->estimated_state(tau);
	adapParam->euler_velocity();
	adapParam->euler_parameters();
	adapParam->convetional_parameters();
	adapParam->filter_parameters(interaction);

	interaction++;
	//deltaV =  adapParam->get_delta_v();

	//std::cout << std::endl << "===================="<< std::endl;
	//std::cout << std::endl << "DELTA_v: "<< adapParam->get_delta_v() << std::endl << std::endl;
	//std::cout << std::endl << "estPhi: " << adapParam->get_est_phi() << std::endl << std::endl;
	//std::cout << std::endl << "adapParam fTau: "<< adapParam->get_fTau() << std::endl << std::endl;
	//std::cout << std::endl << "Estimated States: "<< std::endl << adapParam->get_est_states() << std::endl << std::endl;
	//std::cout << std::endl << "estVelocity: "<< std::endl << adapParam->get_est_velocity() << std::endl << std::endl;
	//std::cout << std::endl << "Filtered Parameters: "<< std::endl << adapParam->get_filtered_parameters() << std::endl << std::endl;
	//std::cout << std::endl << "Parameters: "<< std::endl << adapParam->get_parameters() << std::endl << std::endl;

	//std::cout << std::endl << "REAL velocity: "<< velocity[dof] << std::endl << std::endl;

	modelParameters.inertiaCoeff[dof].positive           = adapParam->get_filtered_parameters()[0];
	modelParameters.quadraticDampingCoeff[dof].positive  = adapParam->get_filtered_parameters()[1];
	modelParameters.linearDampingCoeff[dof].positive     = adapParam->get_filtered_parameters()[2];
	modelParameters.gravityAndBuoyancy[dof]              = adapParam->get_filtered_parameters()[3];
/*
	modelParameters.inertiaCoeff[dof].positive           = adapParam->get_parameters()[0];
	modelParameters.quadraticDampingCoeff[dof].positive  = adapParam->get_parameters()[1];
	modelParameters.linearDampingCoeff[dof].positive     = adapParam->get_parameters()[2];
	modelParameters.gravityAndBuoyancy[dof]              = adapParam->get_parameters()[3];
*/
	//std::cout << std::endl << "teste2 TASK.CPP: "<< std::endl << modelParameters.gravityAndBuoyancy[dof] << std::endl << std::endl;

	_parameters.write(modelParameters);

	deltaV = adapParam->get_delta_v();
	_deltaV.write(deltaV);

}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

double Task::get_deltaV()
{
	return adapParam->get_delta_v();
}



