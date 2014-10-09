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
    	gainLambda = _gLambda.get();
    	gainA = _gA.get();
    	thrusterMatrix = _thrusterMatrix.get();
    	dof = _dofs.get();
    	sampTime = _sTime.get();
    	frequencyTau = _ftau.get();


    	adapParam = new AdapParameters(gainLambda, gainA, thrusterMatrix, dof, sampTime, frequencyTau);


    	if (!adapParam->gainsOk)
    	{
    		std::cout << std::endl << " Gains are not OK. Verify gains values. "<< std::endl;
       	}

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

    Eigen::Matrix<double, 6, 4, Eigen::DontAlign> gainLambda_temp = gainLambda;
    Eigen::Matrix<double, 6, 1, Eigen::DontAlign> gainA_temp = gainA;
    double frequencyTau_temp = frequencyTau;
    DOFS dof_temp = dof;


    gainLambda = _gLambda.get();
   	gainA = _gA.get();
   	thrusterMatrix = _thrusterMatrix.get();
   	dof = _dofs.get();
   	sampTime = _sTime.get();
   	frequencyTau = _ftau.get();

   	if (gainLambda_temp != gainLambda || gainA_temp != gainA || frequencyTau_temp != frequencyTau || dof_temp != dof )
   	{
   		std::cout << std::endl << "update configure: "<< std::endl;
   		adapParam->configure(gainLambda, gainA, thrusterMatrix, dof, sampTime, frequencyTau);
   		std::cout << "update configure2: "<< std::endl << std::endl;
   	}



   	if (!adapParam->gainsOk)
   	{
   		std::cout << std::endl << " Gains are not OK. Verify gains values. "<< std::endl;
   	}


    base::samples::RigidBodyState inputSpeed;
    base::samples::Joints inputThruster;

    base::Vector6d velocity;
    base::VectorXd thruster;
    //thruster.resize(thrusterMatrix.size()/6); //In case the input is the forces applied for each thruster
    thruster.resize(6); // In case the input is the forces and torques applied direct to the auv

    double deltaV;
    double normDeltaV;

    if (!adapParam->definedDof)
    	{
    		adapParam->establish_dof(thruster, velocity);
    	}


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
		//std::cout << std::endl << "velocity: "<< velocity << std::endl << std::endl;
	}


	if(_thruster_samples.readNewest(inputThruster) == RTT::NewData)
	{
		int numberOfThruster = inputThruster.elements.size();
		// Converting from base::samples::Joints to base::Vector6d
		for (int i = 0; i < numberOfThruster; i++)
				thruster[i] = inputThruster.elements[i].effort;
	}

	adapParam->parameters_estimation(thruster, velocity);



/*	if (_thruster_samples.readNewest(inputThruster) == RTT::NewData)
		{
			int numberOfThruster = inputThruster.elements.size();
			std::cout << std::endl << "tnumberOfThruster: "<< numberOfThruster << std::endl << std::endl;
			// Converting from base::samples::Joints to base::Vector6d
			for (int i = 0; i < numberOfThruster; i++)
				thruster[i] = inputThruster.elements[i].effort;

		}
*/


	/*adapParam->delta_velocity(velocity);
	adapParam->estimated_state(tau);
	adapParam->euler_velocity();
	adapParam->euler_parameters();
	adapParam->convetional_parameters();
	*/
//	adapParam->parameters_estimation(thruster, velocity);
//	adapParam->filter_parameters(interaction);

//	interaction++;
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

	//std::cout << std::endl << "inertia: "<< modelParameters.inertiaCoeff[dof].positive << std::endl << std::endl;
	_parameters.write(modelParameters);

	deltaV = adapParam->get_delta_v();
	normDeltaV = adapParam->get_norm_delta_v();

	_deltaV.write(deltaV);
	_normDeltaV.write(normDeltaV);


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




