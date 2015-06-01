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
    	Eigen::Matrix<double, 6, 4, Eigen::DontAlign> gainLambda = _gLambda.get();
    	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> gainA		 = _gA.get();
    	Eigen::MatrixXd thrusterMatrix							 = _thrusterMatrix.get();
    	double sampTime											 = _sTime.get();
    	double frequencyTau										 = _ftau.get();
    	DOFS dof												 = _dofs.get();
    	aligned_data											 = _aligned_data.get();
		body_forces 											 =_body_forces.get();

    	adapParam = new AdapParameters(gainLambda, gainA, thrusterMatrix, dof, sampTime, frequencyTau);

    	if (!adapParam->gainsOk)
    	{
    		std::cout << std::endl << " Gains are not OK. Verify gains values. "<< std::endl;
       	}

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

    bool doIt = false;

    static bool first_time = true;
    static Parameters modelParameters;

    if(first_time)
    {
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
    	first_time = false;
    }

    Eigen::Matrix<double, 6, 4, Eigen::DontAlign> gainLambda = _gLambda.get();
    Eigen::Matrix<double, 6, 1, Eigen::DontAlign> gainA		 = _gA.get();
    Eigen::MatrixXd thrusterMatrix							 = _thrusterMatrix.get();
    double sampTime											 = _sTime.get();
    double frequencyTau										 = _ftau.get();
    DOFS dof												 = _dofs.get();

   	adapParam->configure(gainLambda, gainA, thrusterMatrix, dof, sampTime, frequencyTau);


   	//Inputs
    base::samples::RigidBodyState inputSpeed;
    base::samples::Joints inputThruster;
    adap_samples_input::DynamicAUV dynamic;

    //Convert inputs into vectors
    base::Vector6d velocity;
    base::Vector6d forces;
    base::VectorXd thruster;



    //Outputs
    double deltaV;
    double normDeltaV;
    base::Vector4d parameters;

    if (dof == UNINITIALISED)
    	{
    		adapParam->establish_dof(thruster, velocity); //TODO verify method
    	}

    if(!aligned_data)
    {
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

		if(_thruster_samples.readNewest(inputThruster) == RTT::NewData)
		{
			if(body_forces)
			{
				// Converting from base::samples::Joints to base::Vector6d
				if(inputThruster.elements.size()!=6)
				{
					std::cout<<std::endl<< "Make sure the input forces have the right dimention" <<std::endl;
				}
				for (int i = 0; i < inputThruster.elements.size(); i++)
					forces[i] = inputThruster.elements[i].effort;
			}
			else if(!body_forces)
			{
				for (int i = 0; i < inputThruster.elements.size(); i++)
					{thruster[i] = inputThruster.elements[i].effort;}
					adapParam->forces_torques(thruster,forces);
			}
		}
    }

    else if(aligned_data)
    {
		if(_dynamic_samples.read(dynamic) == RTT::NewData)
		{
			// Converting from base::samples::RigidBodyStates to base::Vector6d
			for (int i = 0; i < 6; i++)
			{
				if (i < 3)
					velocity[i] = dynamic.rbs.velocity[i];
				else
					velocity[i] = dynamic.rbs.angular_velocity[i-3];
			}

			if(body_forces)
			{
				// Converting from base::samples::Joints to base::Vector6d
				if(dynamic.joints.elements.size()!=6)
				{
					std::cout<<std::endl<< "Make sure the input forces have the right dimension" <<std::endl;
				}
				for (int i = 0; i < dynamic.joints.elements.size(); i++)
					forces[i] = dynamic.joints.elements[i].effort;

				doIt = true;
			}
			else if(!body_forces)
			{
				for (int i = 0; i < dynamic.joints.elements.size(); i++)
					{thruster[i] = dynamic.joints.elements[i].effort;}
					adapParam->forces_torques(thruster,forces);
					doIt = true;
			}
		}
    }

    if(doIt)
	{
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




