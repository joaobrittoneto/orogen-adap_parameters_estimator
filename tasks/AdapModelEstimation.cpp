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

	if(handleMeasurement(forces))
	{
		//queueForces.push(forces);
		adapParam->Queue(10*sizeQueue, forces, queueForces);
	}
	//throw std::runtime_error("Transformer callback for forces_samples not implemented");
}

void AdapModelEstimation::pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
	base::samples::RigidBodyState transformed_rbs = pose_samples_sample;
	transformed_rbs.time = ts;

	// Convert velocities to body-frame
    if(transformed_rbs.sourceFrame == "body" && transformed_rbs.targetFrame != "body")
    	{
    		transformed_rbs.velocity = transformed_rbs.orientation.inverse() * transformed_rbs.velocity;
    	}

   // std::cout << std::endl << " pose_callback "<< std::endl;
    double step;
	if(handleMeasurement(transformed_rbs, step))
	{
		queuePose.push(transformed_rbs);
		queueStep.push(step);
		//adapParam->Queue(sizeQueue, transformed_rbs, queuePose);
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
	max_step												 = _sTime.get();
	double frequencyTau										 = _ftau.get();
	DOFS dof												 = _dofs.get();

	adapParam = new AdapParameters(gainLambda, gainA, dof, max_step, frequencyTau);

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


    // Update properties
    Eigen::Matrix<double, 6, 4, Eigen::DontAlign> gainLambda = _gLambda.get();
    Eigen::Matrix<double, 6, 1, Eigen::DontAlign> gainA		 = _gA.get();
    double frequencyTau										 = _ftau.get();
    DOFS dof												 = _dofs.get();

   	adapParam->configure(gainLambda, gainA, dof, frequencyTau);


   	//Inputs
    base::samples::RigidBodyState inputSpeed;
    base::samples::Joints inputForces;

    //Convert inputs into vectors
    base::Vector6d velocity;
    base::Vector6d forces;

    //Outputs
    double deltaV;
    double normDeltaV;
    base::Vector4d parameters;

    if(!queuePose.empty() && !queueForces.empty())
    {
     	inputSpeed = queuePose.front();
    	queuePose.pop();

    	double step;

     	step = queueStep.front();
    	queueStep.pop();



    	if(matchForce(inputSpeed, inputForces))
    		{

    			getVectorForces(inputForces, forces);
    			getVectorVelocities(inputSpeed, velocity);

    			std::cout << " new sample "<< (inputSpeed.time - inputForces.time).toSeconds() << " " << std::endl;
    			std::cout << " queuePose.size "<< queuePose.size() << " queueForces.size "<< queueForces.size() << std::endl;

    			adapParam->update_step(step);
    			std::cout << " step " << step << std::endl;
    			adapParam->parameters_estimation(forces, velocity, parameters, deltaV, normDeltaV);
    			convertParameters(parameters, modelParameters, dof);

    			_parameters.write(modelParameters);
    			_deltaV.write(deltaV);
    			_normDeltaV.write(normDeltaV);
    		}
    	else
    		std::cout << " NO new sample "<< std::endl;

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

bool AdapModelEstimation::matchForce(const base::samples::RigidBodyState &pose_sample, base::samples::Joints &forces_output)
{
	base::samples::Joints front_force;
	base::samples::Joints behind_force;
	behind_force.time = base::Time::fromSeconds(1);
	bool stop = false;
	std::queue<base::samples::Joints> queueofForces = queueForces;
	//match force with pose
	while(!queueofForces.empty() && !stop)
	{
		front_force = queueofForces.front();
		queueofForces.pop();

		// Pose sample before queue
		if( (pose_sample.time-front_force.time).toSeconds() < 0 && behind_force.time == base::Time::fromSeconds(1) )
		{
			behind_force = front_force;
			//front_force = queueofForces.front();
			forces_output = front_force;
			stop = true;
		}

		// Pose sample after queue
		else if ((pose_sample.time-front_force.time).toSeconds() > 0 && queueofForces.empty())
			{
				forces_output = front_force;
				stop = true;
			}

		// Pose in the middle of queue
		if( (pose_sample.time-front_force.time).toSeconds() > 0)
			behind_force = front_force;
		else
			{
				if((pose_sample.time-front_force.time).toSeconds() > (behind_force.time - pose_sample.time).toSeconds() )
					forces_output = front_force;
				else
					forces_output = behind_force;
				stop = true;
			}



//		if( (pose_sample.time-front_force.time).toSeconds() > 0 && !queueofForces.empty() )
//			behind_force = front_force;
//
//		// Check if the behind_force was initialized, otherwise interpolate the last two forces
//		else if ( behind_force.time == base::Time::fromSeconds(1) )
//		{
//			behind_force = front_force;
//			front_force = queueofForces.front();
//			stop = true;
//		}
//		else
//			stop = true;
	}

	return stop;
}

bool AdapModelEstimation::getVectorForces(const base::samples::Joints &forces_sample, base::Vector6d &forces)
{
	if(forces_sample.elements.size()!=6)
	{
		std::cout<<std::endl<< "Make sure the input forces have the right dimention" <<std::endl;
		return false;
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			// Converting from base::samples::Joints to base::Vector6d
			forces[i] = forces_sample.elements[i].effort;
		}
	}
	return true;
}

void AdapModelEstimation::getVectorVelocities(const base::samples::RigidBodyState &pose_sample, base::Vector6d &velocities)
{
	for (int i = 0; i < 6; i++)
	{
		// Converting from base::samples::Joints to base::Vector6d
		if (i < 3)
			velocities[i] = pose_sample.velocity[i];
		else
			velocities[i] = pose_sample.angular_velocity[i-3];
	}
}

void AdapModelEstimation::convertParameters(const base::Vector4d &parameters, Parameters &modelParameters, DOFS dof)
{
	modelParameters.inertiaCoeff[dof].positive           = parameters[0];
	modelParameters.quadraticDampingCoeff[dof].positive  = parameters[1];
	modelParameters.linearDampingCoeff[dof].positive     = parameters[2];
	modelParameters.gravityAndBuoyancy[dof]              = parameters[3];
}

bool AdapModelEstimation::handleMeasurement(const base::samples::RigidBodyState &rbs_sample, double &step)
{
	for (int j=0; j<3; j++)
	{
		if( isnan((double)rbs_sample.velocity[j]) || isnan((double)rbs_sample.angular_velocity[j]) )
			{
				std::cout << "VEl is nan "<< std::endl;
				return false;
			}
	}

	if(lastPoseSample.time > rbs_sample.time)
		{
			std::cout << "pose.time is Bigger  "<< std::endl;
			return false;
		}
	if(lastPoseSample.time == rbs_sample.time)
		{
			std::cout << "pose.time is Equal  "<< std::endl;
			return false;
		}

	static double auxstep2 = 0;
	double auxstep = (rbs_sample.time - lastPoseSample.time).toSeconds();
	double deltaVel = (rbs_sample.velocity[0] - lastPoseSample.velocity[0]);


	//step = 0.01;
//	if(step < max_step)
//		{
//			adapParam->update_step(step);
//			std::cout << " step " << step << std::endl;
////			if(_ftau.get() > 0)
////				{
////				int n = (2*3.141592/_ftau.get()) / step;
////				std::cout << " size queue " << n << std::endl;
////				}
//
//		}




	step = auxstep;

	std::cout << "step "<< step << " delta_Vel "<< deltaVel << std::endl;
	//step = 0.01;
	lastPoseSample = rbs_sample;

	//if(auxstep > 1)
	//	return false;

	//auxstep2 = auxstep2 + auxstep;

	if(auxstep > max_step)
	{
		std::cout << "Step too big "<< std::endl;
		return false;
	}

	//step = auxstep2;
	//std::cout << "step2 "<< step << " delta_Vel "<< deltaVel << std::endl;
	//auxstep2 = 0;

	return true;

}

bool AdapModelEstimation::handleMeasurement(const base::samples::Joints &force_sample)
{
	for (int j=0; j < force_sample.elements.size(); j++)
	{
		if( isnan((double)force_sample.elements[j].effort))
			return false;
	}

	return true;

}


