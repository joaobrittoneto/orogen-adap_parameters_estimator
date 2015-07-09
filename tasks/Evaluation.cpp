/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Evaluation.hpp"

using namespace adap_parameters_estimator;

Evaluation::Evaluation(std::string const& name)
    : EvaluationBase(name)
{
}

Evaluation::Evaluation(std::string const& name, RTT::ExecutionEngine* engine)
    : EvaluationBase(name, engine)
{
}

Evaluation::~Evaluation()
{
}


void Evaluation::model_velocityCallback(const base::Time &ts, const ::base::samples::RigidBodyState &model_velocity_sample)
{

	base::samples::RigidBodyState model = model_velocity_sample;

	// Convert velocities to body-frame
    if(model.sourceFrame == "body" && model.targetFrame != "body")
	{
    	model.velocity = model.orientation.inverse() * model.velocity;
	}

    if(handleModel(model))
	{
		queueModel.push(model);
		//adapParam->Queue(sizeQueue, transformed_rbs, queuePose);
	}


	//throw std::runtime_error("Transformer callback for forces_samples not implemented");
}

void Evaluation::measured_velocityCallback(const base::Time &ts, const ::base::samples::RigidBodyState &measured_velocity_sample)
{
	base::samples::RigidBodyState measured = measured_velocity_sample;

	// Convert velocities to body-frame
    if(measured.sourceFrame == "body" && measured.targetFrame != "body")
	{
    	measured.velocity = measured.orientation.inverse() * measured.velocity;
	}

    if(handleMeasurement(measured))
	{
		queueMeasured.push(measured);
		if(queueMeasured.size() > max_queue_size)
			queueMeasured.pop();
	}

	//throw std::runtime_error("Transformer callback for pose_samples not implemented");
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Evaluation.hpp for more detailed
// documentation about them.

bool Evaluation::configureHook()
{
    if (! EvaluationBase::configureHook())
        return false;
    else
	   {
		dof = _dof.get();

		return true;
	   }
}
bool Evaluation::startHook()
{
    if (! EvaluationBase::startHook())
        return false;
    return true;
}
void Evaluation::updateHook()
{
    EvaluationBase::updateHook();


	//static int size_model = 0;
	//static int size_measurements = 0;
	static int size = 0;
	static double mae_velocity = 0;
	static double mean_velocity = 0;

	double error_velocity = 0;
	double norm_mae_velocity = 0;
	double model = 0;
	double measurement = 0;

   	//Inputs
    base::samples::RigidBodyState inputModel;
    base::samples::RigidBodyState inputMeasured;

    static bool first_time = true;



	if(!queueMeasured.empty() && !queueModel.empty())
	{
		// Don't get the first model sample. Delay analysis by one sample in order to get a better matchPose
		if(first_time)
			{first_time = false;}
		else
		{
			inputModel = queueModel.front();
			queueModel.pop();

			if(matchPose(inputModel, inputMeasured))
			{

				std::cout << " new sample "<< (inputModel.time - inputMeasured.time).toSeconds() << " " << std::endl;
				//std::cout << " queueModel.size "<< queueModel.size() << " queueMeasured.size "<< queueMeasured.size() << std::endl;

				if (dof < 3)
					{model= inputModel.velocity[dof]; measurement = inputMeasured.velocity[dof];}
				else
					{model= inputModel.angular_velocity[dof-3]; measurement = inputMeasured.angular_velocity[dof-3];}

				size++;

				error_velocity = measurement - model;
				mean_velocity = (mean_velocity*(size-1) + fabs(measurement))/size;
				mae_velocity = (mae_velocity*(size-1) + fabs(error_velocity))/size;
				norm_mae_velocity = mae_velocity / mean_velocity;

				_error_velocity.write(error_velocity);
				_mae_velocity.write(mae_velocity);
				_norm_mae_velocity.write(norm_mae_velocity);
			}
		}
	}
	//else if(queueOfMeasurements.empty() || queueOfModel.empty())
	//	std::cout << std::endl << "empty queue "<< std::endl;

	//std::cout << std::endl << "MAE_Vel: "<< mae_velocity << std::endl;
	//std::cout << std::endl << "Norm_MAE_Vel: "<< norm_mae_velocity << std::endl;

	//std::cout << std::endl << "size_model "<< size_model << std::endl;
	//std::cout << std::endl << "vModel "<< Model.velocity[dof] << std::endl;


}
void Evaluation::errorHook()
{
    EvaluationBase::errorHook();
}
void Evaluation::stopHook()
{
    EvaluationBase::stopHook();
}
void Evaluation::cleanupHook()
{
    EvaluationBase::cleanupHook();
}


bool Evaluation::handleModel(const base::samples::RigidBodyState &sample)
{
	for (int j=0; j<3; j++)
	{
		if( isnan((double)sample.velocity[j]) || isnan((double)sample.angular_velocity[j]) )
			{
				std::cout << "model VEl is nan "<< std::endl;
				return false;
			}
	}

	if(lastModelSample.time > sample.time)
		{
			//std::cout << "model pose.time is Bigger  "<< std::endl;
			return false;
		}
	if(lastModelSample.time == sample.time)
		{
			//std::cout << "model pose.time is Equal  "<< std::endl;
			return false;
		}
	double step = (sample.time - lastModelSample.time).toSeconds();

	lastModelSample = sample;
	std::cout << "step model "<< step << std::endl;

	return true;

}

bool Evaluation::handleMeasurement(const base::samples::RigidBodyState &sample)
{
	for (int j=0; j<3; j++)
	{
		if( isnan((double)sample.velocity[j]) || isnan((double)sample.angular_velocity[j]) )
			{
				std::cout << "measured VEl is nan "<< std::endl;
				return false;
			}
	}

	if(lastMeasuredSample.time > sample.time)
		{
			//std::cout << "measured pose.time is Bigger  "<< std::endl;
			return false;
		}
	if(lastMeasuredSample.time == sample.time)
		{
			//std::cout << "measured pose.time is Equal  "<< std::endl;
			return false;
		}

	double step = (sample.time - lastMeasuredSample.time).toSeconds();

	lastMeasuredSample = sample;
	std::cout << "step measured "<< step << std::endl;

	return true;

}


bool Evaluation::matchPose(const base::samples::RigidBodyState &model, base::samples::RigidBodyState &measured)
{
	base::samples::RigidBodyState front_pose;
	base::samples::RigidBodyState behind_pose;
	behind_pose.time = base::Time::fromSeconds(1);
	bool stop = false;
	std::queue<base::samples::RigidBodyState> queueofMeasured = queueMeasured;
	//match force with pose
	while(!queueofMeasured.empty() && !stop)
	{
		front_pose = queueofMeasured.front();
		queueofMeasured.pop();

		// Pose sample before queue
		if( (model.time-front_pose.time).toSeconds() < 0 && behind_pose.time == base::Time::fromSeconds(1) )
		{
			behind_pose = front_pose;
			measured = front_pose;
			//std::cout << "sample BEFORE "<< std::endl;
			stop = true;
		}

		// Pose sample after queue
		else if ((model.time-front_pose.time).toSeconds() > 0 && queueofMeasured.empty())
			{
				measured = front_pose;
				std::cout << "sample AFTER "<< std::endl;
				stop = true;
			}

		// Pose in the middle of queue
		if( (model.time-front_pose.time).toSeconds() > 0)
			behind_pose = front_pose;
		else
			{
				if((model.time-front_pose.time).toSeconds() > (behind_pose.time - model.time).toSeconds() )
					measured = front_pose;
				else
					measured = behind_pose;
				std::cout << "sample MIDDLE "<< std::endl;
				stop = true;
			}
	}
	return stop;
}


void Evaluation::checkSizeQueue(std::queue<base::samples::RigidBodyState> queue)
{
	while(queue.size() > max_queue_size)
		{queue.pop();}
}
