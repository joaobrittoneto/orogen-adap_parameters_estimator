/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace adap_parameters_estimator;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    _gain_lambda.set(base::Vector4d(1,1,1,1));
    _last_received_pose = base::Time::fromSeconds(0);
    _last_processed_pose = base::Time::fromSeconds(0);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    _gain_lambda.set(base::Vector4d(1,1,1,1));
    _last_received_pose = base::Time::fromSeconds(0);
    _last_processed_pose = base::Time::fromSeconds(0);
}

Task::~Task()
{
}

void Task::effort_samplesCallback(const base::Time &ts, const ::base::LinearAngular6DCommand &effort_samples)
{
    base::LinearAngular6DCommand effort = effort_samples;

	if(checkSample(effort))
        queueEffort.push(effort);
    if(queueEffort.size() > size_queue)
        exception(BIG_QUEUE);
}

void Task::pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples)
{

	base::samples::RigidBodyState transformed_rbs = pose_samples;
	transformed_rbs.time = ts;

	if(checkSample(transformed_rbs))
	{
        // Convert velocities to body-frame
        if(transformed_rbs.sourceFrame == "body" && transformed_rbs.targetFrame != "body")
                transformed_rbs.velocity = transformed_rbs.orientation.inverse() * transformed_rbs.velocity;
		queuePose.push(transformed_rbs);
        if(queuePose.size() > size_queue)
            exception(BIG_QUEUE);
	}

}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    adapParam = new AdapParameters(_gain_lambda.get(), _gain_a.get(), TaskContext::getPeriod());
	return true;

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

    if(!queuePose.empty() && !queueEffort.empty())
    {
        base::samples::RigidBodyState pose;
     	pose = queuePose.front();
    	queuePose.pop();

        std::pair<uint, base::LinearAngular6DCommand> effort = matchEffort(pose);
        while(!effort.first)
        {
            queueEffort.pop();
            effort.first--;
        }

        // Update step. In case the pose's periodicity is not constant (eg: using DVL sample)
        if(!_last_processed_pose.toSeconds())
        {
            double step = (pose.time - _last_processed_pose).toSeconds();
            adapParam->setStep(step);
            _last_processed_pose = pose.time;
        }

		base::Vector4d estimated_parameters = adapParam->estimateParameters( getVector(effort.second)[_dof.get()] , getVector(pose)[_dof.get()]);
        OneDOFParameters parameters = convertParameters(estimated_parameters);
        parameters.time = pose.time;
        parameters.dof = _dof.get();

		_parameters.write(parameters);

        double deltaV =0;
        double normDeltaV =0;
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

bool Task::checkSample(const base::samples::RigidBodyState &pose_sample)
{
	for (int j=0; j<3; j++)
	{
		if( base::isNaN(pose_sample.velocity[j]) || base::isNaN(pose_sample.angular_velocity[j]) )
			return false;
	}
	if(_last_received_pose >= pose_sample.time)
		return false;
    _last_received_pose = pose_sample.time;
	return true;
}

bool Task::checkSample(const base::LinearAngular6DCommand &effort_sample)
{
	for (int j=0; j < 3; j++)
	{
		if( base::isNaN(effort_sample.linear[j]))
			return false;
        if( base::isNaN(effort_sample.angular[j]))
            return false;
	}
	return true;
}

std::pair<uint, base::LinearAngular6DCommand> Task::matchEffort(const base::samples::RigidBodyState &pose_sample)
{
	base::LinearAngular6DCommand front_effort;
	base::LinearAngular6DCommand behind_effort;
    base::LinearAngular6DCommand output_effort;
	behind_effort.time = base::Time::fromSeconds(1);
	bool stop = false;
    uint old_effort = 0;
	std::queue<base::LinearAngular6DCommand> copyQueueoEffort = queueEffort;
	//match force with pose
	while(!copyQueueoEffort.empty() && !stop)
	{
		front_effort = copyQueueoEffort.front();
		copyQueueoEffort.pop();

		// Pose sample before or after queue
		if( ((pose_sample.time < front_effort.time) && behind_effort.time == base::Time::fromSeconds(1)) ||
            ((pose_sample.time > front_effort.time) && copyQueueoEffort.empty()) )
		{   // Check for out of phase samples
            if(fabs((pose_sample.time-front_effort.time).toSeconds()) > 1)
                exception(INPUT_DELAY);
			output_effort = front_effort;
			stop = true;
		}
		// Pose in the middle of queue
		else if( (pose_sample.time-front_effort.time).toSeconds() <= 0)
        {
			if((pose_sample.time-front_effort.time).toSeconds() > (behind_effort.time - pose_sample.time).toSeconds() )
				output_effort = front_effort;
			else
				output_effort = behind_effort;
			stop = true;
		}
		else
        {
            behind_effort = front_effort;
            old_effort++;
        }
	}
    // Let one behind_effort in queue
    old_effort--;
	return std::make_pair(old_effort, output_effort);
}

base::Vector6d Task::getVector(const base::LinearAngular6DCommand &effort)
{
    base::Vector6d output;
    for (uint i = 0; i < 3; i++)
    {
        output[i] = effort.linear[i];
        output[i+3] = effort.angular[i];
    }
    return output;
}

base::Vector6d Task::getVector(const base::samples::RigidBodyState &pose)
{
    base::Vector6d output;
    for (uint i = 0; i < 3; i++)
    {
        output[i] = pose.velocity[i];
        output[i+3] = pose.angular_velocity[i];
    }
    return output;
}

OneDOFParameters Task::convertParameters(const base::Vector4d &parameters)
{
    OneDOFParameters output;
    output.inertia = parameters[0];
    output.quadratic_damping = parameters[1];
    output.linear_damping = parameters[2];
    output.buoyancy = parameters[3];
    return output;
}
