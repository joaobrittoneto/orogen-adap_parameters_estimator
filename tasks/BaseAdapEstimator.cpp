/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BaseAdapEstimator.hpp"

using namespace adap_parameters_estimator;

BaseAdapEstimator::BaseAdapEstimator(std::string const& name)
    : BaseAdapEstimatorBase(name)
{
    _last_received_pose = base::Time::fromSeconds(0);
    _last_processed_pose = base::Time::fromSeconds(0);
}

BaseAdapEstimator::BaseAdapEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : BaseAdapEstimatorBase(name, engine)
{
    _last_received_pose = base::Time::fromSeconds(0);
    _last_processed_pose = base::Time::fromSeconds(0);
}

BaseAdapEstimator::~BaseAdapEstimator()
{
}

void BaseAdapEstimator::effort_samplesCallback(const base::Time &ts, const ::base::commands::LinearAngular6DCommand &effort_samples_sample)
{
    if(checkSample(effort_samples_sample))
        queueEffort.push_back(effort_samples_sample);
    if(queueEffort.size() > _max_buffer_size.get())
        exception(BIG_QUEUE);
}

void BaseAdapEstimator::pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
    base::samples::RigidBodyState transformed_rbs = pose_samples_sample;
	transformed_rbs.time = ts;

	if(checkSample(transformed_rbs))
	{
        // Convert velocities to body-frame
        if(transformed_rbs.sourceFrame == "body" && transformed_rbs.targetFrame != "body")
        {
            if (!transformed_rbs.isValidValue(transformed_rbs.orientation))
            {
                exception(INVALID_ORIENTATION);
                return;
            }
            transformed_rbs.velocity = transformed_rbs.orientation.inverse() * transformed_rbs.velocity;
        }
		queuePose.push_back(transformed_rbs);
        if(queuePose.size() > _max_buffer_size.get())
            exception(BIG_QUEUE);
	}
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BaseAdapEstimator.hpp for more detailed
// documentation about them.

bool BaseAdapEstimator::configureHook()
{
    if (! BaseAdapEstimatorBase::configureHook())
        return false;
    queuePose.clear();
    queueEffort.clear();
    return true;
}
bool BaseAdapEstimator::startHook()
{
    if (! BaseAdapEstimatorBase::startHook())
        return false;
    return true;
}
void BaseAdapEstimator::updateHook()
{
    BaseAdapEstimatorBase::updateHook();
}
void BaseAdapEstimator::errorHook()
{
    BaseAdapEstimatorBase::errorHook();
}
void BaseAdapEstimator::stopHook()
{
    BaseAdapEstimatorBase::stopHook();
}
void BaseAdapEstimator::cleanupHook()
{
    BaseAdapEstimatorBase::cleanupHook();
    queuePose.clear();
    queueEffort.clear();
}

bool BaseAdapEstimator::checkSample(const base::samples::RigidBodyState &pose_sample)
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

bool BaseAdapEstimator::checkSample(const base::LinearAngular6DCommand &effort_sample)
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
