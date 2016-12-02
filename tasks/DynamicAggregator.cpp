/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DynamicAggregator.hpp"

using namespace adap_parameters_estimator;

DynamicAggregator::DynamicAggregator(std::string const& name)
    : DynamicAggregatorBase(name)
{
    _last_received_pose = base::Time::fromSeconds(0);
}

DynamicAggregator::DynamicAggregator(std::string const& name, RTT::ExecutionEngine* engine)
    : DynamicAggregatorBase(name, engine)
{
    _last_received_pose = base::Time::fromSeconds(0);
}

DynamicAggregator::~DynamicAggregator()
{
}

void DynamicAggregator::acceleration_samplesCallback(const base::Time &ts, const ::uwv_dynamic_model::SecondaryStates &acceleration_samples_sample)
{
    if(checkSample(acceleration_samples_sample))
        queueAcceleration.push_back(acceleration_samples_sample);
    if(queueAcceleration.size() > _max_buffer_size.get())
        exception(BIG_QUEUE);
}

void DynamicAggregator::effort_samplesCallback(const base::Time &ts, const ::base::commands::LinearAngular6DCommand &effort_samples_sample)
{
    if(checkSample(effort_samples_sample))
        queueEffort.push_back(effort_samples_sample);
    if(queueEffort.size() > _max_buffer_size.get())
        exception(BIG_QUEUE);
}

void DynamicAggregator::pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
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
// hooks defined by Orocos::RTT. See DynamicAggregator.hpp for more detailed
// documentation about them.

bool DynamicAggregator::configureHook()
{
    if (! DynamicAggregatorBase::configureHook())
        return false;
    queuePose.clear();
    queueAcceleration.clear();
    queueEffort.clear();
    return true;
}
bool DynamicAggregator::startHook()
{
    if (! DynamicAggregatorBase::startHook())
        return false;

    return true;
}
void DynamicAggregator::updateHook()
{
    DynamicAggregatorBase::updateHook();

    if(!queuePose.empty() && !queueAcceleration.empty() && !queueEffort.empty())
    {
        uwv_dynamic_model::SecondaryStates accel = queueAcceleration.front();

        std::pair<MachtSampleStatus, base::LinearAngular6DCommand> effort = matchSample(accel.time, queueEffort);
        std::pair<MachtSampleStatus, base::samples::RigidBodyState> pose = matchSample(accel.time, queuePose);

        if(effort.first.pop_queue)
            queueEffort.erase(queueEffort.begin(), queueEffort.begin()+effort.first.pop_queue);
        if(pose.first.pop_queue)
            queuePose.erase(queuePose.begin(), queuePose.begin()+pose.first.pop_queue);
        if(effort.first.pop_reference && pose.first.pop_reference)
            queueAcceleration.pop_front();
        if(effort.first.delay_sample || pose.first.delay_sample)
            return;

        uwv_dynamic_model::DynamicStates dynamic;
        dynamic.pose = pose.second;
        accel.efforts = effort.second;
        dynamic.secondary_states = accel;
        dynamic.time =  pose.second.time;

        _dynamic_sample.write(dynamic);
    }
}
void DynamicAggregator::errorHook()
{
    DynamicAggregatorBase::errorHook();
}
void DynamicAggregator::stopHook()
{
    DynamicAggregatorBase::stopHook();
}
void DynamicAggregator::cleanupHook()
{
    DynamicAggregatorBase::cleanupHook();
    queuePose.clear();
    queueAcceleration.clear();
    queueEffort.clear();
}

bool DynamicAggregator::checkSample(const base::samples::RigidBodyState &pose_sample)
{
    for (int j=0; j<3; j++)
        if( base::isNaN(pose_sample.velocity[j]) || base::isNaN(pose_sample.angular_velocity[j]) )
            return false;

    if(_last_received_pose >= pose_sample.time)
        return false;

    _last_received_pose = pose_sample.time;
    return true;
}

bool DynamicAggregator::checkSample(const base::LinearAngular6DCommand &effort_sample)
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

bool DynamicAggregator::checkSample(const uwv_dynamic_model::SecondaryStates &accel_sample)
{
    for (int j=0; j < 3; j++)
    {
        if( base::isNaN(accel_sample.linear_acceleration.acceleration[j]))
            return false;
        if( base::isNaN(accel_sample.angular_acceleration.acceleration[j]))
            return false;
    }
    return true;
}
