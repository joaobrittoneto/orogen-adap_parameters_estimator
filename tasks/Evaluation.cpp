/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Evaluation.hpp"

using namespace adap_parameters_estimator;

Evaluation::Evaluation(std::string const& name)
    : EvaluationBase(name)
{
    _last_received_model = base::Time::fromSeconds(0);
    _last_received_measured = base::Time::fromSeconds(0);
}

Evaluation::Evaluation(std::string const& name, RTT::ExecutionEngine* engine)
    : EvaluationBase(name, engine)
{
    _last_received_model = base::Time::fromSeconds(0);
    _last_received_measured = base::Time::fromSeconds(0);
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

    if(checkSample(model, _last_received_model))
	{
        _last_received_model = model.time;
        queueModel.push_back(model);
        if(queueModel.size() > _max_buffer_size.get())
            exception(BIG_QUEUE);
	}
}

void Evaluation::measured_velocityCallback(const base::Time &ts, const ::base::samples::RigidBodyState &measured_velocity_sample)
{
	base::samples::RigidBodyState measured = measured_velocity_sample;

	// Convert velocities to body-frame
    if(measured.sourceFrame == "body" && measured.targetFrame != "body")
	{
    	measured.velocity = measured.orientation.inverse() * measured.velocity;
	}

    if(checkSample(measured, _last_received_measured))
	{
        _last_received_measured = measured.time;
        queueMeasured.push_back(measured);
        if(queueMeasured.size() > _max_buffer_size.get())
            exception(BIG_QUEUE);
	}
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Evaluation.hpp for more detailed
// documentation about them.

bool Evaluation::configureHook()
{
    if (! EvaluationBase::configureHook())
        return false;

    _mae_vel = ErrorVelocity6D();
    _number_of_samples = 0;
	queueModel.clear();
	queueMeasured.clear();
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

    if(!queueMeasured.empty() && !queueModel.empty())
    {
        base::samples::RigidBodyState model = queueModel.front();

        std::pair<MachtSampleStatus, base::samples::RigidBodyState> measurement = matchSample(model.time, queueMeasured);

        if(measurement.first.pop_queue)
            queueMeasured.erase(queueMeasured.begin(), queueMeasured.begin()+measurement.first.pop_queue);
        if(measurement.first.pop_reference)
            queueModel.pop_front();
        if(measurement.first.delay_sample)
            return;

        ErrorVelocity6D error_velocity;

        error_velocity.time = model.time;
        error_velocity.linear = measurement.second.velocity - model.velocity;
        error_velocity.linear = measurement.second.angular_velocity - model.angular_velocity;

        _mae_vel.linear = _mae_vel.linear*_number_of_samples + error_velocity.linear.cwiseAbs();
        _mae_vel.angular = _mae_vel.angular*_number_of_samples + error_velocity.linear.cwiseAbs();
        _number_of_samples++;
        _mae_vel.linear /= _number_of_samples;
        _mae_vel.angular /= _number_of_samples;
        _mae_vel.time = model.time;

        _error_velocity.write(error_velocity);
        _mae_velocity.write(_mae_vel);
    }
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

bool Evaluation::checkSample(const base::samples::RigidBodyState &pose_sample, base::Time last_time)
{
    for (int j=0; j<3; j++)
        if( base::isNaN(pose_sample.velocity[j]) || base::isNaN(pose_sample.angular_velocity[j]) )
            return false;

    if(last_time >= pose_sample.time)
        return false;

    return true;
}
