/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OneDOFAdapEstimator.hpp"

using namespace adap_parameters_estimator;

OneDOFAdapEstimator::OneDOFAdapEstimator(std::string const& name)
    : OneDOFAdapEstimatorBase(name)
{
}

OneDOFAdapEstimator::OneDOFAdapEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : OneDOFAdapEstimatorBase(name, engine)
{
}

OneDOFAdapEstimator::~OneDOFAdapEstimator()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OneDOFAdapEstimator.hpp for more detailed
// documentation about them.

bool OneDOFAdapEstimator::configureHook()
{
    if (! OneDOFAdapEstimatorBase::configureHook())
        return false;
    adap_parameters_estimator = new AdapParameters(_gain_lambda.get(), _gain_a.get());
    return true;
}
bool OneDOFAdapEstimator::startHook()
{
    if (! OneDOFAdapEstimatorBase::startHook())
        return false;
    return true;
}
void OneDOFAdapEstimator::updateHook()
{
    OneDOFAdapEstimatorBase::updateHook();

    if(!queuePose.empty() && !queueEffort.empty())
    {
        base::samples::RigidBodyState pose;
        pose = queuePose.front();
        std::pair<MachtSampleStatus, base::LinearAngular6DCommand> effort = matchSample(pose.time, queueEffort);

        if(effort.first.pop_queue)
            queueEffort.erase(queueEffort.begin(), queueEffort.begin()+effort.first.pop_queue);
        if(effort.first.pop_reference)
            queuePose.pop_front();
        if(effort.first.delay_sample)
            return;

        // Update step. In case the pose's periodicity is not constant (eg: using DVL sample)
        if(_last_processed_pose.toSeconds())
        {
            double step = (pose.time - _last_processed_pose).toSeconds();
            adap_parameters_estimator->setStep(step);
        }
        _last_processed_pose = pose.time;

        base::Vector4d estimated_parameters = adap_parameters_estimator->estimateParameters( getVector(effort.second)[_dof.get()] , getVector(pose)[_dof.get()]);
        OneDOFParameters parameters = convertParameters(estimated_parameters);
        parameters.time = pose.time;
        parameters.dof = _dof.get();

        _parameters.write(parameters);

        ErrorVelocity error;
        error.error_velocity = adap_parameters_estimator->getDeltaVelocity();
        error.time = parameters.time;
        error.dof = parameters.dof;
        _error.write(error);
    }
}
void OneDOFAdapEstimator::errorHook()
{
    OneDOFAdapEstimatorBase::errorHook();
}
void OneDOFAdapEstimator::stopHook()
{
    OneDOFAdapEstimatorBase::stopHook();
}
void OneDOFAdapEstimator::cleanupHook()
{
    OneDOFAdapEstimatorBase::cleanupHook();
}

base::Vector6d OneDOFAdapEstimator::getVector(const base::LinearAngular6DCommand &effort)
{
    base::Vector6d output;
    for (uint i = 0; i < 3; i++)
    {
        output[i] = effort.linear[i];
        output[i+3] = effort.angular[i];
    }
    return output;
}

base::Vector6d OneDOFAdapEstimator::getVector(const base::samples::RigidBodyState &pose)
{
    base::Vector6d output;
    for (uint i = 0; i < 3; i++)
    {
        output[i] = pose.velocity[i];
        output[i+3] = pose.angular_velocity[i];
    }
    return output;
}

OneDOFParameters OneDOFAdapEstimator::convertParameters(const base::Vector4d &parameters)
{
    OneDOFParameters output;
    output.inertia = parameters[0];
    output.quadratic_damping = parameters[1];
    output.linear_damping = parameters[2];
    output.buoyancy = parameters[3];
    return output;
}

void OneDOFAdapEstimator::setParameters(::adap_parameters_estimator::OneDOFParameters const & initial_parameters)
{
    base::Vector4d init_parameters(initial_parameters.inertia,
        initial_parameters.quadratic_damping, initial_parameters.linear_damping,
        initial_parameters.buoyancy);
    adap_parameters_estimator->setParameters(init_parameters);
}
