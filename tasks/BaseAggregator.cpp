/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BaseAggregator.hpp"

using namespace adap_parameters_estimator;

BaseAggregator::BaseAggregator(std::string const& name)
    : BaseAggregatorBase(name)
{
    _window_tolerance.set(base::Time::fromSeconds(0.5));
}

BaseAggregator::BaseAggregator(std::string const& name, RTT::ExecutionEngine* engine)
    : BaseAggregatorBase(name, engine)
{
    _window_tolerance.set(base::Time::fromSeconds(0.5));
}

BaseAggregator::~BaseAggregator()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BaseAggregator.hpp for more detailed
// documentation about them.

bool BaseAggregator::configureHook()
{
    if (! BaseAggregatorBase::configureHook())
        return false;
    return true;
}
bool BaseAggregator::startHook()
{
    if (! BaseAggregatorBase::startHook())
        return false;
    return true;
}
void BaseAggregator::updateHook()
{
    BaseAggregatorBase::updateHook();
}
void BaseAggregator::errorHook()
{
    BaseAggregatorBase::errorHook();
}
void BaseAggregator::stopHook()
{
    BaseAggregatorBase::stopHook();
}
void BaseAggregator::cleanupHook()
{
    BaseAggregatorBase::cleanupHook();
}
