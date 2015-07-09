/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ForceApplierModel.hpp"

using namespace adap_parameters_estimator;

ForceApplierModel::ForceApplierModel(std::string const& name)
    : ForceApplierModelBase(name)
{
}

ForceApplierModel::ForceApplierModel(std::string const& name, RTT::ExecutionEngine* engine)
    : ForceApplierModelBase(name, engine)
{
}

ForceApplierModel::~ForceApplierModel()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ForceApplier.hpp for more detailed
// documentation about them.

bool ForceApplierModel::configureHook()
{
    if (! ForceApplierModelBase::configureHook())
        return false;

    numberOfThrusters	= _number_of_thruster.get();
    thrusterVoltage		= _thrusterVoltage.get();

    thrusterNames	= _names.get();
    if(thrusterNames.size() != numberOfThrusters && thrusterNames.size() != 0){
	   exception(WRONG_SIZE_OF_NAMES);
	   return false;
   }

    CoeffPos	= _thruster_coefficients_Pos.get() ;
	CoeffNeg	= _thruster_coefficients_Neg.get();
    if(	CoeffPos.size() != numberOfThrusters &&
    	CoeffNeg.size() != numberOfThrusters ){
		exception(WRONG_SIZE_OF_THRUSTER_COEFFICIENTS);
		return false;
	}

    controlModes	= _control_modes.get();
    if(controlModes.size() == 0){
		//resize the controlModes to number of Thrustes
		controlModes.resize(numberOfThrusters);
		//set undefined controlModes to RAW
		for(int i = 0; i < numberOfThrusters; i++){
			controlModes[i] = base::JointState::RAW;
		}
	} else if(controlModes.size() != numberOfThrusters){
		exception(WRONG_SIZE_OF_CONTROLMODES);
		return false;
	}

    TCM		= _TCM.get();
    if(	TCM.size() != numberOfThrusters*6){
		exception(WRONG_SIZE_OF_THRUSTER_MATRIX);
		return false;
	}

    offsetForces	=	_offset_forces.get();
    if(offsetForces.size() != 6 ){
		exception(WRONG_SIZE_OF_THRUSTER_MATRIX);
		return false;
	}

    // The three first elements are the linear forces and the last three are the torques applied in the auv in body-frame
    forcesOut.resize(6);
    std::vector<std::string> forces_names;
    forces_names.push_back("Surge");
    forces_names.push_back("Sway");
    forces_names.push_back("Heave");
    forces_names.push_back("Roll");
    forces_names.push_back("Pitch");
    forces_names.push_back("Yaw");
    forcesOut.names = forces_names;


    return true;
}
bool ForceApplierModel::startHook()
{
    if (! ForceApplierModelBase::startHook())
        return false;
    return true;
}
void ForceApplierModel::updateHook()
{
    ForceApplierModelBase::updateHook();

    base::VectorXd			forces;

    if (_thruster_samples.read(thrusterForces) == RTT::NewData)
    {
    	if(checkControlInput(thrusterForces))
		{
    		calcThrustersForces(thrusterForces, forces);
    		calcOutput(forcesOut, forces);

    		forcesOut.time = thrusterForces.time;

    		_forces.write(forcesOut);
    		_thruster_forces.write(thrusterForces);
		}
    }

}
void ForceApplierModel::errorHook()
{
    ForceApplierModelBase::errorHook();
}
void ForceApplierModel::stopHook()
{
    ForceApplierModelBase::stopHook();
}
void ForceApplierModel::cleanupHook()
{
    ForceApplierModelBase::cleanupHook();
}


void ForceApplierModel::calcThrustersForces(base::samples::Joints &forcesThruster, base::VectorXd &forces)
{
	forces.resize(numberOfThrusters);
	for (int i=0; i<numberOfThrusters; i++)
	{
		double input;

		input = forcesThruster.elements[i].getField(controlModes[i]);

		if(controlModes[i] == base::JointState::RAW)
			input *= thrusterVoltage;

		double factor = 0.01;
		if( input >= -factor && input <= factor && controlModes[i] != base::JointState::EFFORT)
			forcesThruster.elements[i].effort = 0.0;

		else if(controlModes[i] != base::JointState::EFFORT)
		{
			if(input > 0)
				forcesThruster.elements[i].effort = CoeffPos[i]*input*fabs(input);
			else
				forcesThruster.elements[i].effort = CoeffNeg[i]*input*fabs(input);
		}
		// The vector of forces for each thruster
		forces[i] = forcesThruster.elements[i].effort;
	}
}


void ForceApplierModel::calcOutput(base::samples::Joints &out_cmd, base::VectorXd &forces)
{
	base::Vector6d outCmd = TCM * forces;
	for(int i; i<6; i++)
	{
		out_cmd.elements[i].effort = outCmd[i] + offsetForces[i];
	}
}

// Check thrusters input
bool ForceApplierModel::checkControlInput(base::samples::Joints &forcesThruster)
{
	std::string textElement;
	bool checkError	= false;
	bool checkMode	= false;

	if(forcesThruster.elements.size() != numberOfThrusters)
	{
		RTT::log(RTT::Error) << "The input has not "<< numberOfThrusters <<
				" thruster as predicted "<< RTT::endlog();
		exception(UNEXPECTED_THRUSTER_INPUT);
		return false;
	}

	for (int i=0; i<numberOfThrusters; i++)
	{	// Verify if the input mode is a valid input or a nan
		 switch(controlModes[i])
		{
		 	 case base::JointState::EFFORT:
				if (!forcesThruster.elements[i].hasEffort())
				{
					textElement = "effort";
					checkError = true;
				}
				break;
			 case base::JointState::SPEED:
				if (!forcesThruster.elements[i].hasSpeed())
				{
					textElement = "speed";
					checkError = true;
				}
				break;
			 case base::JointState::RAW:
				if (!forcesThruster.elements[i].hasRaw())
				{
					textElement = "speed";
					checkError = true;
				}
				break;
			 case base::JointState::POSITION:
				if (!forcesThruster.elements[i].hasPosition())
				{
					textElement = "position";
					checkMode = true;
				}
				break;
			 case base::JointState::ACCELERATION:
				if (!forcesThruster.elements[i].hasAcceleration())
				{
					textElement = "acceleration";
					checkMode = true;
				}
				break;
		}

		// In case of any input value is NAN or not the write control mode
		if(checkError || checkMode)
		{
			// Define how to call the problematic thruster
			std::string textThruster;
			if(thrusterNames.size() == numberOfThrusters)
				textThruster = thrusterNames[i];
			else
			{
				std::stringstream number;
				number << i;
				textThruster = number.str();
			}
			// In case of any input value is NAN
			if(checkError)
				RTT::log(RTT::Error) << "The field "<< textElement << " of thruster "<<
									textThruster << " was not set." << RTT::endlog();
			// In case the control mode is unfamiliar
			if(checkMode)
				RTT::log(RTT::Error) << "The field "<< textElement << " of thruster "<<
					textThruster << " is not a usual control input." << RTT::endlog();
			exception(UNSET_THRUSTER_INPUT);
			return false;
		}
	}
	// In case of all inputs have valid values
	return true;
}
