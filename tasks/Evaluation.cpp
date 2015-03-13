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

    base::samples::RigidBodyState	Model;
	base::samples::RigidBodyState	Measurements;
	static std::queue<base::samples::RigidBodyState>	queueOfModel;
	static std::queue<base::samples::RigidBodyState>	queueOfMeasurements;

	//static int size_model = 0;
	//static int size_measurements = 0;
	static int size = 0;
	static double mae_velocity = 0;
	static double mean_velocity = 0;

	static double error_velocity = 0;
	static double norm_mae_velocity = 0;
	double model = 0;
	double measurement = 0;

	//static bool mo = false;
	//static bool me = false;



	if (_model_velocity.read(Model) == RTT::NewData /*&& !mo*/ )
	{
		queueOfModel.push(Model);
		//size_model ++;
		//mo = true;
	}

	if (_measured_velocity.read(Measurements) == RTT::NewData /*&& !me*/  )
	{
		queueOfMeasurements.push(Measurements);
		//size_measurements++;
		//me = true;
	}

	if(!queueOfMeasurements.empty() && !queueOfModel.empty())
	{

		if (queueOfMeasurements.front().time == queueOfModel.front().time /*size_measurements==size_model && Model.time==Measurements.time && me && mo*/)
		{
			if (dof < 3)
				{model= queueOfModel.front().velocity[dof]; measurement = queueOfMeasurements.front().velocity[dof];}
			else
				{model= queueOfModel.front().angular_velocity[dof]; measurement = queueOfMeasurements.front().angular_velocity[dof];}

			size++;
			queueOfMeasurements.pop();
			queueOfModel.pop();


			error_velocity = measurement - model;
			mean_velocity = (mean_velocity*(size-1) + fabs(measurement))/size;
			mae_velocity = (mae_velocity*(size-1) + fabs(error_velocity))/size;
			norm_mae_velocity = mae_velocity / mean_velocity;

			//mo = false;
			//me = false;

		}

		else if(queueOfMeasurements.front().time > queueOfModel.front().time)
		{
			std::cout << std::endl << "lost of model sample "<< queueOfModel.front().time << std::endl;
			queueOfModel.pop();
		}
		else if(queueOfMeasurements.front().time < queueOfModel.front().time)
		{
			std::cout << std::endl << "lost of measurement sample "<< queueOfMeasurements.front().time << std::endl;
			queueOfMeasurements.pop();
		}

	}
	//else if(queueOfMeasurements.empty() || queueOfModel.empty())
	//	std::cout << std::endl << "empty queue "<< std::endl;

	//std::cout << std::endl << "MAE_Vel: "<< mae_velocity << std::endl;
	//std::cout << std::endl << "Norm_MAE_Vel: "<< norm_mae_velocity << std::endl;
	_error_velocity.write(error_velocity);
	_mae_velocity.write(mae_velocity);
	_norm_mae_velocity.write(norm_mae_velocity);

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
