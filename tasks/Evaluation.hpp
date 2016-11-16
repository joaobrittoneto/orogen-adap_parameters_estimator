/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ADAP_PARAMETERS_ESTIMATOR_EVALUATION_TASK_HPP
#define ADAP_PARAMETERS_ESTIMATOR_EVALUATION_TASK_HPP

#include "adap_parameters_estimator/EvaluationBase.hpp"
#include "adap_parameters_estimator/AdapParameters.hpp"

namespace adap_parameters_estimator {

    /*! \class Evaluation
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','adap_parameters_estimator::Evaluation')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Evaluation : public EvaluationBase
    {
	friend class EvaluationBase;
    protected:

		adap_parameters_estimator::DOF dof;

		// Queue of model and measured samples
		std::queue<base::samples::RigidBodyState> queueModel;
		std::queue<base::samples::RigidBodyState> queueMeasured;
		int max_queue_size	= 100;

		// Aux variables
		base::samples::RigidBodyState	lastModelSample;
		base::samples::RigidBodyState	lastMeasuredSample;

        virtual void model_velocityCallback(const base::Time &ts, const ::base::samples::RigidBodyState &model_velocity_sample);

        virtual void measured_velocityCallback(const base::Time &ts, const ::base::samples::RigidBodyState &measured_velocity_sample);

        bool handleModel(const base::samples::RigidBodyState &sample);
        bool handleMeasurement(const base::samples::RigidBodyState &sample);

        bool matchPose(base::samples::RigidBodyState &input, std::queue<base::samples::RigidBodyState> &queue, base::samples::RigidBodyState &output, int &back_queue);
        void checkSizeQueue(std::queue<base::samples::RigidBodyState> queue);

    public:
        /** TaskContext constructor for Evaluation
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Evaluation(std::string const& name = "adap_parameters_estimator::Evaluation");

        /** TaskContext constructor for Evaluation
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        Evaluation(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Evaluation
         */
	~Evaluation();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif
