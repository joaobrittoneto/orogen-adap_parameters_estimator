/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ADAP_PARAMETERS_ESTIMATOR_BASEAGGREGATOR_TASK_HPP
#define ADAP_PARAMETERS_ESTIMATOR_BASEAGGREGATOR_TASK_HPP

#include "adap_parameters_estimator/BaseAggregatorBase.hpp"
#include "adap_parameters_estimator/adap_parameters_estimatorTypes.hpp"
#include <deque>

namespace adap_parameters_estimator{

    /*! \class BaseAggregator
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','adap_parameters_estimator::BaseAggregator')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class BaseAggregator : public BaseAggregatorBase
    {
	friend class BaseAggregatorBase;
    protected:



    public:
        /** TaskContext constructor for BaseAggregator
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        BaseAggregator(std::string const& name = "adap_parameters_estimator::BaseAggregator");

        /** TaskContext constructor for BaseAggregator
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        BaseAggregator(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of BaseAggregator
         */
	~BaseAggregator();

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

        // Provide the nearest sample in queue to the reference_time, also its position in queue
        template<typename Type>
        std::pair<MachtSampleStatus, Type> matchSample (base::Time reference_time, const std::deque<Type> &queue)
        {
            MachtSampleStatus status;
            Type front_sample;
            Type behind_sample;
            Type output_sample;
            behind_sample.time = base::Time::fromSeconds(1);
            bool stop = false;
            int old_sample = 0;

            //match queue sample with reference_time
            while(old_sample < queue.size() && !stop)
            {
                front_sample = queue.at(old_sample);
                // copyQueue.pop_front();

                // Reference before or after queue
                if( ((reference_time < front_sample.time) && behind_sample.time == base::Time::fromSeconds(1)) ||
                    ((reference_time > front_sample.time) && old_sample == queue.size()-1) )
                {
                    output_sample = front_sample;
                    stop = true;
                }
                // Reference in the middle of queue
                else if( reference_time <= front_sample.time)
                {
                    if( (front_sample.time - reference_time) < (reference_time - behind_sample.time) )
                        output_sample = front_sample;
                    else
                        output_sample = behind_sample;
                    stop = true;
                }
                else
                {
                    behind_sample = front_sample;
                    old_sample++;
                }
            }
            // Check for out of phase samples
            if(fabs((reference_time-output_sample.time).toSeconds()) > _window_tolerance.get().toSeconds())
            {
                status.delay_sample = true;
                // In the end of queue
                if (old_sample == queue.size()-1)
                    status.pop_reference = false;
            }
            status.pop_queue = old_sample;
            return std::make_pair(status, output_sample);
        }
    };
}

#endif
