#ifndef adap_parameters_estimator_TYPES_HPP
#define adap_parameters_estimator_TYPES_HPP

#include <base/Time.hpp>
#include <adap_parameters_estimator/DataTypes.hpp>
/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace adap_parameters_estimator {

    struct ErrorVelocity
    {
        DOF dof;
        base::Time time;
        double error_velocity;
    };

    struct ErrorVelocity6D
    {
        base::Time time;
        base::Vector3d linear;
        base::Vector3d angular;

        ErrorVelocity6D(){
            linear = base::Vector3d::Zero();
            angular = base::Vector3d::Zero();
        }
    };

    struct MachtSampleStatus
    {
        bool pop_reference;
        bool delay_sample;
        uint pop_queue;

        MachtSampleStatus(){
            pop_reference = true;
            delay_sample = false;
            pop_queue = 0;
        }
    };
}

#endif
