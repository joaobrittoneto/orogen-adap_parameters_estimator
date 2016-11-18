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
}

#endif
