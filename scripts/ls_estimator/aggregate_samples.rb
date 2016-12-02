require 'orocos'
require 'vizkit'
include Orocos

address = ARGV[0].to_s

@log_replay = Orocos::Log::Replay.open(address)


#######################################################################

Orocos.run 'adap_parameters_estimator::DynamicAggregator' => 'aggregator',
           'auv_numeric_filtering::VelocityDerivative'  => 'accel' do

    aggregator     = TaskContext.get 'aggregator'
    accel          = TaskContext.get 'accel'

    Orocos.log_all

    #########################################################
    # velocity_provider in body-frame
    velocity      = @log_replay.velocity_provider_filter
    efforts       = @log_replay.thruster_force_2_body_effort
    #########################################################
    accel.apply_conf_file('auv_numeric_filtering::VelocityDerivative.yml')
    ##########################################################################

    velocity.velocity_samples.connect_to    accel.pose_in,                      :type => :buffer, :size => 100
    velocity.velocity_samples.connect_to    aggregator.pose_samples,            :type => :buffer, :size => 100
    accel.accel_out.connect_to              aggregator.acceleration_samples,    :type => :buffer, :size => 100
    efforts.body_efforts.connect_to         aggregator.effort_samples,          :type => :buffer, :size => 100

    aggregator.configure
    accel.configure

    aggregator.start
    accel.start


    Vizkit.control @log_replay
    Vizkit.exec
end
