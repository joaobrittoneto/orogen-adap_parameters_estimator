require 'orocos'
require 'vizkit'
include Orocos

address = ARGV[0].to_s

@log_replay = Orocos::Log::Replay.open(address)


#######################################################################

Orocos.run 'adap_parameters_estimator::OneDOFAdapEstimator' => 'adap_estimator' do

    adap_estimator  = TaskContext.get 'adap_estimator'

    # Orocos.log_all

    #########################################################
    # velocity_provider in body-frame
    velocity      = @log_replay.velocity_provider_filter
    efforts       = @log_replay.thruster_force_2_body_effort
    #########################################################
    adap_estimator.apply_conf_file('OneDOFAdapEstimator.yml', ['sway'])
    ##########################################################################

    velocity.velocity_samples.connect_to    adap_estimator.pose_samples,    :type => :buffer, :size => 100
    efforts.body_efforts.connect_to         adap_estimator.effort_samples,  :type => :buffer, :size => 100
    # adap_estimator.window_tolerance = Time.at(2)

    adap_estimator.configure
    adap_estimator.start

    Vizkit.control @log_replay
    Vizkit.exec
end
