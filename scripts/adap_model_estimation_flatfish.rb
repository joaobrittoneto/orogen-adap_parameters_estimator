#library for displaying data
require 'orocos'
require 'vizkit'
require './gui/gui_parameters.rb'

include Orocos

Orocos::CORBA.name_service = "192.168.128.50"  # Flatfish


Orocos.run 'adap_parameters_estimator::ForceApplier'        => 'forces&torques',
           'adap_parameters_estimator::AdapModelEstimation' => 'adap_model' do


    forces_torques      = TaskContext.get 'forces&torques'
    adap_model          = TaskContext.get 'adap_model'
    
    #########################################################
    pose_estimator     = TaskContext.get 'pose_estimator'
#    pose_estimator     = TaskContext.get 'pose_estimator_dead_reckoning'
#    thurster            = TaskContext.get 'thrusters'
    thurster            = TaskContext.get 'acceleration_controller'    

    #########################################################
    

    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplier.yml',['flatfish']) 
    adap_model.apply_conf_file('config/adap_parameters_estimator::AdapModelEstimation.yml',['dagon', 'dagon_surge'])
    
 
#    thurster.state_out.connect_to       forces_torques.thruster_samples  
    thurster.cmd_out.connect_to       forces_torques.thruster_samples  
          
    pose_estimator.pose_samples.connect_to      adap_model.pose_samples,        :type => :buffer, :size => 100 
    forces_torques.forces.connect_to            adap_model.forces_samples,      :type => :buffer, :size => 100      
    
    adap_model.aggregator_max_latency = 0.5
    adap_model.pose_samples_period = 0.0001
    adap_model.forces_samples_period = 0.0001
    adap_model.sTime = 2.0
    
    forces_torques.configure      
    adap_model.configure
    forces_torques.start
    adap_model.start
    
    
   

 
   ## Defining the proxy for each task 
   parametersproxy      = Orocos::Async.proxy("adap_model")
   inputforcesproxy     = Orocos::Async.proxy("forces&torques")
   inputvelocitiesproxy = Orocos::Async.proxy("pose_estimator")        
   
	
   ## Defining the port variables using the proxys
   deltaVport       = parametersproxy.port("deltaV")
   nomrDeltaVport   = parametersproxy.port("normDeltaV")
   parametersport   = parametersproxy.port("parameters")      
   velocityport     = inputvelocitiesproxy.port("pose_samples")
   forcesport       = inputforcesproxy.port("forces")
        
    #open control widget and start replay
    supervisory = Supervisory.new(adap_model)
    supervisory.evolution(deltaVport, nomrDeltaVport, velocityport, forcesport, parametersport)
    
        
    supervisory.show
    
    #Vizkit.control @log_replay
    Vizkit.exec
end
