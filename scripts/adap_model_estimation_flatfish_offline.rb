#library for displaying data
require 'orocos'
require 'vizkit'
require './gui/gui_parameters.rb'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.50"  # Flatfish

#load log file 
#######################################################################

#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linX.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linY.log")
@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linZ.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/angZ.log")

#######################################################################

Orocos.run 'adap_parameters_estimator::ForceApplier'        => 'forces&torques',
           'adap_parameters_estimator::AdapModelEstimation' => 'adap_model',
           'filter_apriltag_detector::AngVel'               => 'ang_vel' do


    forces_torques      = TaskContext.get 'forces&torques'
    adap_model          = TaskContext.get 'adap_model'
    ang_vel             = TaskContext.get 'ang_vel'
    
   # Orocos.log_all
    
    #########################################################
    
    pose_estimator     = @log_replay.pose_estimator
    ori_estimator        = @log_replay.orientation_estimator
#    pose_estimation     = @log_replay.pose_estimator_dead_reckoning    
#    thurster            = @log_replay.thrusters  
    thurster            = @log_replay.acceleration_controller     
    #########################################################
    
    dof = 'heave'

    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplier.yml',['flatfish']) 
    adap_model.apply_conf_file('config/adap_parameters_estimator::AdapModelEstimation.yml',['flatfish', dof])
    
 
#    thurster.state_out.connect_to       forces_torques.thruster_samples  
    thurster.cmd_out.connect_to       forces_torques.thruster_samples  

    if dof == 'yaw'
        ori_estimator.orientation_samples_out.connect_to      ang_vel.orientation_sample,     :type => :buffer, :size => 100
        pose_estimator.pose_samples.connect_to                ang_vel.pose_sample,            :type => :buffer, :size => 100  
        ang_vel.output.connect_to                             adap_model.pose_samples,        :type => :buffer, :size => 100 
    else             
        pose_estimator.pose_samples.connect_to      adap_model.pose_samples,        :type => :buffer, :size => 100 
    end
    
    forces_torques.forces.connect_to            adap_model.forces_samples,      :type => :buffer, :size => 100      
    
    #adap_model.aggregator_max_latency = 2.0
    #adap_model.pose_samples_period = 0.0001
    #adap_model.forces_samples_period = 0.0001
 #   adap_model.sTime = 2.0
    
    forces_torques.configure      
    adap_model.configure
    ang_vel.configure
    
    forces_torques.start
    adap_model.start
    ang_vel.start
    
   

 
   ## Defining the proxy for each task 
   parametersproxy      = Orocos::Async.proxy("adap_model")
   inputforcesproxy     = Orocos::Async.proxy("forces&torques")
   if dof == 'yaw'
        inputvelocitiesproxy = Orocos::Async.proxy("ang_vel")
        velocityport     = inputvelocitiesproxy.port("output")  
   else
        inputvelocitiesproxy = Orocos::Async.proxy("pose_estimator")  
        velocityport     = inputvelocitiesproxy.port("pose_samples")      
   end
	
   ## Defining the port variables using the proxys
   deltaVport       = parametersproxy.port("deltaV")
   nomrDeltaVport   = parametersproxy.port("normDeltaV")
   parametersport   = parametersproxy.port("parameters")      

   forcesport       = inputforcesproxy.port("forces")
        
    #open control widget and start replay
    supervisory = Supervisory.new(adap_model)
    supervisory.evolution(deltaVport, nomrDeltaVport, velocityport, forcesport, parametersport)
    
        
    supervisory.show
    
    Vizkit.control @log_replay
    Vizkit.exec
end
