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

Orocos.run 'adap_samples_pose', 
           'adap_parameters_estimator::ForceApplier'        => 'forces&torques',
           'adap_parameters_estimator::AdapModelEstimation' => 'adap_model',
           'filter_apriltag_detector::Task'                 => 'filter'   do


    forces_torques      = TaskContext.get 'forces&torques'
    adap_model          = TaskContext.get 'adap_model'
    adap_samples_pose   = TaskContext.get 'adap_samples_pose'
    ls_pose             = TaskContext.get 'ls_method_pose'
    filter             = TaskContext.get 'filter'    
    
    Orocos.log_all

    #########################################################
    pose_estimator      = @log_replay.pose_estimator
    cam_pose            = @log_replay.apriltag_detector 
    thurster            = @log_replay.acceleration_controller      
    #########################################################
    
    dof = 'heave'

    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplier.yml',['flatfish']) 
    adap_model.apply_conf_file('config/adap_parameters_estimator::AdapModelEstimation.yml',['flatfish_cam', dof])
    adap_samples_pose.apply_conf_file('./config/adap_samples_input::Task.yml',['cam_pose'])
    filter.apply_conf_file('./config/filter.yml',['default'])

 
    ##########################################################################
 
    thurster.cmd_out.connect_to       forces_torques.thruster_samples  
    
    forces_torques.forces.connect_to            adap_samples_pose.forces_samples,       :type => :buffer, :size => 100       
    cam_pose.marker_poses.connect_to            filter.pose_sample
    filter.output.connect_to                    adap_samples_pose.pose_samples,         :type => :buffer, :size => 100      
 
        
    adap_samples_pose.velocities.connect_to     adap_model.pose_samples,        :type => :buffer, :size => 100 
    adap_samples_pose.forces.connect_to         adap_model.forces_samples,      :type => :buffer, :size => 100     
          
#    pose_estimator.pose_samples.connect_to      adap_model.pose_samples,        :type => :buffer, :size => 100 
#    forces_torques.forces.connect_to            adap_model.forces_samples,      :type => :buffer, :size => 100      
    
   # adap_model.aggregator_max_latency = 2.0
   # adap_model.pose_samples_period = 0.0001
   # adap_model.forces_samples_period = 0.0001
   # adap_model.sTime = 2.0
    
    if dof == 'heave' 
        adap_samples_pose.number_samples = 41
    end
    
    forces_torques.configure  
    adap_samples_pose.configure    
    adap_model.configure
    filter.configure
           
    forces_torques.start
    adap_samples_pose.start
    adap_model.start
    filter.start
    
   

 
   ## Defining the proxy for each task 
   parametersproxy      = Orocos::Async.proxy("adap_model")
   inputproxy           = Orocos::Async.proxy("adap_samples_pose")
   
   
	
   ## Defining the port variables using the proxys
   deltaVport       = parametersproxy.port("deltaV")
   nomrDeltaVport   = parametersproxy.port("normDeltaV")
   parametersport   = parametersproxy.port("parameters")      
   velocityport     = inputproxy.port("velocities")
   forcesport       = inputproxy.port("forces")
        
    #open control widget and start replay
    supervisory = Supervisory.new(adap_model)
    supervisory.evolution(deltaVport, nomrDeltaVport, velocityport, forcesport, parametersport)
    
        
    supervisory.show
    
    Vizkit.control @log_replay
    Vizkit.exec
end
