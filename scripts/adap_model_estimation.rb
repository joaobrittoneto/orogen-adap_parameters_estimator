#library for displaying data
require 'orocos'
require 'vizkit'
require './gui/gui_parameters.rb'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 
#######################################################################
@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1743/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1707/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1519/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1615/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1635/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1703/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1724/angZ.log")

#######################################################################

Orocos.run 'adap_parameters_estimator::ForceApplier'        => 'forces&torques',
           'adap_parameters_estimator::AdapModelEstimation' => 'adap_model' do


    forces_torques      = TaskContext.get 'forces&torques'
    adap_model          = TaskContext.get 'adap_model'

    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplier.yml',['dagon']) 
    adap_model.apply_conf_file('config/adap_parameters_estimator::AdapModelEstimation.yml',['dagon'])
    
 
    pose_estimation     = @log_replay.pose_estimation
    dispatcher          = @log_replay.dispatcher   
    

    dispatcher.all_joint_state.connect_to       forces_torques.thruster_samples,:type => :buffer, :size => 100   
          
    pose_estimation.pose_samples.connect_to     adap_model.pose_samples,        :type => :buffer, :size => 100 
    forces_torques.forces.connect_to            adap_model.forces_samples,      :type => :buffer, :size => 100      
    
    
    forces_torques.configure      
    adap_model.configure
    forces_torques.start
    adap_model.start
    
    
   

 
   ## Defining the proxy for each task 
   parametersproxy      = Orocos::Async.proxy("adap_model")
   inputforcesproxy     = Orocos::Async.proxy("forces&torques")
   inputvelocitiesproxy = Orocos::Async.proxy("pose_estimation")        
   
	
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
    
    Vizkit.control @log_replay
    Vizkit.exec
end
