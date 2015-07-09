#library for displaying data
require 'orocos'
require 'vizkit'
require './gui/gui_evaluation.rb'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 

#Orocos.initialize

######################################################################
#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/angY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/all_in_one.log")
#######################################################################
#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1743/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1707/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1519/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1615/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1635/angZ.log")

@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1703/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1724/angZ.log")

#######################################################################

Orocos.run 'uwv_motion_model::Task'                     => 'motion_model',
           'adap_parameters_estimator::ForceApplierModel'    => 'forces&torques',
           'adap_parameters_estimator::Evaluation'      => 'evaluation' do
           
    widget = Vizkit.default_loader.Plot2d 
    widget2 = Vizkit.default_loader.Plot2d      

    model               = TaskContext.get 'motion_model'
    forces_torques      = TaskContext.get 'forces&torques'
    eval                = TaskContext.get 'evaluation'

    #model.apply_conf_file('config/uwv_motion_model::Task.yml',['dagon_input_trhuster']) 
    model.apply_conf_file('config/uwv_motion_model::Task.yml',['dagon']) 
    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplierModel.yml',['dagon']) 
    eval.apply_conf_file('config/adap_parameters_estimator::Evaluation.yml',['yaw'])
   
#    pose_estimation     = @log_replay.pose_estimator   # old log files (20150303)
    pose_estimation     = @log_replay.pose_estimation   # new log files (20150409)   
    dispatcher          = @log_replay.dispatcher   
    
    dispatcher.all_joint_state.connect_to       forces_torques.thruster_samples   
    #dispatcher.all_joint_state.connect_to       model.cmd_in
    forces_torques.forces.connect_to            model.cmd_in,           :type => :buffer, :size => 100
    
    pose_estimation.pose_samples.connect_to     eval.measured_velocity, :type => :buffer, :size => 100
    model.cmd_out.connect_to                    eval.model_velocity,    :type => :buffer, :size => 100         
    
 #  model.cmd_out.connect_to do |sample|
 #       widget.update(sample.velocity[0], "model_vel_x [m/s]")
 #  end
   
 #  pose_estimation.pose_samples.connect_to do |sample|
 #       widget.update(sample.velocity[0], "pose_vel_x [m/s]")
 #  end
   
   forces_torques.forces.connect_to do |sample|
        widget2.update(sample.elements[0].effort, "Effort_x [N]")
   end

 
   ## Defining the proxy for each task 
#   inputproxy = Orocos::Async.proxy("pose_estimator")  # old log files (20150303)
   inputproxy = Orocos::Async.proxy("pose_estimation")  # new log files (20150409)
   speedproxy = Orocos::Async.proxy("motion_model")
   evalproxy = Orocos::Async.proxy("evaluation")
   
	
   ## Defining the port variables using the proxys
  
   velocityModelport            = speedproxy.port("cmd_out")
   velocityMeasuredport         = inputproxy.port("pose_samples")
   errorvport                   = evalproxy.port("error_velocity")
   maevport                     = evalproxy.port("mae_velocity")
   normmaevport                 = evalproxy.port("norm_mae_velocity")    
        
    #open control widget and start replay
    supervisory = Supervisory.new()
    supervisory.evaluation(velocityModelport,velocityMeasuredport, errorvport, maevport, normmaevport)
    
    eval.aggregator_max_latency = 0.5
    eval.measured_velocity_period = 0.001
    eval.model_velocity_period = 0.01
    
    forces_torques.configure      
    model.configure
    eval.configure
    
    eval.start
    forces_torques.start
    model.start
    
      
  #  widget.show
    widget2.show    
        
                
    supervisory.show
   
   
   
   Vizkit.control @log_replay
   Vizkit.exec
   
   #@log_replay.run#(true,1)
   
   
end
