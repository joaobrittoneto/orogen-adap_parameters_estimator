#library for displaying data
require 'orocos'
require 'vizkit'
require './gui/gui_evaluation.rb'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 

#Orocos.initialize

#######################################################################
@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1743/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1707/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1519/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1615/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1635/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1703/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1724/angZ.log")

#######################################################################

Orocos.run 'motion_model1', 
#           'motion_model2',
           'adap_parameters_estimator::ForceApplier'    => 'forces&torques',
#           'adap_samples_input::GetPoseForcePeriodic'   => 'get_pose_force',
           'adap_parameters_estimator::Evaluation'      => 'evaluation' do
           
    widget = Vizkit.default_loader.Plot2d 
    widget2 = Vizkit.default_loader.Plot2d 
    widget3 = Vizkit.default_loader.Plot2d     

    model1              = TaskContext.get 'motion_model1'
#    model2              = TaskContext.get 'motion_model2'
    forces_torques      = TaskContext.get 'forces&torques'
#    input_data          = TaskContext.get 'get_pose_force'
    eval                = TaskContext.get 'evaluation'

    #model.apply_conf_file('config/uwv_motion_model::Task.yml',['dagon_input_trhuster']) 
    model1.apply_conf_file('config/uwv_motion_model::Task.yml',['dagon']) 
#    model2.apply_conf_file('config/uwv_motion_model::Task.yml',['dagon'])
    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplier.yml',['dagon']) 
    eval.apply_conf_file('config/adap_parameters_estimator::Evaluation.yml',['surge'])
#    input_data.apply_conf_file('config/adap_samples_input::GetPoseForcePeriodic.yml',['dagon'])

   
    pose_estimation     = @log_replay.pose_estimation
    dispatcher          = @log_replay.dispatcher   
    
    dispatcher.all_joint_state.connect_to       forces_torques.thruster_samples   
#    forces_torques.forces.connect_to            input_data.forces_samples,      :type => :buffer, :size => 1000
#    pose_estimation.pose_samples.connect_to     input_data.pose_samples,        :type => :buffer, :size => 1000
    
    forces_torques.forces.connect_to            model1.cmd_in,                  :type => :buffer, :size => 100
#    input_data.effort.connect_to                model1.cmd_in,                  :type => :buffer, :size => 100
#    forces_torques.forces.connect_to            model2.cmd_in,                  :type => :buffer, :size => 100
#    input_data.effort.connect_to                model2.cmd_in,                  :type => :buffer, :size => 100
    
#    model2.cmd_out.connect_to                   eval.measured_velocity,         :type => :buffer, :size => 1000
    pose_estimation.pose_samples.connect_to     eval.measured_velocity,         :type => :buffer, :size => 1000
#    input_data.velocity.connect_to              eval.measured_velocity,         :type => :buffer, :size => 1000
    model1.cmd_out.connect_to                   eval.model_velocity,            :type => :buffer, :size => 1000         
    
   
   forces_torques.forces.connect_to do |sample|
        widget.update(sample.elements[0].effort, "Effort_x_NO_F [N]")
   end
   
#   input_data.effort.connect_to do |sample|
#        widget.update(sample.elements[0].effort, "Effort_x [N]")
#   end
   
#   pose_estimation.pose_samples.connect_to do |sample|
#        widget2.update(sample.velocity[0], "Vel_x_NO_F [m/s]")
#   end
   
#   input_data.velocity.connect_to do |sample|
#        widget2.update(sample.velocity[0], "Vel_x [m/s]")
#   end   
   
#   model1.cmd_out.connect_to do |sample|
#        widget3.update(sample.velocity[0], "V-Mo_NO_F [m/s]")
#   end
   
#   model2.cmd_out.connect_to do |sample|
#        widget3.update(sample.velocity[0], "V-Mo [m/s]")
#   end   
        


 
   ## Defining the proxy for each task 
   inputproxy = Orocos::Async.proxy("pose_estimation")
#   inputproxy = Orocos::Async.proxy("motion_model2")
#  inputproxy = Orocos::Async.proxy("get_pose_force")   
   modelproxy = Orocos::Async.proxy("motion_model1")
   evalproxy = Orocos::Async.proxy("evaluation")
   
	
   ## Defining the port variables using the proxys
  

   velocityMeasuredport         = inputproxy.port("pose_samples")
#   velocityMeasuredport         = inputproxy.port("cmd_out")
#   velocityMeasuredport         = inputproxy.port("velocity")   
   velocityModelport            = modelproxy.port("cmd_out")       
   errorvport                   = evalproxy.port("error_velocity")
   maevport                     = evalproxy.port("mae_velocity")
   normmaevport                 = evalproxy.port("norm_mae_velocity")    
        
    #open control widget and start replay
    supervisory = Supervisory.new()
    supervisory.evaluation(velocityModelport,velocityMeasuredport, errorvport, maevport, normmaevport)
    
    eval.aggregator_max_latency = 1.0
    eval.measured_velocity_period = 0.001
    eval.model_velocity_period = 0.001
    
    forces_torques.configure      
    model1.configure
#    model2.configure
#    input_data.configure
    eval.configure
    
#    input_data.start
    eval.start
    forces_torques.start
    model1.start
#    model2.start
    
      
    widget.show
#    widget2.show 
#    widget3.show   
        
                
    supervisory.show
   
   
   
   Vizkit.control @log_replay
   Vizkit.exec
   
   #@log_replay.run#(true,1)
   
   
end
