#library for displaying data
require 'orocos'
require 'vizkit'
require './gui/gui_evaluation.rb'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.50"  # Flatfish

#load log file 
#######################################################################

#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linX.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linY.log")
@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linZ.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/angZ.log")

#######################################################################



#Orocos.initialize


Orocos.run 'uwv_motion_model::Task'                             => 'motion_model',
           'adap_parameters_estimator::ForceApplierModel'       => 'forces&torques',
           'adap_parameters_estimator::Evaluation'              => 'evaluation' do

 

    model               = TaskContext.get 'motion_model'
    forces_torques      = TaskContext.get 'forces&torques'
    eval                = TaskContext.get 'evaluation'  
    
    Orocos.log_all
    
    #########################################################
    
    pose_estimator     = @log_replay.pose_estimator
    thurster           = @log_replay.acceleration_controller     
     
    #########################################################
    
    dof = 'heave'    
        
    model.apply_conf_file('config/uwv_motion_model::Task.yml',['flatfish']) 
    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplierModel.yml',['flatfish']) 
    eval.apply_conf_file('config/adap_parameters_estimator::Evaluation.yml',['default', dof])
 
    
    thurster.cmd_out.connect_to                 forces_torques.thruster_samples, :type => :buffer, :size => 100    
    forces_torques.forces.connect_to            model.cmd_in,           :type => :buffer, :size => 100
    
                
      
    pose_estimator.pose_samples.connect_to     eval.measured_velocity, :type => :buffer, :size => 100
    model.cmd_out.connect_to                    eval.model_velocity,    :type => :buffer, :size => 100         
    

 
   ## Defining the proxy for each task 



  
   speedproxy = Orocos::Async.proxy("motion_model")
   evalproxy = Orocos::Async.proxy("evaluation")
   inputproxy = Orocos::Async.proxy("pose_estimator")   
	
   ## Defining the port variables using the proxys
  
   velocityModelport            = speedproxy.port("cmd_out")

   errorvport                   = evalproxy.port("error_velocity")
   maevport                     = evalproxy.port("mae_velocity")
   normmaevport                 = evalproxy.port("norm_mae_velocity")    
   velocityMeasuredport         = inputproxy.port("pose_samples")      
        
    #open control widget and start replay
    dof = eval.dof
    #dof_aux = :SURGE
    supervisory = Supervisory.new(dof)
    supervisory.evaluation(velocityModelport,velocityMeasuredport, errorvport, maevport, normmaevport)
    
#    eval.aggregator_max_latency = 0.5
#    eval.measured_velocity_period = 0.0001
#    eval.model_velocity_period = 0.0001
    
    forces_torques.configure      
    model.configure
    eval.configure

    eval.start
    forces_torques.start
    model.start
     
        
                
    supervisory.show
   
   
   
   Vizkit.control @log_replay
   Vizkit.exec
   
   #@log_replay.run#(true,1)
   
   
end
