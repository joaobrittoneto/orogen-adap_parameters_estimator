if ARGV.empty?
    puts "ERROR: missing argument: You have to inform the log path"
    exit
end

#library for displaying data
require 'orocos'
require 'vizkit'

include Orocos

    

#Orocos::CORBA.name_service = "192.168.128.50"  # Flatfish

#load log file 
#######################################################################
address1 = ARGV[0].to_s


#@log2 = Orocos::Log::Replay.open(address2)
#@log3 = Orocos::Log::Replay.open(address3)
#@log4 = Orocos::Log::Replay.open(address4)
#@model = Orocos::Log::Replay.open(address)

#@log1 = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linX.log", address1)
#@log1 = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linY.log", address1)
#@log1 = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linZ.log", address1)
@log1 = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/angZ.log", address1)


#######################################################################

Orocos.run  do

    
  #  Orocos.log_all
    
    #########################################################
    
    pose_estimator              = @log1.pose_estimator
    orientation_estimator       = @log1.orientation_estimator
    models                      = @log1.motion_model
   # model_adap_pose             = @log1.motion_model
   # model_LS_pose               = @log2.motion_model
   # model_adap_cam              = @log3.motion_model
   # model_LS_cam                = @log4.motion_model            
    
    
    #########################################################

    widget = Vizkit.default_loader.Plot2d
    
    widget.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget.options[:yaxis_window] = 1
   # widget.set_x_axis_scale(-0.5,0.5)
    widget.setXTitle("time [s]")
    widget.setTitle("Yaw velocity")
    
    
    models.cmd_out.connect_to do |sample, _|     
        widget.update(sample.angular_velocity[2], "adaptative pose estimator")
        widget.setYTitle("angular velocity [rad/s]")
        widget.set_y_axis_scale(-0.1,0.1)
    end
    
    models.cmd_out_1.connect_to do |sample, _|     
        widget.update(sample.angular_velocity[2], "LS pose estimator")
        widget.setYTitle("angular velocity [rad/s]")
        widget.set_y_axis_scale(-0.1,0.1)
    end
    
    models.cmd_out_2.connect_to do |sample, _|     
        widget.update(sample.angular_velocity[2], "adaptative cam pose")
        widget.setYTitle("angular velocity [rad/s]")
        widget.set_y_axis_scale(-0.1,0.1)
    end
    
    models.cmd_out_3.connect_to do |sample, _|     
        widget.update(sample.angular_velocity[2], "LS cam pose")
        widget.setYTitle("angular velocity [rad/s]")
        widget.set_y_axis_scale(-0.1,0.1)
    end 
                   
    
#    pose_estimator.pose_samples.connect_to do |sample, _|     
#        widget.update(sample.angular_velocity[2], "measurement")
#        widget.setYTitle("angular velocity [rad/s]")
#        widget.set_y_axis_scale(-0.15,0.15)
#    end
    
    orientation_estimator.orientation_samples_out.connect_to do |sample, _|     
        widget.update(sample.angular_velocity[2], "measurement")
        widget.setYTitle("angular velocity [rad/s]")
        widget.set_y_axis_scale(-0.1,0.1)
    end    

    widget.show 
     
   
    Vizkit.control @log1
  #  Vizkit.control @log2
  #  Vizkit.control @log3
  #  Vizkit.control @log4

    #@log1.run(true,1)
    #@log2.run(true,1)
   # @log3.run(true,1)
   # @log4.run(true,1)
                
    Vizkit.exec
end
