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
address = ARGV[0].to_s

@log = Orocos::Log::Replay.open(address)

#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linX.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linY.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linZ.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/angZ.log")


#######################################################################

Orocos.run  do

    
  #  Orocos.log_all
    
    #########################################################
    
    #adap_model     = @log_replay.adap_model
    pose_estimator      = @log.pose_estimator
    cam_pose            = @log.apriltag_detector 
    dvl                 = @log.dvl_seapilot
    
    #########################################################

    widget1 = Vizkit.default_loader.Plot2d
    widget2 = Vizkit.default_loader.Plot2d
    widget3 = Vizkit.default_loader.Plot2d
    widget4 = Vizkit.default_loader.Plot2d
    
    widget1.options[:cached_time_window] = 400              #window size during auto scrolling    
    widget1.options[:yaxis_window] = 1
    #widget1.set_x_axis_scale(-0.5,0.5)
    widget1.setXTitle("time [s]")
    widget1.setTitle("Velocities measured [m/s]")
    
    widget2.options[:cached_time_window] = 400              #window size during auto scrolling    
    widget2.options[:yaxis_window] = 1
    #widget2.set_x_axis_scale(-0.5,0.5)
    widget2.setXTitle("time [s]")
    widget2.setTitle("Cam pose [m]")
    
        
    widget3.options[:cached_time_window] = 400              #window size during auto scrolling    
    widget3.options[:yaxis_window] = 1
    #widget3.set_x_axis_scale(-0.5,0.5)
    widget3.setXTitle("time [s]")
    widget3.setTitle("pose estimator [m]")
    
        
    widget4.options[:cached_time_window] = 400              #window size during auto scrolling    
    widget4.options[:yaxis_window] = 1
    #widget4.set_x_axis_scale(-0.5,0.5)
    widget4.setXTitle("time [s]")
    widget4.setTitle("DVL velocity [m]")
    
  
    
    pose_estimator.pose_samples.connect_to do |sample, _|
        widget1.update(sample.velocity[0], "Surge velocity")
        widget1.update(sample.velocity[1], "Sway velocity")
        widget1.update(sample.velocity[2], "Heave velocity")
        widget1.setYTitle("velocity [m/s]")
        widget1.set_y_axis_scale(-0.3,0.45)
    end      
   
   cam_pose.marker_poses.connect_to do |sample, _|
        widget2.update(sample.position[0], "Surge position")
        widget2.update(sample.position[1], "Sway position")
        widget2.update(sample.position[2], "Heave position")
        widget2.setYTitle("Pose [m]")
       # widget2.set_y_axis_scale(-0.3,0.45)        
   end
   
    pose_estimator.pose_samples.connect_to do |sample, _|
        widget3.update(sample.position[0], "Surge position")
        widget3.update(sample.position[1], "Sway position")
        widget3.update(sample.position[2], "Heave position")
        widget3.setYTitle("Pose [m]")
       # widget1.set_y_axis_scale(-0.3,0.45)
    end    
   
    dvl.velocity_samples.connect_to do |sample, _|
        widget4.update(sample.velocity[0], "Surge velocity")
        widget4.update(sample.velocity[1], "Sway velocity")
        widget4.update(sample.velocity[2], "Heave velocity")
        widget4.setYTitle("velocity dvl [m/s]")
        widget4.set_y_axis_scale(-0.3,0.45)
    end 
    
    widget1.show 
    widget2.show 
    widget3.show 
    widget4.show
     
    Vizkit.control @log
    Vizkit.exec
end
