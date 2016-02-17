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
address2 = ARGV[1].to_s
address3 = ARGV[2].to_s
address4 = ARGV[3].to_s

@log = Orocos::Log::Replay.open(address1, address2, address3)
@log1 = Orocos::Log::Replay.open(address4)


#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linX.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linY.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linZ.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/angZ.log")


#######################################################################

Orocos.run  do

    
  #  Orocos.log_all
    
    #########################################################
    
    #adap_model     = @log_replay.adap_model
    adap_model_cam      = @log.adap_model
    adap_model_pose     = @log1.adap_model
    pose_estimator      = @log.pose_estimator    
    adap_samples        = @log.adap_samples_pose
    
    #########################################################

    widget1 = Vizkit.default_loader.Plot2d
    widget2 = Vizkit.default_loader.Plot2d
    widget3 = Vizkit.default_loader.Plot2d
    widget4 = Vizkit.default_loader.Plot2d
    widget5 = Vizkit.default_loader.Plot2d
    widget6 = Vizkit.default_loader.Plot2d
    widget7 = Vizkit.default_loader.Plot2d 
    widget8 = Vizkit.default_loader.Plot2d
    widget9 = Vizkit.default_loader.Plot2d    
    
    
    widget1.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget1.options[:yaxis_window] = 1
    #widget1.set_x_axis_scale(-0.5,0.5)
    widget1.setXTitle("time [s]")
    widget1.setTitle("Inertia evolution")
    
    widget2.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget2.options[:yaxis_window] = 1
    #widget2.set_x_axis_scale(-0.5,0.5)
    widget2.setXTitle("time [s]")
    widget2.setTitle("Q. Damping evolution")
    
        
    widget3.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget3.options[:yaxis_window] = 1
    #widget3.set_x_axis_scale(-0.5,0.5)
    widget3.setXTitle("time [s]")
    widget3.setTitle("L. Damping evolution")
    
        
    widget4.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget4.options[:yaxis_window] = 1
    #widget4.set_x_axis_scale(-0.5,0.5)
    widget4.setXTitle("time [s]")
    widget4.setTitle("Buoyancy evolution")
    
    widget5.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget5.options[:yaxis_window] = 1
    #widget5.set_x_axis_scale(-0.5,0.5)
    widget5.setXTitle("time [s]")
    widget5.setTitle("Delta velocity")    
    
    widget6.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget6.options[:yaxis_window] = 1
    #widget6.set_x_axis_scale(-0.5,0.5)
    widget6.setXTitle("time [s]")
    widget6.setTitle("Surge velocity")    
    
    
    widget7.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget7.options[:yaxis_window] = 1
    #widget7.set_x_axis_scale(-0.5,0.5)
    widget7.setXTitle("time [s]")
    widget7.setTitle("Cam surge acceleration")    
    
    
    widget8.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget8.options[:yaxis_window] = 1
    #widget8.set_x_axis_scale(-0.5,0.5)
    widget8.setXTitle("time [s]")
    widget8.setTitle("Force surge")
    
    widget9.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget9.options[:yaxis_window] = 1
    #widget9.set_x_axis_scale(-0.5,0.5)
    widget9.setXTitle("time [s]")
    widget9.setTitle("Cam surge position")         
    
    
  
    
    adap_model_cam.parameters.connect_to do |sample, _|
        widget1.update(sample.inertiaCoeff[0].positive, "cam_pose")
        widget1.setYTitle("Inertia [Kg]")
        widget1.set_y_axis_scale(0,2000)
        
        widget2.update(sample.quadraticDampingCoeff[0].positive, "cam_pose")
        widget2.setYTitle("Quadratic Damping [Kg/m]")
        widget2.set_y_axis_scale(-20,30) 
        
        widget3.update(sample.linearDampingCoeff[0].positive, "cam_pose")
        widget3.setYTitle("Linear Damping [Kg/s]")
        widget3.set_y_axis_scale(-40,120)
        
        widget4.update(sample.gravityAndBuoyancy[0], "cam_pose")
        widget4.setYTitle("Buoyancy [N]")
        widget4.set_y_axis_scale(-10,10)                 
    end      
   
    adap_model_pose.parameters.connect_to do |sample, _|
        widget1.update(sample.inertiaCoeff[0].positive, "pose_estimator")
        widget1.setYTitle("Inertia [Kg]")
        widget1.set_y_axis_scale(0,2000)
        
        widget2.update(sample.quadraticDampingCoeff[0].positive, "pose_estimator")
        widget2.setYTitle("Quadratic Damping [Kg/m]")
        widget2.set_y_axis_scale(-20,30) 
        
        widget3.update(sample.linearDampingCoeff[0].positive, "pose_estimator")
        widget3.setYTitle("Linear Damping [Kg/s]")
        widget3.set_y_axis_scale(-40,120)
        
        widget4.update(sample.gravityAndBuoyancy[0], "pose_estimator")
        widget4.setYTitle("Buoyancy [N]")
        widget4.set_y_axis_scale(-10,10)        
    end
   

    adap_samples.dynamic.connect_to do |sample, _|
        widget6.update(sample.rbs.velocity[0], "cam_pose")
        widget6.setYTitle("velocity [m/s]")
        widget6.set_y_axis_scale(-0.3,0.4)

#        widget7.update(sample.rba.acceleration[0], "cam surge acceleration")
#        widget7.setYTitle("acceleration [m/s^2]")
#        widget7.set_y_axis_scale(-0.1,0.1)
        
        widget8.update(sample.joints.elements[0].effort, "Force surge")
        widget8.setYTitle("Force [N]")
        widget8.set_y_axis_scale(-60,60)
        
#        widget9.update(sample.rbs.position[0], "cam surge position")
#        widget9.setYTitle("Position [m]")
#        widget9.set_y_axis_scale(-15,15)
    end 
    
    pose_estimator.pose_samples.connect_to do |sample, _|
        widget6.update(sample.velocity[0], "pose_estimator")
        widget6.setYTitle("velocity [m/s]")
        widget6.set_y_axis_scale(-0.3,0.4)
    end     
        
#    adap_model.deltaV.connect_to do |sample, _|
#        widget8.update(sample, "Delta velocity")
#        widget8.setYTitle("Delta velocity [m/s]")
#        widget8.set_y_axis_scale(-0.1,0.1)
#    end         
    
    widget1.show 
    widget2.show 
    widget3.show 
    widget4.show
#    widget5.show
    widget6.show
#    widget7.show
    widget8.show 
#    widget9.show                
     
    Vizkit.control @log
    Vizkit.control @log1    
    Vizkit.exec
end
