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


@log = Orocos::Log::Replay.open(address1)
@log1 = Orocos::Log::Replay.open(address2)


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
    
    #########################################################

    widget7 = Vizkit.default_loader.Plot2d

    
   
    widget7.options[:cached_time_window] = 1000              #window size during auto scrolling    
    widget7.options[:yaxis_window] = 1
    #widget7.set_x_axis_scale(-0.5,0.5)
    widget7.setXTitle("time [s]")
    widget7.setTitle("Normalized error evolution")    
    
  
    
  
    adap_model_pose.normDeltaV.connect_to do |sample, _|
        widget7.update(sample, "Pose estimator")
        widget7.setYTitle("Normalized velocity error")
     #   widget7.set_y_axis_scale(-0.1,0.1)
    end  
    
    adap_model_cam.normDeltaV.connect_to do |sample, _|
        widget7.update(sample, "Cam pose")
        widget7.setYTitle("Normalized velocity error")
        #widget7.set_y_axis_scale(-0.1,0.1)
    end           
    
#    widget1.show 
#    widget2.show 
#    widget3.show 
#    widget4.show
#    widget5.show
#    widget6.show
    widget7.show
#    widget8.show 
#    widget9.show                
     
    Vizkit.control @log
    Vizkit.control @log1    
    Vizkit.exec
end
