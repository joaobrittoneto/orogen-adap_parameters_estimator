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
    adap_model          = @log.adap_model
    
    #########################################################

    widget1 = Vizkit.default_loader.Plot2d
    widget2 = Vizkit.default_loader.Plot2d
    widget3 = Vizkit.default_loader.Plot2d
    widget4 = Vizkit.default_loader.Plot2d
    widget5 = Vizkit.default_loader.Plot2d    
    
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
    
  
    
    adap_model.parameters.connect_to do |sample, _|
        widget1.update(sample.inertiaCoeff[0].positive, "Inertia")
        widget1.setYTitle("Inertia [Kg]")
        widget1.set_y_axis_scale(0,2000)
    end      
   
    adap_model.parameters.connect_to do |sample, _|
        widget2.update(sample.quadraticDampingCoeff[0].positive, "Q. damping")

        widget2.setYTitle("Quadratic Damping [Kg/m]")
        widget2.set_y_axis_scale(-10,30)        
    end
   
    adap_model.parameters.connect_to do |sample, _|
        widget3.update(sample.linearDampingCoeff[0].positive, "L. damping ")
        widget3.setYTitle("Linear Damping [Kg/s]")
        widget3.set_y_axis_scale(-20,80)
    end    
   
    adap_model.parameters.connect_to do |sample, _|
        widget4.update(sample.gravityAndBuoyancy[0], "Buoyancy")
        widget4.setYTitle("Buoyancy [N]")
        widget4.set_y_axis_scale(-10,10)
    end
    
    
    adap_model.deltaV.connect_to do |sample, _|
        widget5.update(sample, "Delta velocity")
        widget5.setYTitle("Delta velocity [m/s]")
        widget5.set_y_axis_scale(-0.1,0.1)
    end     
    
    widget1.show 
    widget2.show 
    widget3.show 
    widget4.show
    widget5.show
     
    Vizkit.control @log
    Vizkit.exec
end
