#library for displaying data
require 'orocos'
require 'vizkit'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.50"  # Flatfish

#load log file 
#######################################################################

@model = Orocos::Log::Replay.open("adap_model.0.log", "properties.0.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linX.log")
@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linY.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linZ.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/angZ.log")


#######################################################################

Orocos.run  do

    
  #  Orocos.log_all
    
    #########################################################
    
    #adap_model     = @log_replay.adap_model
    model           = @model
    
    
    #########################################################

    widget = Vizkit.default_loader.Plot2d
    widget.options[:cached_time_window] = 100              #window size during auto scrolling    
   
    gainA = Eigen::VectorX.new(6)
    
#    adap_model.parameters.connect_to do |sample, _|        
#        widget.update(sample.inertiaCoeff[2].positive, "inertia [Kg]")
#       gainA = adap_model.gA 
#        puts gainA
#    end 
    
    model.cmd_out.connect_to do |sample, _|
        widget.update(sample.velocity[0], "inertia [Kg]")
    end      
   
    
    widget.show 
     
    Vizkit.control @log_replay
    Vizkit.exec
end
