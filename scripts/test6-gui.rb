#***********************************************************************/
# Ruby's code for adaptive parameters identification of auv            */
#                                                                      */
#       Input thruster come from sinus function and input velocity     */
#       come from model                                                */
#       Implement vizkit for vizualition                               */
#                                                                      */ 
# FILE --- test.rb                                                     */
#                                                                      */
# PURPOSE --- Tests for the adaptive method component                  */
#                                                                      */
#  João Da Costa Britto Neto                                           */
#  joao.neto@dfki.de                                                   */
#  DFKI - BREMEN 2014                                                  */
#***********************************************************************/


require 'orocos'
require 'vizkit'
require './adap_properties2.rb'
require './model_properties.rb'
require './force_properties.rb'
require './parameterarray.rb'
require './dofarray.rb'
require './plot.rb'
require './plot_inertia.rb'
require './gui/ui_parametersStatus.rb'

include Orocos

Orocos.initialize

Orocos.run 'adap_parameters_estimator::Task' => 'adap_parameters',
           'forces_torques_gen::Task' => 'forces_torques',
           'motion_model::Task' => 'motion_model'   do
             
#        widget = Vizkit.default_loader.Plot2d
#        widget = Vizkit.default_loader.Plot2d
#        widget = Vizkit.default_loader.Plot2d
#        widget = Vizkit.default_loader.Plot2d

	## Get the specific task context ##
	adapP = TaskContext.get 'adap_parameters'
        model = TaskContext.get 'motion_model'
        forcesTorques = TaskContext.get 'forces_torques'
	
	puts "teste"
	
	
	thrusterInput = Types::Base::Samples::Joints.new 
	jointStates = Types::Base::JointState.new
	
	# initialising the adaptive method properties (from adap_properties.rb)	
        configure_adap = adapP
        adap_properties(configure_adap)
        adapP = configure_adap
        
        configure_model = model
        model_properties(configure_model)
        model = configure_model
        
        forces = forcesTorques
        force_properties(forces)
        forcesTorques = forces
	
       puts "teste1"
        
	##########################################################################
	#	     SETTING INITIAL POSITION, VELOCITY AND LINEAR CONTROL VALUES
	##########################################################################

       # numberThruster = adapP.thrusterMatrix.size/6
	#for i in 0..(numberThruster-1)
	#	thrusterInput.elements << jointStates	# Creates a new jointState element
	#	thrusterInput.elements[i].effort = 0	# Gets the array value for the new jointState.effort element
	#end

        	
       

	##########################################################################
	#		                    COMPONENT INPUT PORTS
	##########################################################################

  
	## Writers
	#thrusterSampleWriter_adap = adapP.thruster_samples.writer
	#thrusterSampleWriter_model = model.thruster_samples.writer
	
	

	# Samples
	#thrusterSample_adap = thrusterSampleWriter_adap.new_sample
	#thrusterSample_model = thrusterSampleWriter_model.new_sample
	

	# Loading the values into the sample variables
	#thrusterSample_adap = thrusterInput
	#thrusterSample_model = thrusterInput
	
	# Connecting the ports
	forcesTorques.forces.connect_to model.thruster_samples
	forcesTorques.forces.connect_to adapP.thruster_samples
	model.velocity.connect_to adapP.speed_samples
	
	
        puts "teste2"
		
	# Configuring and starting the component
  forcesTorques.configure
  adapP.configure
  model.configure

puts "teste3"


  #forcesTorques.start
  #adapP.start
  model.start
  
	# Writing the sample variables on the input ports
	#thrusterSampleWriter_adap.write(thrusterSample_adap)
	#thrusterSampleWriter_model.write(thrusterSample_model)
	
		

	##########################################################################
	#		                    COMPONENT OUTPUT PORT
	##########################################################################
        
        #parametersReader = adapP.parameters.reader 
        #deltavReader = adapP.deltaV.reader 
        #velocityReader = model.velocity.reader
        
               
        #velocitySamplesArray = Array.new {Array.new(6)}      
        #auxiliarArray = Array.new(4)
        #parametersArray = Array.new
        #deltaVSamplesArray = Array.new
	#tauArray = Array.new
		
	#k = 0
        #n = 1
	
	@dof = adapP.dofs
	puts @dof
	puts adapP.dofs
	
	
	## Defining the proxy for each task 
	parametersproxy = Orocos::Async.proxy("adap_parameters")
	velocityproxy = Orocos::Async.proxy("motion_model")
	thrusterproxy = Orocos::Async.proxy("forces_torques")
	
        ## Defining the port variables using the proxys
	deltaVport       = parametersproxy.port("deltaV")
	nomrDeltaVport   = parametersproxy.port("normDeltaV")
	parametersport   = parametersproxy.port("parameters")
	velocityport     = velocityproxy.port("velocity")
	forcesport       = thrusterproxy.port("forces")
	
	puts "test"
	
	
       # adapP.parameters.connect_to do |sample, _|
       #        widget.update sample.inertiaCoeff[0].positive, "Ix"
        #	puts sample.inertiaCoeff[0].positive
    	#end
	
	
	supervisory = Supervisory.new(adapP, model, forcesTorques)
	supervisory.evolution(deltaVport, nomrDeltaVport, velocityport, forcesport, parametersport)
	puts "test1"
	
        supervisory.show
	# widget.show 
	
	puts "test2"
        Vizkit.exec 
	
	       
      
       

end
