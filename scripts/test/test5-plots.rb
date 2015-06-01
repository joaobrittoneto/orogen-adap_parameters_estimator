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
require './parameterarray.rb'
require './dofarray.rb'
require './plot.rb'
require './plot_inertia.rb'

include Orocos

Orocos.initialize

Orocos.run 'adap_parameters_estimator::Task' => 'adap_parameters',
           'motion_model::Task' => 'motion_model'   do
             
        widget = Vizkit.default_loader.Plot2d
#        widget = Vizkit.default_loader.Plot2d
#        widget = Vizkit.default_loader.Plot2d
#        widget = Vizkit.default_loader.Plot2d

	## Get the specific task context ##
	adapP = TaskContext.get 'adap_parameters'
        model = TaskContext.get 'motion_model'

	
	
	
	
	thrusterInput = Types::Base::Samples::Joints.new 
	jointStates = Types::Base::JointState.new
	
	# initialising the adaptive method properties (from adap_properties.rb)	
        configure_adap = adapP
        adap_properties2(configure_adap)
        adapP = configure_adap
        
        configure_model = model
        model_properties(configure_model)
        model = configure_model
	
       
        
	##########################################################################
	#	     SETTING INITIAL POSITION, VELOCITY AND LINEAR CONTROL VALUES
	##########################################################################

        #numberThruster = adapP.thrusterMatrix.size/6
	for i in 0..5 #(numberThruster-1)
		thrusterInput.elements << jointStates	# Creates a new jointState element
		thrusterInput.elements[i].effort = 0	# Gets the array value for the new jointState.effort element
	end

        	
       

	##########################################################################
	#		                    COMPONENT INPUT PORTS
	##########################################################################

  
	## Writers
	thrusterSampleWriter_adap = adapP.thruster_samples.writer
	thrusterSampleWriter_model = model.thruster_samples.writer
	
	

	# Samples
	thrusterSample_adap = thrusterSampleWriter_adap.new_sample
	thrusterSample_model = thrusterSampleWriter_model.new_sample
	

	# Loading the values into the sample variables
	thrusterSample_adap = thrusterInput
	thrusterSample_model = thrusterInput
	
	# Connecting the ports
	model.velocity.connect_to adapP.speed_samples
	
	

		
	# Configuring and starting the component
  adapP.configure
  model.configure
  adapP.start
  model.start
  
	# Writing the sample variables on the input ports
	thrusterSampleWriter_adap.write(thrusterSample_adap)
	thrusterSampleWriter_model.write(thrusterSample_model)
	
		

	##########################################################################
	#		                    COMPONENT OUTPUT PORT
	##########################################################################
        
        parametersReader = adapP.parameters.reader 
        deltavReader = adapP.deltaV.reader 
        velocityReader = model.velocity.reader
        
               
        velocitySamplesArray = Array.new {Array.new(6)}      
        auxiliarArray = Array.new(4)
        parametersArray = Array.new
        deltaVSamplesArray = Array.new
	tauArray = Array.new
		
	k = 0
        n = 0
	
	until k > 120 do
#=begin
                
                thrusterInput.elements[0].effort = 2*(50*Math.sin(k*adapP.ftau) + 50)
	       # thrusterInput.elements[1].effort = -(50*Math.sin(k*adapP.ftau) + 50)
	       # thrusterInput.elements[2].effort = 50*Math.sin(k*adapP.ftau) + 50
	       # thrusterInput.elements[5].effort = 15*Math.sin(k*adapP.ftau) + 15
	        
	
	        # Loading the values into the sample variables
	        thrusterSample = thrusterInput
	        #tauArray << (adapP.thrusterMatrix * thrusterInput.elements.effort)[n]
	        tauArray << thrusterInput.elements[n].effort
	        
		# Writing the sample variables on the input ports
	        thrusterSampleWriter_model.write(thrusterSample)
	        thrusterSampleWriter_adap.write(thrusterSample)
                
                
                
                
                			
	        if parametersSample = parametersReader.read_new
                        puts "\n\nparameters: \n"
	                puts "\n", parametersSample.inertiaCoeff[n].positive
	                puts "\n", parametersSample.quadraticDampingCoeff[n].positive
	                puts "\n", parametersSample.linearDampingCoeff[n].positive
	                puts "\n", parametersSample.gravityAndBuoyancy[n]
	      
	             auxiliarArray    +=  [parametersSample.inertiaCoeff[n].positive]
	             auxiliarArray    +=  [parametersSample.quadraticDampingCoeff[n].positive]
	             auxiliarArray    +=  [parametersSample.linearDampingCoeff[n].positive]
	             auxiliarArray    +=  [parametersSample.gravityAndBuoyancy[n]]
	             
	             4.times {auxiliarArray.shift}
	             
	             parametersArray    <<      auxiliarArray
	                      
	                	                               	                
	        end	
		        
                 
	        
	         if newVelocitySample = velocityReader.read_new
                         velocitySamplesArray << newVelocitySample.velocity.to_a + newVelocitySample.angular_velocity.to_a             
                 end
                 
                 if newdeltaVSample = deltavReader.read_new
                         deltaVSamplesArray << newdeltaVSample
                         
                         puts "\n\n deltaV"
                         puts newdeltaVSample
                 end
	        
	           
	        
		k = k + adapP.sTime	
#=end
	end
       
       
       puts "\n\nparametersArray\n"
       puts parametersArray.size
       
       #puts "\n\ntauArray\n"
       #puts tauArray.size
       
       puts "\n\nvelocitySamplesArray\n"
       puts velocitySamplesArray.size

       puts "\n\ndeltaVSamplesArray\n"
       puts deltaVSamplesArray.size  
       
       puts "\n\nK\n"
       puts k/adapP.sTime       
       
       parametersArray.shift(10)
       
       plotSamples = parameterarray(parametersArray,:INERTIA) # [:INERTIA, :QUADRATIC, :LINEAR, :BUOYANCY]
       plot_inertia(plotSamples, adapP.sTime, "INERTIA")
       
       puts "\n\nMin inercia\n"
       puts plotSamples.min 
       
       plotSamples = parameterarray(parametersArray,:QUADRATIC)
       plot(plotSamples, adapP.sTime, "QUADRATIC DAMPING")
       
       plotSamples = parameterarray(parametersArray,:LINEAR) 
       plot(plotSamples, adapP.sTime, "LINEAR DAMPING")   
       
       plotSamples = parameterarray(parametersArray,:BUOYANCY) 
       plot(plotSamples, adapP.sTime, "BUOYANCY") 
       
       plotSamples = dofarray(velocitySamplesArray,:SURGE) #[:SURGE, :SWAY, :HEAVE, :ROLL, :PITCH, :YAW]
       plot(plotSamples, adapP.sTime, "Velocity") 
       
       plot(deltaVSamplesArray, adapP.sTime, "delta velocity" )   
       
       plot(tauArray, adapP.sTime, "Forces & Torques" )            
       
       Vizkit.exec
        

end
