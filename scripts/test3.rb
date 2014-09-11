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
require './adap_properties.rb'
require './model_properties.rb'

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
        adap_properties(configure_adap)
        adapP = configure_adap
        
        configure_model = model
        model_properties(configure_model)
        model = configure_model
	
       
        
	##########################################################################
	#	     SETTING INITIAL POSITION, VELOCITY AND LINEAR CONTROL VALUES
	##########################################################################


	for i in 0..5
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
        
               
		
	k = 0
	puts "test"
	 
	 ## doesn't work.
	adapP.parameters.connect_to do |sample, _|
	
	        puts "test2"
		widget.update_custom "inertia Coeff (SURGE)",k, sample.inertiaCoeff[0].positive.to_f 
		puts "test3"
	
	        thrusterInput.elements[0].effort = 2*(50*Math.sin(k*adapP.ftau) + 50)
        	#thrusterInput.elements[1].effort = 50*Math.sin(k*adapP.ftau) + 50
        	#thrusterInput.elements[5].effort = 15*Math.sin(k*adapP.ftau) + 15
		
	        # Loading the values into the sample variables
	        thrusterSample = thrusterInput
		
	        # Writing the sample variables on the input ports
	        thrusterSampleWriter_adap.write(thrusterSample)
	        thrusterSampleWriter_model.write(thrusterSample)
	
	        k += adapP.sTime	

        end
        puts k
        widget.show
        Vizkit.exec

end
