#***********************************************************************/
# Ruby's code for adaptive parameters identification of auv            */
#                                                                      */
#       Input thruster come from sinus function and input velocity     */
#       come from model                                                */
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
require './adap_properties.rb'
require './model_properties.rb'

include Orocos

Orocos.initialize

Orocos.run 'adap_parameters_estimator::Task' => 'adap_parameters',
           'motion_model::Task' => 'motion_model'   do
             
       
	## Get the specific task context ##
	adapP = TaskContext.get 'adap_parameters'
        model = TaskContext.get 'motion_model'

	
	
	
	
	thrusterInput = Types::Base::Samples::Joints.new 
	thrusterInput2 = Types::Base::Samples::Joints.new 
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
		thrusterInput2.elements << jointStates
		
		thrusterInput.elements[i].effort = 0	# Gets the array value for the new jointState.effort element
		thrusterInput2.elements[i].effort = 0
	end

                

	
       

	##########################################################################
	#		                    COMPONENT INPUT PORTS
	##########################################################################

  
	## Writers
	thrusterSampleWriter_adap = adapP.thruster_samples.writer
	thrusterSampleWriter_model = model.thruster_samples.writer
	
	

	# Samples
	thrusterSample = thrusterSampleWriter_adap.new_sample
	#thrusterSample_model = thrusterSampleWriter_model.new_sample
	

	# Loading the values into the sample variables
	thrusterSample = thrusterInput
	
		
	# Connecting the ports
	model.velocity.connect_to adapP.speed_samples
	
	

		
	# Configuring and starting the component
  adapP.configure
  model.configure
  adapP.start
  model.start
  
	# Writing the sample variables on the input ports
	thrusterSampleWriter_adap.write(thrusterSample)
	thrusterSampleWriter_model.write(thrusterSample)
	
	

	##########################################################################
	#		                    COMPONENT OUTPUT PORT
	##########################################################################
        
               
	parametersReader = adapP.parameters.reader
	#velocity = model.velocity.reader
	
        k = 0
        n = 0
	
	until k > 120 do
#=begin
			
	        if parametersSample = parametersReader.read_new
                        puts "\n\nparameters: \n"
	                puts "\n", parametersSample.inertiaCoeff[n].positive
	                puts "\n", parametersSample.quadraticDampingCoeff[n].positive
	                puts "\n", parametersSample.linearDampingCoeff[n].positive
	                puts "\n", parametersSample.gravityAndBuoyancy[n]
	                #puts "\n", parametersSample.coriolisCentripetalMatrix
	                
	                	                               	                
	        end	
		
	
	        thrusterInput.elements[0].effort = 2*(50*Math.sin(k*adapP.ftau) + 50)
	       # thrusterInput.elements[1].effort = 50*Math.sin(k*adapP.ftau) + 50
	       # thrusterInput.elements[2].effort = 50*Math.sin(k*adapP.ftau) + 50
	       # thrusterInput.elements[5].effort = 15*Math.sin(k*adapP.ftau) + 15
	        
	
	        # Loading the values into the sample variables
	        thrusterSample = thrusterInput
	       
	
		# Writing the sample variables on the input ports
	        thrusterSampleWriter_adap.write(thrusterSample)
	        thrusterSampleWriter_model.write(thrusterSample)
		
	        k = k + adapP.sTime	
#=end
	end
    

end
