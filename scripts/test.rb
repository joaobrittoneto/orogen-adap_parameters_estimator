#***********************************************************************/
# Ruby's code for adaptive parameters identification of auv            */
#                                                                      */
#   inputs (thruster and speed) come from sinus functions              */
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
include Orocos

Orocos.initialize

Orocos.run 'adap_parameters_estimator::Task' => 'adap_parameters' do
             

	## Get the specific task context ##
	adapP = TaskContext.get 'adap_parameters'


	
	
	
	speedInput = Types::Base::Samples::RigidBodyState.new
	thrusterInput = Types::Base::Samples::Joints.new 
	jointStates = Types::Base::JointState.new
	gainA = Eigen::VectorX.new(6)
	gainLambda = Eigen::MatrixX.new(6,4)
	matrixTrhuster = Eigen::MatrixX.new(6,5)
		
  ##########################################################################
  #                             CONFIG DATA
  ##########################################################################

	
	sampTime = 0.01
	frequencyTau = 1
	gainLambdaMatrix = [ 0.0005, 0.5, 0.5, 0.0005, 0.0001, 1, 1, 0, 1, 1, 1,1, 1, 1, 1, 1, 1, 1, 1, 1, 0.02, 0.5, 0.5, 0.0005]
	gainAVector = [-0.1, -0.0001, -1, -1, -1, -0.001]
	mTrhuster = [1,1,0,0,0,  0,0,1,0,0, 0,0,0,1,1, 0,0,0,0,0,	0,0,0,0,0, -0.21,0.21,-0.3,0,0]
	dof = :YAW;

        #puts gainLambda
        #puts gainLambdaMatrix
        
        gainLambda.from_a(gainLambdaMatrix, 6, gainLambdaMatrix.size/6, false)
        gainA.from_a(gainAVector)
        matrixTrhuster.from_a(mTrhuster, 6,5,false)
                
        
	##########################################################################
	#	     SETTING INITIAL POSITION, VELOCITY AND LINEAR CONTROL VALUES
	##########################################################################


	#thrusterInput.efforts = Eigen::VectorX.new(0, 0, 0, 0, 0, 0)

	speedInput.velocity = Eigen::Vector3.new(0, 0, 0)
	speedInput.angular_velocity = Eigen::Vector3.new(0, 0, 0)

	for i in 0..5
		thrusterInput.elements << jointStates	# Creates a new jointState element
		thrusterInput.elements[i].effort = 0	# Gets the array value for the new jointState.effort element
	end

                

	##########################################################################
	#		                    COMPONENT  PROPERTIES
	##########################################################################

	# The method "from_a" uses an array to set the matrix (Eigen::MatrixX).
	# Arguments:
	# 1 - Array
	# 2 - Matrix number of rows
	# 3 - Matrix number of columns
	# 4 - Matrix assignment orientation (true = column orientation, false = row orientation
	

	
	adapP.dofs = dof
	adapP.sTime = sampTime
	adapP.ftau = frequencyTau
        
	adapP.gLambda = gainLambda
	adapP.gA = gainA
	adapP.thrusterMatrix = matrixTrhuster

       puts adapP.thrusterMatrix 

	##########################################################################
	#		                    COMPONENT INPUT PORTS
	##########################################################################

  
	## Writers
	thrusterSampleWriter = adapP.thruster_samples.writer
	speedSampleWriter = adapP.speed_samples.writer
	

	# Samples
	thrusterSample = thrusterSampleWriter.new_sample
	speedSample = speedSampleWriter.new_sample
	

	# Loading the values into the sample variables
	thrusterSample = thrusterInput
	speedSample = speedInput
	
	#puts speedSample.velocity
        #puts speedSample.angular_velocity
        #printf "\n\n"
        #puts thrusterSample.elements[0].effort

	puts "trhuster"
	puts thrusterSample.elements.size
	
        
	# Configuring and starting the component
  adapP.configure
  adapP.start
 
	# Writing the sample variables on the input ports
	thrusterSampleWriter.write(thrusterSample)
	speedSampleWriter.write(speedSample)
	
	#puts adapP.speed_samples.velocity
        #puts adapP.speed_samples.angular_velocity
        #printf "\n\n"
        #puts adapP.thruster_samples.elements[0].effort

	##########################################################################
	#		                    COMPONENT OUTPUT PORT
	##########################################################################
        
               
	parametersReader = adapP.parameters.reader
	
	
        k = 0
        n = 5
	
	until k > 600 do
#=begin
					
		#if parametersSample = parametersReader.read_new
     # printf "\n\nParameters: \n\n"
		#  for i in 0...5
        #			printf "%f\n", parametersSample.inertiaCoeff[i].positive
		#	end
		#end
		
		
	#speedInput.velocity[0] = 0.519*Math.sin(k*frequencyTau - 1) + 1.53
	#speedInput.velocity[1] = 0.2505*Math.sin(k*frequencyTau - 1) + 0.785
	speedInput.angular_velocity[2] = 0.74*Math.sin(k*frequencyTau - 0.45) + 1.34
	
	#thrusterInput.elements[0].effort = 2*(50*Math.sin(k*frequencyTau) + 50)
	#thrusterInput.elements[1].effort = 50*Math.sin(k*frequencyTau) + 50
	thrusterInput.elements[5].effort = 15*Math.sin(k*frequencyTau) + 15
		
	# Loading the values into the sample variables
	thrusterSample = thrusterInput
	speedSample = speedInput
		
	#puts speedSample.velocity
        #puts speedSample.angular_velocity
        #printf "\n\n"
        #puts thrusterSample.elements[0].effort
		
	# Writing the sample variables on the input ports
	thrusterSampleWriter.write(thrusterSample)
	speedSampleWriter.write(speedSample)
	
	
	 if parametersSample = parametersReader.read_new
                        puts "\n\nparameters: \n"
	                puts "\n", parametersSample.inertiaCoeff[n].positive
	                puts "\n", parametersSample.quadraticDampingCoeff[n].positive
	                puts "\n", parametersSample.linearDampingCoeff[n].positive
	                puts "\n", parametersSample.gravityAndBuoyancy[n]
	      
	            
	  end       
	
	
	
	k = k + sampTime	
#=end
	end


	##########################################################################
	#			                     DISPLAYING DATA
	##########################################################################

=begin
	printf "\n\nThruster Matrix: \n\n"
    
	printf fblin.thruster_matrix.to_s[9..-3]

	printf "\n\n\nControllable DOFs: \n\n"
	fblin.controllable_dofs.each{|a| printf "%s ",a}

	printf "\n\n\nRobust compensation horizon: \n\n"
	printf "%i\n\n", fblin.robust_compensation_horizon


=end


end
