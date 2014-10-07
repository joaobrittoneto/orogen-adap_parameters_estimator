#! /usr/bin/env ruby

# gains of the adaptive method.
# gainA: gaing of adatpive estimator
# gainLambda: gain of the update law

def adap_properties(task)

        gainA = Eigen::VectorX.new(6)
	gainLambda = Eigen::MatrixX.new(6,4)
	thrusterMatrix = Eigen::MatrixX.new
	
	
	sampTime = 0.01
	frequencyTau = 1
	
	dof = :YAW; #[:SURGE, :SWAY, :HEAVE, :ROLL, :PITCH, :YAW] default in orogen :UNINITIALISED 
	
	gainLambdaMatrix = [0.0005,0.5,0.5,0.0005,  0.0001,1,1,0,  0.0001,1,1,0,  1,1,1,1,  1,1,1,1,  0.02,0.5,0.5,0.0005] #0.02,0.5,0.5,0.0005
	
	gainAVector = [-0.1, -0.0001, -0.001, -1, -1, -0.001]
		
	tMatrix = [1,1,0,0,0,  0,0,1,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  -0.21,0.21,0,0,0]
	
	
	
	gainLambda.from_a(gainLambdaMatrix, 6, gainLambdaMatrix.size/6, false)
        gainA.from_a(gainAVector)
        thrusterMatrix.from_a(tMatrix, 6, tMatrix.size/6, false)
        
                       
        task.gLambda = gainLambda
	task.gA = gainA
        task.thrusterMatrix = thrusterMatrix
        task.dofs = dof
	task.sTime = sampTime
	task.ftau = frequencyTau
        
       
	
end
	
