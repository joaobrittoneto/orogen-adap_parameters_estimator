#! /usr/bin/env ruby

# gains of the adaptive method.
# gainA: gaing of adatpive estimator
# gainLambda: gain of the update law

def adap_properties(task)

        gainA = Eigen::VectorX.new(6)
	gainLambda = Eigen::MatrixX.new(6,4)
	
	sampTime = 0.01
	frequencyTau = 1
	gainLambdaMatrix = [ 0.0005,0.5,0.5,0.0005,  0.0001,1,1,0,  0.0001,1,1,0,  1,1,1,1,  1,1,1,1,  0.02,0.5,0.5,0.0005] #0.02,0.5,0.5,0.0005
	gainAVector = [-0.1, -0.0001, -0.001, -1, -1, -0.001]
	dof = :SURGE; #[:SURGE, :SWAY, :HEAVE, :ROLL, :PITCH, :YAW]
	
	gainLambda.from_a(gainLambdaMatrix, 6, gainLambdaMatrix.size/6, false)
        gainA.from_a(gainAVector)
        
        
        task.dofs = dof
	task.sTime = sampTime
	task.ftau = frequencyTau
        
	task.gLambda = gainLambda
	task.gA = gainA
end
	
