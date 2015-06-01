#! /usr/bin/env ruby

# 

def model_properties(task)

        inertiaMatrix = Eigen::MatrixX.new(6,6)
        linearDampingMatrix = Eigen::MatrixX.new(6,6)
        quadraticDampingMatrix = Eigen::MatrixX.new(6,6)
        coriolisCentripetalMatrix = Eigen::MatrixX.new(6,6)
        thrusterMatrix = Eigen::MatrixX.new(6,6)
        gravityBuoyancyVector = Eigen::VectorX.new(6)
       
        modelParamSample = task.model_parameters
        
#        modelParamSample = modelParamSampleWriter.new_sample
	
	
        integrationStep = 0.01
	
	inertia = [ 158,0,0,0,0,0,  0,168,0,0,0,0,  0,0,178,0,0,0,  0,0,0,1,0,0,  0,0,0,0,1,0,  0,0,0,0,0,10.5]
	linearDamping = [ 16,0,0,0,0,0,  0,15.23,0,0,0,0,  0,0,15.4,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,2.03]
	quadraticDamping = [ 30.715,0,0,0,0,0,  0,58.5,0,0,0,0,  0,0,47.4,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,5.518]
	coriolisCentripetal = [ 0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0]
	thuster = [ 1,1,0,0,0,  0,0,1,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  -0.21,0.21,0,0,0]
	gravityBuoyancy = [ 0,0,0,0,0,0]
	
	inertiaMatrix.from_a(inertia, 6, inertia.size/6, false)
	linearDampingMatrix.from_a(linearDamping, 6, linearDamping.size/6, false)
	quadraticDampingMatrix.from_a(quadraticDamping, 6, quadraticDamping.size/6, false)
	coriolisCentripetalMatrix.from_a(coriolisCentripetal, 6, coriolisCentripetal.size/6, false)
	thrusterMatrix.from_a(thuster, 6, thuster.size/6, false)
	gravityBuoyancyVector.from_a(gravityBuoyancy)
	
        
        
        
        #task.inertiaMatrix = inertiaMatrix
	#task.linearDampingMatrix = linearDampingMatrix
	#task.quadraticDampingMatrix = quadraticDampingMatrix
	#task.coriolisCentripetalMatrix = coriolisCentripetalMatrix
	#task.thrusterMatrix = thrusterMatrix
        #task.gravityBuoyancyVector = gravityBuoyancyVector
	#task.integrationStep = integrationStep
	
	modelParamSample.inertiaMatrix = inertiaMatrix
	modelParamSample.linearDampingMatrix = linearDampingMatrix
	modelParamSample.quadraticDampingMatrix = quadraticDampingMatrix
	modelParamSample.coriolisCentripetalMatrix = coriolisCentripetalMatrix
	modelParamSample.thrusterMatrix = thrusterMatrix
        modelParamSample.gravityBuoyancyVector = gravityBuoyancyVector
	modelParamSample.integrationStep = integrationStep
	
	task.model_parameters = modelParamSample
end
	
