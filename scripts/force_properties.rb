#! /usr/bin/env ruby

# gains of the adaptive method.
# gainA: gaing of adatpive estimator
# gainLambda: gain of the update law

def force_properties(task)

        frequency = Eigen::VectorX.new(6)
        amplitude = Eigen::VectorX.new(6)
        offset    = Eigen::VectorX.new(6)
	
	
	sampTime = 0.01
	
	
	
	
	vectorFrequency = [0, 0, 0, 0, 0, 0]
      
        vectorAmplitude = [0, 0, 0, 0, 0, 0]
         
        vecotrOffset = [0, 0, 0, 0, 0, 0]
	
	
	
	frequency.from_a(vectorFrequency)
        amplitude.from_a(vectorAmplitude)
        offset.from_a(vecotrOffset)
        
                       
       task.frequency = frequency
       task.amplitude = amplitude
       task.offset = offset
        
       
	
end
	
