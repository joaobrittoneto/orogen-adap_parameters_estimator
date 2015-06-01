def parameterarray (samples, parameter)
	array = Array.new	
	case parameter
	when :INERTIA
		samples.each do |x|
			array << x[0]
		end	

	when :QUADRATIC
		samples.each do |x|
			array << x[1]
		end	

	when :LINEAR
		samples.each do |x|
			array << x[2]
		end	

	when :BUOYANCY
		samples.each do |x|
			array << x[3]
		end	

	else
		puts "The provided parameter (#{parameter.to_s}) is invalid."	
	end
	return array
end
