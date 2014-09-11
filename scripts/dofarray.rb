def dofarray (samples, dof)
	array = Array.new	
	case dof
	when :SURGE
		samples.each do |x|
			array << x[0]
		end	

	when :SWAY
		samples.each do |x|
			array << x[1]
		end	

	when :HEAVE
		samples.each do |x|
			array << x[2]
		end	

	when :ROLL
		samples.each do |x|
			array << x[3]
		end	

	when :PITCH
		samples.each do |x|
			array << x[4]
		end

	when :YAW
		samples.each do |x|
			array << x[5]
		end
	else
		puts "The provided dof (#{dof.to_s}) is invalid."	
	end
	return array
end
