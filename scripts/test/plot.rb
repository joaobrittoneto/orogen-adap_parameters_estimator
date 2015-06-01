require 'vizkit'

def plot(samples, sampleTime, name)

	time = Array.new

	plot = Vizkit.default_loader.Plot2d

	plot.options[:auto_scrolling] = false
	plot.options[:update_period] = sampleTime

	#if samples[0] != 0
	#	nullArray = Array.new
        #	nullArray << 0
	#	samples = nullArray + samples
	#end


	# Generating time array
	for i in 0...samples.size
		time << i*sampleTime
	end

	plot.set_x_axis_scale(time.min, time.max)
        #plot.set_y_axis_scale(samples.min/2,samples.min*2)
        plot.set_y_axis_scale(samples.min,samples.max)
	plot.setXTitle("Time")


	for i in 0...samples.size
		plot.update_custom(name, time[i], samples[i])
	end

	plot.show
       
#for i in 0..68
#	puts "time: #{time[i]}  parameter:  #{samples[i]}"
#end

	


end
