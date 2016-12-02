require 'pocolog'
require 'eigen'
include Pocolog

address = ARGV[0].to_s

file = Logfiles.new File.open(address)
data_stream = file.stream("/aggregator.dynamic_sample")
puts "#{data_stream.size}"
states = Eigen::MatrixX.new(data_stream.size,4)
accel = Eigen::VectorX.new(data_stream.size)
i=0
dof = 0
data_stream.samples.each do |realtime, logical, sample|
    states[i,0] = sample.secondary_states.efforts.linear[dof]
    vel = sample.pose.velocity[0]
    states[i,1] = vel * vel.abs
    states[i,2] = vel
    states[i,3] = 1
    accel[i]  = sample.secondary_states.linear_acceleration.acceleration[dof]
    i = i+1
  # puts "velocity: #{sample.pose.velocity[0]}"
end

solver = states.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
param = solver.solve(accel)
puts "param #{param}"
parameters = Eigen::VectorX.from_a([1/param[0], -param[1]/param[0], -param[2]/param[0], -param[3]/param[0]])
puts "inertia:   #{parameters[0]}"
puts "quad_damp: #{parameters[1]}"
puts "lin_damp:  #{parameters[2]}"
puts "buoy:      #{parameters[3]}"
