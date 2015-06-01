###########################################################################
##                                                                       ##
##                      SUPERVISORY SYSTEM CLASS                         ##
##                                                                       ##
###########################################################################


class Supervisory
  
 def initialize(adap, parent = nil)

    @window = Vizkit.load(File.join(File.dirname(__FILE__), 'Parameters.ui'), parent)
    
    @dofs = adap.dofs
    @freq = adap.ftau
    @stime = adap.sTime
    @min_norm_error = 10
    @old_min_norm_error = 10
    
        
    ##############
    # used in the deltaV and normDeltaV dor adjust the y_scale
    @int1 = 0
    @int2 = 0
    if @freq != 0
        @n = ((2*Math::PI/@freq) / @stime)
    else
        @n = 30  
    end 
    
    @auxiliarArray1 = Array.new
    @auxiliarArray2 = Array.new
    ##############
    
    
   
   if @dofs == :SURGE
        @dof = 0
   elsif  @dofs == :SWAY
        @dof = 1
   elsif  @dofs == :HEAVE
        @dof = 2
   elsif  @dofs == :ROLL
        @dof = 3                 
   elsif  @dofs == :PITCH
        @dof = 4
   elsif  @dofs == :YAW
        @dof = 5
   end             

  ##########################################################################
  #                     CONFIGURING PLOT2D OBJETCS                          #
  ##########################################################################

    @plotArray = [ @window.deltaV,     @window.inertia,     @window.linearD,    @window.quadraticD,     
                   @window.buoyancy, @window.lin_vel_x,   @window.lin_vel_y,  @window.lin_vel_z, 
                   @window.rot_vel_x,  @window.rot_vel_y,   @window.rot_vel_z,  @window.lin_for_x, 
                   @window.lin_for_y,  @window.lin_for_z,   @window.rot_for_x,  @window.rot_for_y,
                   @window.rot_for_z,  @window.deltaV_normalized,          ]    
     

   for i in 0..(@plotArray.size - 1) 
        @plotArray[i].options[:auto_scrolling_y] = true    
        @plotArray[i].options[:time_window] = 50        
        @plotArray[i].getLegend.setVisible(false)          
        @plotArray[i].options[:update_time] = @stime
   end    

   @window.deltaV.setTitle("Error Velocity")
   @window.deltaV_normalized.setTitle("Normalized Error Velocity")
   @window.inertia.setTitle("Inertia")
   @window.linearD.setTitle("Linear Damping")
   @window.quadraticD.setTitle("Quadratic Damping")
   @window.buoyancy.setTitle("Buoyancy")
   @window.lin_vel_x.setTitle("Surge velocity")
   @window.lin_vel_y.setTitle("Sway velocity")
   @window.lin_vel_z.setTitle("Heave velocity")
   @window.rot_vel_x.setTitle("Roll velocity")
   @window.rot_vel_y.setTitle("Pitch velocity")
   @window.rot_vel_z.setTitle("Yaw velocity")
   @window.lin_for_x.setTitle("Surge force")
   @window.lin_for_y.setTitle("Sway force")
   @window.lin_for_z.setTitle("Heave force")
   @window.rot_for_x.setTitle("Roll torque")
   @window.rot_for_y.setTitle("Pitch torque")
   @window.rot_for_z.setTitle("Yaw torque")
   
   
  ##########################################################################
  #                    BUTTONS COMMANDS                                    #
  ##########################################################################
   # stop button
   @window.stop.connect(SIGNAL('clicked()')) do ||
      if adap.state == :RUNNING
        adap.stop
      end
   end
    
   # start button
   @window.start.connect(SIGNAL('clicked()')) do ||
      if adap.state == :STOPPED
        adap.start
      end 
    end
   
   # select degree of freedom
   @window.dofs.connect(SIGNAL('currentIndexChanged(const QString &)')) do |value|      
      case value
      when "Surge"
        @dofs = :SURGE  
        @dof = 0        
      when "Sway"
        @dofs = :SWAY  
        @dof = 1        
      when "Heave"
        @dofs = :HEAVE
        @dof = 2          
      when "Roll"
        @dofs = :ROLL
        @dof = 3          
      when "Pitch"
        @dofs = :PITCH 
        @dof = 4         
      when "Yaw"
        @dofs = :YAW
        @dof = 5
      when  "Uninitialised"
        @dofs = :UNINITIALISED
        @dof = 6
      end
      puts @dofs
   end
   
   
   # apply the configuration into the components
   @window.apply.connect(SIGNAL('clicked()')) do ||
      gainA = Eigen::VectorX.new(6)
      gainLambda = Eigen::MatrixX.new(6,4)
      
      frequency = Eigen::VectorX.new(6)
      amplitude = Eigen::VectorX.new(6)
      offset = Eigen::VectorX.new(6)
      
      
      vectorGainA = [@window.gainAx.value, @window.gainAy.value,
                     @window.gainAz.value, @window.gainArx.value, 
                     @window.gainAry.value, @window.gainArz.value] 
                      
      matrixGainLambda = [@window.gainL1x.value, @window.gainL2x.value,
                          @window.gainL3x.value, @window.gainL4x.value, 
                          @window.gainL1y.value, @window.gainL2y.value, 
                          @window.gainL3y.value, @window.gainL4y.value,
                          @window.gainL1z.value, @window.gainL2z.value,
                          @window.gainL3z.value, @window.gainL4z.value,
                          @window.gainL1rx.value, @window.gainL2rx.value,
                          @window.gainL3rx.value, @window.gainL4rx.value,
                          @window.gainL1ry.value, @window.gainL2ry.value,
                          @window.gainL3ry.value, @window.gainL4ry.value,
                          @window.gainL1rz.value, @window.gainL2rz.value,
                          @window.gainL3rz.value, @window.gainL4rz.value]
                          
      vectorFrequency = [@window.frex.value, @window.frey.value, @window.frez.value,
                         @window.frerx.value, @window.frery.value, @window.frerz.value]
      
     # vectorAmplitude = [@window.ampx.value, @window.ampy.value, @window.ampz.value,
     #                    @window.amprx.value, @window.ampry.value, @window.amprz.value]
      
     # vecotrOffset = [@window.offx.value, @window.offy.value, @window.offz.value,
     #                 @window.offrx.value, @window.offry.value, @window.offrz.value]
                                                   
      
      gainA.from_a(vectorGainA)
      gainLambda.from_a(matrixGainLambda, 6, 4, false)
      
      frequency.from_a(vectorFrequency)
     # amplitude.from_a(vectorAmplitude)
     # offset.from_a(vecotrOffset)
      
      
      adap.gA = gainA
      adap.gLambda = gainLambda
      adap.dofs = @dofs
      if @dof < 6
        adap.ftau = frequency[@dof]
      else
        adap.ftau = 0
      end
            
      #forces.frequency = frequency
      #forces.amplitude = amplitude
      #forces.offset = offset
                      
      ##############
      # used in the deltaV and normDeltaV dor adjust the y_scale
      @int1 = 0
      @int2 = 0
      @freq = adap.ftau
      @stime = adap.sTime
      
      if @freq != 0
         @n = ((2*Math::PI/@freq) / @stime)
      else
         @n = 30  
      end 
    
      @auxiliarArray1.replace([])
      @auxiliarArray2.replace([])
      ##############
      
    end
    
 end

  def deltaV_evolution (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.deltaV.update(sample, "Error Velocity (m/s or rad/s)")
        ########################################################
        # Set y-axis scale by the average value of the absolute 
        # value of the error in the period
        ######################################################## 
        
        
        if @int1 < @n
                @auxiliarArray1 << sample.abs
                meanValue = @auxiliarArray1.reduce(:+).to_f / (@auxiliarArray1.length)
                @int1 += 1
        elsif @int1 >= @n and !@auxiliarArray1.empty?
                @auxiliarArray1.shift
                @auxiliarArray1 << sample.abs
                meanValue = @auxiliarArray1.reduce(:+).to_f / (@auxiliarArray1.length)
        end
         
       
        if meanValue >= 10
                @window.deltaV.set_y_axis_scale(-20, 20)
        elsif meanValue < 10 and meanValue >= 5
                @window.deltaV.set_y_axis_scale(-10, 10)
        elsif meanValue < 5 and meanValue >= 1
                @window.deltaV.set_y_axis_scale(-5, 5)
        elsif meanValue < 1 and meanValue >= 0.1
                @window.deltaV.set_y_axis_scale(-1, 1)
        elsif meanValue < 0.1 and meanValue >= 0.01
                @window.deltaV.set_y_axis_scale(-0.1, 0.1)
        elsif meanValue < 0.01 and meanValue >= 0.001
                @window.deltaV.set_y_axis_scale(-0.01, 0.01) 
        else                                                               
                @window.deltaV.set_y_axis_scale(-0.001, 0.001)
        end
                
    end
  end  
  
  
  def normDeltaV_evolution(port)
    @port = port
    @port.connect_to do |sample, _|                    
        @window.deltaV_normalized.update(sample, "mean(|Ve-Vm|) / mean(|Vm|)")
        ########################################################
        # Set y-axis scale by the average value of the absolute 
        # value of the error in the period
        ######################################################## 
          
        if @int2 < @n
                @auxiliarArray2 << sample.abs
                meanValue = @auxiliarArray2.reduce(:+).to_f / (@auxiliarArray2.length)
                @int2 += 1
        elsif @int2 >= @n and !@auxiliarArray2.empty?
                @auxiliarArray2.shift
                @auxiliarArray2 << sample.abs
                meanValue = @auxiliarArray2.reduce(:+).to_f / (@auxiliarArray2.length)
        end
                             
        if meanValue >= 100
                @window.deltaV_normalized.set_y_axis_scale(-1, 200)
        elsif meanValue < 100 and meanValue >= 50
                @window.deltaV_normalized.set_y_axis_scale(-11, 100)
        elsif meanValue < 50 and meanValue >= 10
                @window.deltaV_normalized.set_y_axis_scale(-1, 50)
        elsif meanValue < 10 and meanValue >= 1
                @window.deltaV_normalized.set_y_axis_scale(-1, 10)
        elsif meanValue < 1 and meanValue >= 0.1
                @window.deltaV_normalized.set_y_axis_scale(-0.5, 1)
        elsif meanValue < 0.1 and meanValue >= 0.01
                @window.deltaV_normalized.set_y_axis_scale(-0.05, 0.1)                
        elsif meanValue < 0.01 and meanValue >= 0.001
                @window.deltaV_normalized.set_y_axis_scale(-0.005, 0.01) 
        else                                                               
                @window.deltaV_normalized.set_y_axis_scale(-0.0005, 0.001)
        end
                
        if @min_norm_error > sample
                @min_norm_error = sample         
        end
        @window.deltaV_normalized.update(@min_norm_error, " ")        
                
    end
  end
  
  
  def velocity_evolution (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.lin_vel_x.update(sample.velocity[0], "Surge velocity (m/s)")
        @window.lin_vel_y.update(sample.velocity[1], "Sway velocity (m/s)")
        @window.lin_vel_z.update(sample.velocity[2], "Heave velocity (m/s)")
        @window.rot_vel_x.update(sample.angular_velocity[0], "Roll velocity (rad/s)")
        @window.rot_vel_y.update(sample.angular_velocity[1], "Pitch velocity (rad/s)")
        @window.rot_vel_z.update(sample.angular_velocity[2], "Yaw velocity (rad/s)")
    end
  end    
  
  def forces_evolution (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.lin_for_x.update(sample.elements[0].effort, "Surge force (N)")
        @window.lin_for_y.update(sample.elements[1].effort, "Sway force (N)")
        @window.lin_for_z.update(sample.elements[2].effort, "Heave force (N)")
        @window.rot_for_x.update(sample.elements[3].effort, "Roll torque (N*m)")
        @window.rot_for_y.update(sample.elements[4].effort, "Pitch torque (N*m)")
        @window.rot_for_z.update(sample.elements[5].effort, "Yaw torque (N*m)")
    end    
  end 
    
  def parameters_evolution (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.inertia.update(sample.inertiaCoeff[@dof].positive, "kg or Kg*m^2/rad" ) #
        @window.linearD.update(sample.linearDampingCoeff[@dof].positive, "Kg/s or Kg*m^2/(rad*s)" )
        @window.quadraticD.update(sample.quadraticDampingCoeff[@dof].positive, "Kg/m or Kg*m^2" ) #or Kg*rad
        @window.buoyancy.update(sample.gravityAndBuoyancy[@dof], "N or N*m") #or Kg*rad/s²
        
        @window.inertia.set_y_axis_scale(sample.inertiaCoeff[@dof].positive/1.5, sample.inertiaCoeff[@dof].positive*1.5) 
        @window.linearD.set_y_axis_scale(sample.linearDampingCoeff[@dof].positive/1.5, sample.linearDampingCoeff[@dof].positive*1.5) 
        @window.quadraticD.set_y_axis_scale(sample.quadraticDampingCoeff[@dof].positive/1.5, sample.quadraticDampingCoeff[@dof].positive*1.5) 
        @window.buoyancy.set_y_axis_scale(sample.gravityAndBuoyancy[@dof]/1.5, sample.gravityAndBuoyancy[@dof]*1.5) 
        
        if @old_min_norm_error > @min_norm_error
        
                @window.xIner.display sample.inertiaCoeff[0].positive
                @window.yIner.display sample.inertiaCoeff[1].positive
                @window.zIner.display sample.inertiaCoeff[2].positive
                @window.rxIner.display sample.inertiaCoeff[3].positive
                @window.ryIner.display sample.inertiaCoeff[4].positive
                @window.rzIner.display sample.inertiaCoeff[5].positive
               
                @window.xQuad.display sample.quadraticDampingCoeff[0].positive
                @window.yQuad.display sample.quadraticDampingCoeff[1].positive
                @window.zQuad.display sample.quadraticDampingCoeff[2].positive
                @window.rxQuad.display sample.quadraticDampingCoeff[3].positive
                @window.ryQuad.display sample.quadraticDampingCoeff[4].positive
                @window.rzQuad.display sample.quadraticDampingCoeff[5].positive
                
                @window.xLin.display sample.linearDampingCoeff[0].positive
                @window.yLin.display sample.linearDampingCoeff[1].positive
                @window.zLin.display sample.linearDampingCoeff[2].positive
                @window.rxLin.display sample.linearDampingCoeff[3].positive
                @window.ryLin.display sample.linearDampingCoeff[4].positive
                @window.rzLin.display sample.linearDampingCoeff[5].positive
                
                @window.xBuoy.display sample.gravityAndBuoyancy[0]
                @window.yBuoy.display sample.gravityAndBuoyancy[1]
                @window.zBuoy.display sample.gravityAndBuoyancy[2]
                @window.rxBuoy.display sample.gravityAndBuoyancy[3]
                @window.ryBuoy.display sample.gravityAndBuoyancy[4]
                @window.rzBuoy.display sample.gravityAndBuoyancy[5]
                
                @old_min_norm_error = @min_norm_error
         end
            
    end       
     
  end   
    
  def evolution (dV, ndV, vel, ft, para)
        puts @dof
        deltaV_evolution (dV)
        normDeltaV_evolution(ndV)
        velocity_evolution (vel)
        forces_evolution (ft)
        parameters_evolution (para)
  end      
        
 

  def show
    @window.show
  end

  
end
