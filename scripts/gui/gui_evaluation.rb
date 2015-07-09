###########################################################################
##                                                                       ##
##                      SUPERVISORY SYSTEM CLASS                         ##
##                                                                       ##
###########################################################################


class Supervisory
  
# def initialize(model, parent = nil)
 def initialize(parent = nil)

   @window = Vizkit.load(File.join(File.dirname(__FILE__), 'Evaluation.ui'), parent)
    
 
  ##########################################################################
  #                     CONFIGURING PLOT2D OBJETCS                          #
  ##########################################################################

   @plotArray = [ @window.v,     @window.error_v,     @window.maev,    @window.normmaev]    
     

   for i in 0..(@plotArray.size - 1) 
        @plotArray[i].options[:auto_scrolling_y] = true    
        @plotArray[i].options[:time_window] = 50        
        @plotArray[i].getLegend.setVisible(false)          
        @plotArray[i].options[:update_time] = @stime
   end    

   @window.v.setTitle("Velocity")
   @window.error_v.setTitle("Error Velocity")
   @window.maev.setTitle("MAE velocity")
   @window.normmaev.setTitle("Normalized error")

   
   ##########################################################################
   #                    BUTTONS COMMANDS                                    #
   ##########################################################################
   # apply the configuration into the components
   @window.apply.connect(SIGNAL('clicked()')) do ||
        
        inertiaMatrix = Eigen::MatrixX.new(6,6)
        linearDampingMatrix = Eigen::MatrixX.new(6,6)
        quadraticDampingMatrix = Eigen::MatrixX.new(6,6)
        coriolisCentripetalMatrix = Eigen::MatrixX.new(6,6)
        thrusterMatrix = Eigen::MatrixX.new(6,6)
        gravityBuoyancyVector = Eigen::VectorX.new(6)
              
#        modelParamSample = model.model_parameters
          
        inertia = [@window.inertia_x.value,0,0,0,0,0,  0,@window.inertia_y.value,0,0,0,0,  0,0,@window.inertia_z.value,0,0,0,  0,0,0,@window.inertia_rx.value,0,0,  0,0,0,0,@window.inertia_ry.value,0,  0,0,0,0,0,@window.inertia_rz.value]
        quadraticDamping = [@window.quadratic_x.value,0,0,0,0,0,  0,@window.quadratic_y.value,0,0,0,0,  0,0,@window.quadratic_z.value,0,0,0,  0,0,0,@window.quadratic_rx.value,0,0,  0,0,0,0,@window.quadratic_ry.value,0,  0,0,0,0,0,@window.quadratic_rz.value]
        linearDamping = [@window.linear_x.value,0,0,0,0,0,  0,@window.linear_y.value,0,0,0,0,  0,0,@window.linear_z.value,0,0,0,  0,0,0,@window.linear_rx.value,0,0,  0,0,0,0,@window.linear_ry.value,0,  0,0,0,0,0,@window.linear_rz.value]
        coriolisCentripetal = [ 0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0]
        thuster = [ 1,1,0,0,0,  0,0,1,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0]
        gravityBuoyancy = [ @window.buoy_x.value, @window.buoy_y.value, @window.buoy_z.value, @window.buoy_rx.value, @window.buoy_ry.value, @window.buoy_rz.value]
	
	inertiaMatrix.from_a(inertia, 6, inertia.size/6, false)
        linearDampingMatrix.from_a(linearDamping, 6, linearDamping.size/6, false)
        quadraticDampingMatrix.from_a(quadraticDamping, 6, quadraticDamping.size/6, false)
        coriolisCentripetalMatrix.from_a(coriolisCentripetal, 6, coriolisCentripetal.size/6, false)
        thrusterMatrix.from_a(thuster, 6, thuster.size/6, false)
        gravityBuoyancyVector.from_a(gravityBuoyancy)
        
#       modelParamSample.inertiaMatrix = inertiaMatrix
#	modelParamSample.linearDampingMatrix = linearDampingMatrix
#	modelParamSample.quadraticDampingMatrix = quadraticDampingMatrix
#	modelParamSample.coriolisCentripetalMatrix = coriolisCentripetalMatrix
#	modelParamSample.thrusterMatrix = thrusterMatrix
#       modelParamSample.gravityBuoyancyVector = gravityBuoyancyVector
#	modelParamSample.integrationStep = integrationStep
	
#	model.model_parameters = modelParamSample
        
   end
 end


  def v_model (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.v.update(sample.velocity[0], "V_Model ")
        @window.v.set_y_axis_scale(-0.5, 1.0)
        @window.v.getLegend.setVisible(true)     
    end
  end  
  
  def v_measured (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.v.update(sample.velocity[0], "V_Measured")
        @window.v.set_y_axis_scale(-0.5, 1.0) 
        @window.v.getLegend.setVisible(true)                           
    end
  end 
  
  def error_v (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.error_v.update(sample, "Error velocity (m/s or rad/s)")
        @window.error_v.set_y_axis_scale(-0.5, 0.5)                       
    end
  end 
  
  def mae_v (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.maev.update(sample, "MAE velocity")
        @window.maev.set_y_axis_scale(-0.5, 0.5) 
        @window.mae_v.display sample                     
    end
  end 
  
  def norm_mae_v (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.normmaev.update(sample, "Norm MAE velocity")
        @window.normmaev.set_y_axis_scale(-0.5, 0.5) 
        @window.norm_mae_v.display sample                     
    end
  end 
  
  
   
    
  def evaluation (vmodel, vmeasured, error, mae, normmae)
        
        v_model (vmodel)
        v_measured (vmeasured)
        error_v (error)
        mae_v (mae)
        norm_mae_v (normmae)
        
  end      
        
 

  def show
    @window.show
  end

  
end
