function sysCall_init() 
    closingJointHandles={-1,-1}
    closingJointHandles[1]=sim.getObjectHandle('Barrett_openCloseJoint#0')
    closingJointHandles[2]=sim.getObjectHandle('Barrett_openCloseJoint0#0')
    attachHandle = sim.getObjectHandle('BarrettHand_attachPoint#0')
    --cylinderHandle = sim.getObjectHandle('Cylinder0#')
    rotJointHandles={-1,-1}
    rotJointHandles[1]=sim.getObjectHandle('BarrettHand_jointA_0#0')
    rotJointHandles[2]=sim.getObjectHandle('BarrettHand_jointA_2#0')
    modelHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    val = sim.setStringSignal('close_signal', '')
end

function sysCall_cleanup() 
end 

function sysCall_actuation()
    sim.setJointTargetPosition(rotJointHandles[1],-math.pi*0.5)
    sim.setJointTargetPosition(rotJointHandles[2],-math.pi*0.5)
    closing_string=sim.getStringSignal('close_signal')
    if closing_string == 'yes' then
        sim.setJointTargetVelocity(closingJointHandles[1],-0.04)
        sim.setJointTargetVelocity(closingJointHandles[2],-0.04)
    else
        sim.setJointTargetVelocity(closingJointHandles[1],0.04)
        sim.setJointTargetVelocity(closingJointHandles[2],0.04)
    end
end 

