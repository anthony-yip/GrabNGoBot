createBaseDummies=function()
    --Creates dummies at the platform and the basket.
    local platform_handle=sim.getObjectHandle("Platform")
    local basket_handle=sim.getObjectHandle("Basket")
    local arm_handle=sim.getObjectHandle("LBR_iiwa_7_R800")
    local dummy_ori1=sim.getObjectOrientation(arm_handle,-1)[3]
    local dummy_ori2=dummy_ori1 - 3*math.pi/4
    local baseDummy1=sim.createDummy(0.05)
    sim.setObjectName(baseDummy1, "baseDummy1")
    sim.setObjectParent(baseDummy1,platform_handle,true)
    sim.setObjectPosition(baseDummy1,sim.handle_parent,{0.05,0.6,-1.2})
    sim.setObjectOrientation(baseDummy1,-1,{0,0,dummy_ori1})
    local baseDummy2=sim.createDummy(0.05)
    sim.setObjectName(baseDummy2, "baseDummy2")
    sim.setObjectParent(baseDummy2,basket_handle,true)
    sim.setObjectPosition(baseDummy2, sim.handle_parent, {0,1.025,-0.09269})
    sim.setObjectOrientation(baseDummy2,-1,{0,0,dummy_ori2})
    return baseDummy1,baseDummy2
end

findShortestAngle=function(angle1,angle2,angle3,angle4)
    --Finds the shortest absolute angle.
    local ang=angle1
    if math.abs(angle2) < math.abs(ang) then
        ang=angle2
    elseif math.abs(angle3) < math.abs(ang) then
        ang=angle3
    elseif math.abs(angle4) < math.abs(ang) then
        ang=angle4
    end
    return ang
end

getGammas=function(x_change,y_change)
    --Finds orientations such that the Omnirob can move orthogonally.
    if x_change/y_change > 0 then
        local tangent=math.abs(y_change/x_change)
        gamma1=math.atan(tangent)
        gamma2=gamma1 + math.pi/2
        gamma3=gamma1 - math.pi/2
        gamma4=gamma1 - math.pi
    else 
        local tangent=math.abs(y_change/x_change)
        gamma1=math.pi/2 - math.atan(tangent)
        gamma2=gamma1 + math.pi/2
        gamma3=gamma1 - math.pi/2
        gamma4=gamma1 - math.pi
    end
    return {gamma1,gamma2,gamma3,gamma4}
end

getArgument=function(x_change,y_change)
    --Describes intended direction of travel.
    local tangent=math.abs(y_change/x_change)
    if x_change>=0 and y_change>=0 then
        arg=math.atan(tangent) + 3*math.pi/2
    elseif x_change>=0 and y_change<0 then
        arg=3*math.pi/2 - math.atan(tangent)
    elseif x_change<0 and y_change>=0 then
        arg=math.pi/2 - math.atan(tangent)
    elseif x_change<0 and y_change<0 then
        arg=math.atan(tangent) + math.pi/2
    end
    return arg
end

brake=function()
    sim.setJointTargetVelocity(h[1],0)
    sim.setJointTargetVelocity(h[2],0)
    sim.setJointTargetVelocity(h[3],0)
    sim.setJointTargetVelocity(h[4],0)
end

forward=function()
    sim.setJointTargetVelocity(h[1],v1)
    sim.setJointTargetVelocity(h[2],-v1)
    sim.setJointTargetVelocity(h[3],-v1)
    sim.setJointTargetVelocity(h[4],v1)
end

backward=function()
    sim.setJointTargetVelocity(h[1],-v1)
    sim.setJointTargetVelocity(h[2],v1)
    sim.setJointTargetVelocity(h[3],v1)
    sim.setJointTargetVelocity(h[4],-v1)
end

left=function()
    sim.setJointTargetVelocity(h[1],-v1)
    sim.setJointTargetVelocity(h[2],-v1)
    sim.setJointTargetVelocity(h[3],v1)
    sim.setJointTargetVelocity(h[4],v1)
end

right=function()
    sim.setJointTargetVelocity(h[1],v1)
    sim.setJointTargetVelocity(h[2],v1)
    sim.setJointTargetVelocity(h[3],-v1)
    sim.setJointTargetVelocity(h[4],-v1)
end

left_right=function(dist)
    local t=math.abs(dist)/0.09
    if dist>=0 then
        right()
    else 
        left()
    end
    sim.wait(t)
    brake()
end

front_back=function(dist)
    local t=math.abs(dist)/0.09
    if dist>=0 then
        forward()
    else
        backward()
    end
    sim.wait(t)
    brake()
end

clockwise=function()
    sim.setJointTargetVelocity(h[1],v2)
    sim.setJointTargetVelocity(h[2],v2)
    sim.setJointTargetVelocity(h[3],v2)
    sim.setJointTargetVelocity(h[4],v2)
end

anti_clockwise=function()
    sim.setJointTargetVelocity(h[1],-v2)
    sim.setJointTargetVelocity(h[2],-v2)
    sim.setJointTargetVelocity(h[3],-v2)
    sim.setJointTargetVelocity(h[4],-v2)
end

rotate=function(angle)
    local t=math.abs(angle)/(math.pi/8.7)
    if angle>=0 then
        clockwise()
    else
        anti_clockwise()
    end
    sim.wait(t)
    brake()
end

rotateRobot=function(endPos)
    --Rotate the Omnirob to an orientation found in getGammas()
    --such that the angle rotated is minimum. 
    local startPos=sim.getObjectPosition(Omni,-1)
    local x_change=endPos[1]-startPos[1]
    local y_change=endPos[2]-startPos[2]
    local gammas=getGammas(x_change,y_change)
    local startGamma=sim.getObjectOrientation(Omni,-1)[3]
    local rotations = {0,0,0,0}
    if startGamma<0 then
        startGamma = 2*math.pi + startGamma
    end
    for i=1,4,1 do
        if gammas[i]<0 then
            gammas[i] = 2*math.pi + gammas[i]
        end
        rot = gammas[i] - startGamma
        if rot > math.pi then
            rot = rot - 2*math.pi
        elseif rot < -math.pi then
            rot = rot + 2*math.pi
        end
        rotations[i] = rot
    end
    local final_rotation=findShortestAngle
    (rotations[1],rotations[2],rotations[3],rotations[4])
    rotate(-final_rotation)
end

moveRobot=function(endPos)
    --Moves the robot linearly to the endPos.
    local startPos=sim.getObjectPosition(Omni,-1)
    local x_change=endPos[1]-startPos[1]
    local y_change=endPos[2]-startPos[2]
    local dist=math.sqrt(x_change^2 + y_change^2)
    local end_gamma=getArgument(x_change,y_change)
    local current_gamma=sim.getObjectOrientation(Omni,-1)[3]
    if current_gamma < 0 then
        current_gamma = current_gamma + 2*math.pi
    end
    local difference=end_gamma-current_gamma
    if difference < 0 then
        difference = difference + 2* math.pi
    end
    if difference < math.pi/4 or difference > 7*math.pi/4 then
        front_back(-dist)
    elseif difference > math.pi/4 and difference < 3*math.pi/4 then
        left_right(dist)
    elseif difference > 3*math.pi/4 and difference < 5*math.pi/4 then
        front_back(dist)
    else
        left_right(-dist)
    end
end

final_rotate=function(input)
    --Rotates the Omnirob at the final base dummies.
    local ori1=sim.getObjectOrientation(Omni,-1)[3] 
    local ori2=sim.getObjectOrientation(targetHandles[input],-1)[3]
    if ori1 < 0 then
        ori1 = ori1 + 2*math.pi
    end
    if ori2 < 0 then
        ori2 = ori2 + 2*math.pi
    end
    rotate_amt1=ori2 - ori1
    if rotate_amt1 < 0 then
        rotate_amt2=rotate_amt1+2*math.pi
    else
        rotate_amt2=rotate_amt1-2*math.pi
    end
    if math.abs(rotate_amt1) < math.abs(rotate_amt2) then
        rotate(-rotate_amt1)
    else
        rotate(-rotate_amt2)
    end
end

findBasePathLength=function(path)
    --Finds the length of a path.
    local l=0
    for i=1,#path/3-1,1 do
        x_change = path[3*i+1] - path[3*i-2]
        y_change = path[3*i+2] - path[3*i-1]
        dist=math.sqrt(x_change^2 + y_change^2)
        l=l+dist
    end
    return l
end

computePath=function(basePathCnt,baseMaxTime,basePathStateCnt)
    local _path=nil
    local _l=999999999999999999999999
    for i=1,basePathCnt,1 do
        local r,path=simOMPL.compute(t,baseMaxTime,-1,basePathStateCnt)
        if r and path then
            l=findBasePathLength(path)
            if l<_l then
                _l=l
                _path=path
            end
        end
    end
    if not _path then
        print("failure")
        sim.stopSimulation()
    end
    return _path
end
followPath=function(path)
    --Follows a path generated by computePath().
    for i=1,#path/3-1,1 do
        endPos={path[3*i+1],path[3*i+2]}        
        rotateRobot(endPos)
        moveRobot(endPos)
    end
end

function sysCall_threadmain()
    --Initialisation phase: 
    v1=42.375*math.pi/180
    --Translational motion, speed of 0.09 ms-1.
    v2=100*math.pi/180
    --Angular motion, full rotation in 17.4s .
    sim.setStringSignal("base_lock", "")
    Omni=sim.getObjectHandle('Omnirob')
    jRL=sim.getObjectHandle('Omnirob_RLwheel_motor')
    jRR=sim.getObjectHandle('Omnirob_RRwheel_motor')
    jFL=sim.getObjectHandle('Omnirob_FLwheel_motor')
    jFR=sim.getObjectHandle('Omnirob_FRwheel_motor')
    h={jFL,jFR,jRR,jRL}    
    targetHandle1,targetHandle2=createBaseDummies()
    targetHandles={targetHandle1,targetHandle2}
    initial_pos=sim.getObjectPosition(Omni, -1)
    goalpos1={sim.getObjectPosition(targetHandle1,-1)[1],
    sim.getObjectPosition(targetHandle1,-1)[2]}
    goalpos2={sim.getObjectPosition(targetHandle2,-1)[1],
    sim.getObjectPosition(targetHandle2,-1)[2]}
    goalpositions={goalpos1,goalpos2}
    robot_collection = sim.getCollectionHandle('robot')
    for j=1,2,1 do
        for i=1,2,1 do
            --Sets up path planning task.
            t=simOMPL.createTask('t')
            ss={simOMPL.createStateSpace('2d',
            simOMPL.StateSpaceType.pose2d,Omni,{-2,-2},{2,2},1)}
            simOMPL.setStateSpace(t,ss)
            simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
            obs_collection=sim.getCollectionHandle("obstacles")
            startpos={sim.getObjectPosition(Omni,-1)[1],
            sim.getObjectPosition(Omni,-1)[2]}
            simOMPL.setStartState(t,startpos)
            simOMPL.setGoalState(t,goalpositions[i])
            simOMPL.setCollisionPairs(t,{robot_collection,obs_collection})
            --Computes and follows a path to a base dummy.
            _path=computePath(5,60,0)
            followPath(_path)
            final_rotate(i)
            --Locks the Omnirob and allows the arm to move.
            sim.setStringSignal("base_lock", "yes")
            repeat
                sig=sim.getStringSignal("base_lock")
            until sig==""
        end
    end
    --Moves back to the initial position.
    t=simOMPL.createTask('t')
    ss={simOMPL.createStateSpace('2d',
    simOMPL.StateSpaceType.pose2d,Omni,{-2,-2},{2,2},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    obs_collection=sim.getCollectionHandle("obstacles")    startpos={sim.getObjectPosition(Omni,-1)[1],sim.getObjectPosition(Omni,-1)[2]}
    goalpos={initial_pos[1],initial_pos[2]}
    simOMPL.setStartState(t,startpos)
    simOMPL.setGoalState(t,goalpos)
    simOMPL.setCollisionPairs(t,{robot_collection,obs_collection})
    _path=computePath(5,40,0)
    followPath(_path)
    current_orientation=sim.getObjectOrientation(Omni,-1)[3]
    rotate(current_orientation)
end

function sysCall_cleanup()
    -- Put some clean-up code here
end
