local clock = os.clock

generateOrientations = function() 
--Using the orientation of the arm, generates orientations of for the   --dummies at the platform and basket.
    local arm_handle = sim.getObjectHandle('LBR_iiwa_7_R800')
    local dummy1 = sim.getObjectHandle('defaultDummy1')
    local dummy2 = sim.getObjectHandle('defaultDummy2')
    local arm_position = sim.getObjectPosition(arm_handle, -1)
    local arm_orientation = sim.getObjectOrientation(arm_handle, -1)
    local x_axis = {math.cos(arm_orientation[3]), 
    math.sin(arm_orientation[3]), 0}
    local z_axis = {0, 0, math.cos(arm_orientation[1])}
    local arm_matrix = sim.getObjectMatrix(arm_handle, -1)
    local matrix1 = sim.rotateAroundAxis(arm_matrix, x_axis,
    arm_position, math.pi/2)
    sim.setObjectMatrix(dummy1, -1, matrix1)
    local defaultOrientation1 = sim.getObjectOrientation(dummy1, -1)
    local matrix2 = sim.rotateAroundAxis(arm_matrix, z_axis,
    arm_position, -math.pi/2)
    local matrix3 = sim.rotateAroundAxis(matrix2, x_axis,
    arm_position, -math.pi * 4/3)
    sim.setObjectMatrix(dummy2, -1, matrix3)
    local defaultOrientation2 = sim.getObjectOrientation(dummy2, -1)
    return defaultOrientation1, defaultOrientation2
end

createPickObjects=function(table)
    --Table is a table of tables. Each table is used to describe one object.
    --e.g. table = {{primitiveType1, options1, sizes1, mass1, position1,
    --object_orientation1, dummy_orientation1}, 
    --{primitiveType2, options2, sizes2, mass2, position2,
    --object_orientation2, dummy_orientation2}}
    local obj_handles = {}
    local dummy_handles = {}
    local collection = sim.getCollectionHandle("obstacles")
    for i=1,#table,1 do
        local handle=sim.createPureShape(table[i][1], table[i][2],
        table[i][3], table[i][4])
        obj_handles[i] = handle
        sim.setObjectPosition(handle,-1,table[i][5])
        table[i][6] = table[i][6] or sim.getObjectOrientation(arm_handle,-1)
        sim.setObjectOrientation(handle,-1, table[i][6])
        sim.addObjectToCollection(collection, handle, sim.handle_single, 0)
        local dummyHandle = sim.createDummy(0.05)
        sim.setObjectParent(dummyHandle,handle,true)
        sim.setObjectPosition(dummyHandle,sim.handle_parent,{0,0.05,0.045})
        table[i][7] = table[i][7] or defaultOrientation1
        sim.setObjectOrientation(dummyHandle,-1,table[i][7])
        dummy_handles[i] = dummyHandle
    end
    return obj_handles, dummy_handles
end

createObstacles=function(table)
    --Table is a table of tables. Each table is used to describe one object.
    --e.g. table = {{primitiveType1, options1, sizes1, mass1,
    --position1, orientation1}, 
    --{primitiveType2, options, sizes, mass, position, orientation}}
    local collection = sim.getCollectionHandle("obstacles")
    for i=1,#table,1 do
        local handle=sim.createPureShape(table[i][1], table[i][2],
        table[i][3], table[i][4])
        sim.setObjectPosition(handle,-1,table[i][5])
        table[i][6] = table[i][6] or {0, 0, 0}
        sim.setObjectOrientation(handle,-1, table[i][6])
        sim.addObjectToCollection(collection, handle, sim.handle_single, 0)
    end
end

findDestinationObject=function(orientation)
    --Locates the basket model and creates a dummy for motion planning there.
    local obj_handle = sim.getObjectHandle("Basket")
    sim.setObjectOrientation(obj_handle,-1,
    sim.getObjectOrientation(arm_handle,-1))
    local pos=sim.getObjectPosition(obj_handle,-1)
    local dummyHandle = sim.createDummy(0.05)
    sim.setObjectParent(dummyHandle,obj_handle,true)
    sim.setObjectPosition(dummyHandle, sim.handle_parent, {0,0.4,1.2})
    local orientation = orientation or defaultOrientation2
    sim.setObjectOrientation(dummyHandle, -1, orientation)
    return dummyHandle
end
   
displayInfo=function(txt)
    if dlgHandle then
        sim.endDialog(dlgHandle)
    end
    dlgHandle=nil
    if txt and #txt>0 then
        dlgHandle=sim.displayDialog('info',txt,sim.dlgstyle_message,false)
        sim.switchThread()
    end
end

setJointToHoldPosition=function(jointHandle)
    sim.setThreadAutomaticSwitch(false) -- only for threaded child scripts
    sim.setObjectInt32Parameter(jointHandle,2001,1) -- enable the control loop
    sim.setJointTargetPosition(jointHandle,sim.getJointPosition(jointHandle))
    sim.setThreadAutomaticSwitch(true) -- only for threaded child scripts
end

setJointToVelocityControl=function(jointHandle)
    sim.setObjectInt32Parameter(jointHandle,2001,0) -- disable control loop
end

getMatrixShiftedAlongZ=function(matrix,localZShift)
    --Returns a pose or matrix shifted by localZShift along the matrix's 
    --z-axis.
    local m={}
    for i=1,12,1 do
        m[i]=matrix[i]
    end
    m[4]=m[4]+m[3]*localZShift
    m[8]=m[8]+m[7]*localZShift
    m[12]=m[12]+m[11]*localZShift
    return m
end

forbidThreadSwitches=function(forbid)
    -- Allows or forbids automatic thread switches.
    -- This can be important for threaded scripts. For instance,
    -- you do not want a switch to happen while you have temporarily
    -- modified the robot configuration, since you would then see
    -- that change in the scene display.
    if forbid then
        forbidLevel=forbidLevel+1
        if forbidLevel==1 then
            sim.setThreadAutomaticSwitch(false)
        end
    else
        forbidLevel=forbidLevel-1
        if forbidLevel==0 then
            sim.setThreadAutomaticSwitch(true)
        end
    end
end




findCollisionFreeConfigAndCheckApproach=function(matrix,disThreshold,maxTimeEndConfig)
    -- Here we search for a robot configuration..
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    -- 3. ..that does not collide and that can perform the IK linear approach
    sim.setObjectMatrix(ikTarget,-1,matrix)
    -- Here we check point 1 & 2:
    local c=sim.getConfigForTipPose(ikGroup,jh,disThreshold,
    maxTimeEndConfig,nil,collisionPairs)
    if c then
        -- Here we check point 3:
        local m=getMatrixShiftedAlongZ(matrix,ikShift)
        local path=generateIkPath(c,m,ikSteps)
        if path==nil then
            c=nil
        end
    end
    return c
end

findSeveralCollisionFreeConfigsAndCheckApproach=function(matrix,disThreshold, maxTimeEndConfig,endConfigTrialCnt,maxEndConfigs)
    -- Here we search for several robot configurations...
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    -- 3. ..that does not collide and that can perform the IK linear approach
    forbidThreadSwitches(true)
    sim.setObjectMatrix(ikTarget,-1,matrix)
    local cc=getConfig()
    local cs={}
    local l={}
    for i=1,endConfigTrialCnt,1 do
        local c=findCollisionFreeConfigAndCheckApproach(matrix,
        disThreshold,maxTimeEndConfig)
        if c then
            local dist=getConfigConfigDistance(cc,c)
            local p=0
            local same=false
            for j=1,#l,1 do
                if math.abs(l[j]-dist)<0.001 then
                    --We might have the exact same config. Avoid that.
                    same=true
                    for k=1,#jh,1 do
                        if math.abs(cs[j][k]-c[k])>0.01 then
                            same=false
                            break
                        end
                    end
                end
                if same then
                    break
                end
            end
            if not same then
                cs[#cs+1]=c
                l[#l+1]=dist
            end
        end
        if #l>=maxEndConfigs then
            break
        end
    end
    forbidThreadSwitches(false)
    if #cs==0 then
        cs=nil
    end
    return cs
end

getConfig=function()
    -- Returns the current robot configuration.
    local config={}
    for i=1,#jh,1 do
        config[i]=sim.getJointPosition(jh[i])
    end
    return config
end

setConfig=function(config)
    -- Applies the specified configuration to the robot.
    if config then
        for i=1,#jh,1 do
            sim.setJointPosition(jh[i],config[i])
        end
    end
end

getConfigConfigDistance=function(config1,config2)
    -- Returns the distance (in configuration space) between two  
    --configurations.
    local d=0
    for i=1,#jh,1 do
        local dx=(config1[i]-config2[i])*metric[i]
        d=d+dx*dx
    end
    return math.sqrt(d)
end

getPathLength=function(path)
    -- Returns the length of the path in configuration space.
    local d=0
    local l=#jh
    local pc=#path/l
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],
        path[(i-1)*l+3],path[(i-1)*l+4],
        path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7]}
        local config2={path[i*l+1],path[i*l+2],
        path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6],path[i*l+7]}
        d=d+getConfigConfigDistance(config1,config2)
    end
    return d
end

followPath=function(path)
    -- Follows the specified path points. Each path point is a robot 
    --configuration. Here we don't do any interpolation.
    if path then
        local l=#jh
        local pc=#path/l
        for i=1,pc,1 do
            local config={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],
            path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7]}
            setConfig(config)
            sim.switchThread()
        end
    end
end

findPath=function(startConfig,goalConfigs,maxTimePath,pathStateCnt,cnt)
    --Here we do path planning between the specified start and goal 
    --configurations using the OMPL Library. 
    --We run the search cnt times, returning the shortest path and its length.
    local task=simOMPL.createTask('task')
    simOMPL.setAlgorithm(task,simOMPL.Algorithm.RRTConnect)
    local j1_space=simOMPL.createStateSpace('j1_space',
    simOMPL.StateSpaceType.joint_position,jh[1],
    {-170*math.pi/180},{170*math.pi/180},1)
    local j2_space=simOMPL.createStateSpace('j2_space',
    simOMPL.StateSpaceType.joint_position,jh[2],
    {-120*math.pi/180},{120*math.pi/180},2)
    local j3_space=simOMPL.createStateSpace('j3_space',
    simOMPL.StateSpaceType.joint_position,jh[3],
    {-170*math.pi/180},{170*math.pi/180},3)
    local j4_space=simOMPL.createStateSpace('j4_space',
    simOMPL.StateSpaceType.joint_position,jh[4],
    {-120*math.pi/180},{120*math.pi/180},0)
    local j5_space=simOMPL.createStateSpace('j5_space',
    simOMPL.StateSpaceType.joint_position,jh[5],
    {-170*math.pi/180},{170*math.pi/180},0)
    local j6_space=simOMPL.createStateSpace('j6_space',
    simOMPL.StateSpaceType.joint_position,jh[6],
    {-120*math.pi/180},{120*math.pi/180},0)
    local j7_space=simOMPL.createStateSpace('j7_space',
    simOMPL.StateSpaceType.joint_position,jh[7],
    {-170*math.pi/180},{170*math.pi/180},0)
    simOMPL.setStateSpace(task,{j1_space,j2_space,j3_space,
    j4_space,j5_space,j6_space,j7_space})
    simOMPL.setCollisionPairs(task,collisionPairs)
    simOMPL.setStartState(task,startConfig)
    simOMPL.setGoalState(task,goalConfigs[1])
    for i=2,#goalConfigs,1 do
        simOMPL.addGoalState(task,goalConfigs[i])
    end
    local path=nil
    local l=999999999999
    forbidThreadSwitches(true)
    for i=1,cnt,1 do
        local res,_path=simOMPL.compute(task,maxTimePath,-1,pathStateCnt)
        if res and _path then
            local _l=getPathLength(_path)
            if _l<l then
                l=_l
                path=_path
            end
        end
    end
    forbidThreadSwitches(false)
    simOMPL.destroyTask(task)
    return path,l
end

findShortestPath=function(startConfig,goalConfigs,maxTimePath,
pathStateCnt,pathSearchCnt)
    -- This function will search for several paths between the specified
    -- start configuration, and several of the specified goal configurations.        
    --The shortest path will be returned.
    forbidThreadSwitches(true)
    local thePath=findPath(startConfig,goalConfigs,
    maxTimePath,pathStateCnt,pathSearchCnt)
    forbidThreadSwitches(false)
    return thePath
end

generateIkPath=function(startConfig,goalPose,steps)
    -- Generates (if possible) a linear, collision free path 
    -- between a robot config and a target pose
    forbidThreadSwitches(true)
    local currentConfig=getConfig()
    setConfig(startConfig)
    sim.setObjectMatrix(ikTarget,-1,goalPose)
    local c=sim.generateIkPath(ikGroup,jh,steps,collisionPairs)
    setConfig(currentConfig)
    forbidThreadSwitches(false)
    return c
end

getReversedPath=function(path)
    -- This function will simply reverse a path
    local retPath={}
    local ptCnt=#path/#jh
    for i=ptCnt,1,-1 do
        for j=1,#jh,1 do
            retPath[#retPath+1]=path[(i-1)*#jh+j]
        end
    end
    return retPath
end

isEven=function(n)
    if n % 2 == 0 then
        return true
    else
        return false
    end
end

function sysCall_threadmain()
    --Initialization phase:
    arm_handle = sim.getObjectHandle('LBR_iiwa_7_R800#')
    defaultOrientation1, defaultOrientation2 = generateOrientations()
    jh={-1,-1,-1,-1,-1,-1,-1}
    for i=1,7,1 do
        jh[i]=sim.getObjectHandle('LBR_iiwa_7_R800_joint'..i)
    end
    proximity_sensor=sim.getObjectHandle('BarrettHand_attachProxSensor#')
    obs_collection = sim.getCollectionHandle('obstacles')
    --Setting up environment:
    picked_objects, dummy_handles = createPickObjects({{2, 15, 
    {0.05, 0.05, 0.2}, 0.1, {1.29,0.92,1.35}}, 
    {2, 15, {0.05, 0.05, 0.2}, 0.1, {1.09,0.703,1.35}}})
    createObstacles({{0, 15, {0.1, 0.1, 1.5}, 10, {1.273, -0.055, 0.75}}})
    destinationHandle = findDestinationObject()
    ikGroup=sim.getIkGroupHandle('arm_ik')
    ikTarget=sim.getObjectHandle('arm_target')
    finalDestination=sim.getObjectPosition(ikTarget, -1)
    finalDummy=sim.createDummy(0.05)
    sim.setObjectPosition(finalDummy,-1,finalDestination)
    allTargets={}
    for i=1,#dummy_handles,1 do
        target_start = dummy_handles[i]
        allTargets[2*i-1]=target_start
        target_end = destinationHandle
        allTargets[2*i]=destinationHandle
    end
    allTargets[2*#dummy_handles+1]=finalDummy
    metric={0.5,1,1,0.5,0.1,0.2,0.1}
    forbidLevel=0
    ikShift=0.1
    ikSteps=50
    --Main loop:
    local targetIndex=1
    while true do
        --Holds the arm in place while the Omnirob moves. 
        jpositions = {1, 1, 1, 1, 1, 1, 1}
        for i=1,7,1 do
            jpositions[i] = sim.getJointPosition(jh[i])
        end
        repeat 
            sig=sim.getStringSignal("base_lock")
            for i=1,7,1 do
                sim.setJointPosition(jh[i], jpositions[i])
            end
        until sig=="yes"
        collisionPairs={sim.getCollectionHandle('arm'),obs_collection}
        n = #allTargets
        PickOrPlace = not isEven(targetIndex)
        if PickOrPlace then
            picked_object = picked_objects[(targetIndex + 1) / 2]
        end
        local theTarget=allTargets[targetIndex]
        targetIndex=targetIndex+1
        if targetIndex>n+1 then
            break
        end
        --m is the transformation matrix or pose of the current target:
        local m=sim.getObjectMatrix(theTarget,-1)
        --Compute a pose that is shifted by ikDist along the Z-axis of pose m,
        --so that we have a final approach that is linear along target axis Z:
        m=getMatrixShiftedAlongZ(m,-ikShift)
        --Find several suitable configs for pose m, and order them according    
        --to the distance to current configuration (smaller distance better)
        displayInfo('searching for a maximum of 90 
        valid goal configurations...')
        local c=findSeveralCollisionFreeConfigsAndCheckApproach
        (m,0.65,10,300,90)
        --Search a path from the current config to a goal config. 
        --For the goal configs returned by the above function,
        --search for 6 paths and keep the shortest.
        sim.switchThread() -- to see the change before next operation locks
        local txt='Found '..#c..' different goal configurations 
        for the desired goal pose.'
        txt=txt..'&&nNow searching the shortest path of 6 searches...'
        displayInfo(txt)
        local path=findShortestPath(getConfig(),c,10,300,6)
        displayInfo(nil)
        --Follow the path:
        followPath(path)
        --For the final approach, the target is the original target pose:
        m=sim.getObjectMatrix(theTarget,-1)
        --Compute a straight-line path from current config to pose m:
        path=generateIkPath(getConfig(),m,ikSteps)
        --Follow the path:
        followPath(path)
        jpositions = {1, 1, 1, 1, 1, 1, 1}
        for i=1,7,1 do
            jpositions[i] = sim.getJointPosition(jh[i])
        end
        prox_bool=sim.readProximitySensor(proximity_sensor)
        --Send a signal to pick up the object:
        if prox_bool and PickOrPlace and targetIndex <= n then
            sim.setStringSignal('close_signal', 'yes')
            sim.addObjectToCollection(obs_collection, picked_object,   
            sim.handle_single, 1)
        end
        if prox_bool and not PickOrPlace then
            sim.setStringSignal('close_signal', '')
        end
        --Hold the joints in place while the object is being picked up:
        local t0 = clock()
        while clock() - t0 <= 5 do 
            for i=1,7,1 do
                sim.setJointPosition(jh[i], jpositions[i])
            end
        end
        --Generate a reversed path in order to move back:
        if targetIndex <= n then 
            path=getReversedPath(path)
        --Follow the path:
            followPath(path)
        end
        --Reset the joints to avoid collision:
        if not PickOrPlace then
            for i=1,7,1 do
                sim.setJointPosition(jh[i],0)
            end
        end
        sim.setStringSignal("base_lock", "")
    end
end
