sim=require'sim'

function sysCall_init()
    corout=coroutine.create(coroutineMain)
    sim = require('sim')
end

function sysCall_actuation()
    -- Control logic
end

function sysCall_cleanup()
    -- Cleaning after end simulation
end

function sysCall_thread()
    motorHandles={-1,-1,-1,-1}

    motorHandles[1]=sim.getObject('../front_left_wheel')
    motorHandles[2]=sim.getObject('../front_right_wheel')
    motorHandles[3]=sim.getObject('../back_right_wheel')
    motorHandles[4]=sim.getObject('../back_left_wheel')

end

-- See the user manual or the available code snippets for additional callback functions and details
function coroutineMain()
    simRemoteApi.start(19999)
end
