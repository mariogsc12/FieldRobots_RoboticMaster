sim=require'sim'

function sysCall_init()
    corout=coroutine.create(coroutineMain)
    sim = require('sim')
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

function sysCall_cleanup()
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
