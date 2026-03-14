sim = require('sim')

function sysCall_init()
    print_all_scene_objects()
end

function sysCall_actuation()
    -- Control logic
end

function sysCall_cleanup()
    -- Cleaning after end simulation
end

function print_all_scene_objects()
    -- sim.handle_scene is the root of the simulation
    local all_objects = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    
    sim.addLog(sim.verbosity_scriptinfos, "=== ALL OBJECTS OF THE SCENE ===")
    
    local count = 0
    for i, handle in ipairs(all_objects) do
        -- get the object alias
        local name = sim.getObjectAlias(handle)
        sim.addLog(sim.verbosity_scriptinfos, "ID: " .. handle .. " | Name: " .. name)
        count = count + 1
    end
    
    sim.addLog(sim.verbosity_scriptinfos, "Total objects found: " .. count)
    sim.addLog(sim.verbosity_scriptinfos, "===============================================")
end