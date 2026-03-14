sim = require('sim')

DATA_PATH = 'C:/Users/mario/OneDrive/Desktop/MASTER/SEGUNDO_CUATRI/ROBOTS_DE_CAMPO/TRABAJO/FieldRobots_RoboticMaster/simulation_data'

function sysCall_init()
    local win_path = string.gsub(DATA_PATH, "/", "\\")
    os.execute('if not exist "' .. win_path .. '" mkdir "' .. win_path .. '"')
    
    save_mesh_data(DATA_PATH)
end

function sysCall_actuation()
    -- Control logic
end

function sysCall_cleanup()
    -- Cleaning after end simulation
end

function save_mesh_data(folder_path)
    local indices_file = folder_path .. '/indices.csv'
    local vertices_file = folder_path .. '/vertices.csv'
    
    local height_field_handle = sim.getObject('/heightfield')
    local vertices, indices, normals = sim.getShapeMesh(height_field_handle)
    
    -- Store csv
    save_grouped_to_csv(vertices, vertices_file, 3)
    save_grouped_to_csv(indices, indices_file, 3)
end

function save_grouped_to_csv(array, filename, group_size)
    if not array or #array == 0 then
        sim.addLog(sim.verbosity_scripterrors, "Warning: No data found from " .. filename)
        return
    end

    local file = io.open(filename, "w")
    if file then
        for i = 1, #array, group_size do
            local row_data = {}
            for j = 0, group_size - 1 do
                table.insert(row_data, array[i + j])
            end
            local line = table.concat(row_data, ",") .. "\n"
            file:write(line)
        end
        file:close()
        sim.addLog(sim.verbosity_scriptinfos, "Success: " .. (#array / group_size) .. " rows stored in " .. filename)
    else
        sim.addLog(sim.verbosity_scripterrors, "ERROR: Impossible to open " .. filename .. ". Ensure that the folder exists")
    end
end