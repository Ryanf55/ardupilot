-- Script decodes and logs wind sensor data.

-- Parameters:
-- SCR_ENABLE 1
-- SERIAL5_PROTOCOL 28

-- In SITL, you can enable serial ports to connect to the real device.
-- https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html#the-8-uarts
-- ./Tools/autotest/sim_vehicle.py -v Plane --console --map -A "--serial5=uart:/dev/ttyUSB0"

-- Example data string (excluding quotes, including the carriage return line feed ending)
-- "S  00.06 D  346 U  00.01 V -00.06 W  00.02 T  23.89 H  20.26 P  827.32 PI -011.0 RO  014.4 MD  339␍␊"

-- Log severities
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- find the serial first (0) scripting serial port instance
local port = serial:find_serial(0)

if not port then
    gcs:send_text(MAV_SEVERITY.EMERGENCY, "No Scripting Serial Port")
    return
end

-- begin the serial port
port:begin(115200)
port:set_flow_control(0)


local buffer = ""

function parse_wind_data(buffer)
    local parsed_values = {}

    -- for value, key in buffer:gmatch("([%-%.%d]+)%s*([SDUVWT])") do
    --     if key == "S" then
    --         parsed_values.S = tonumber(value)
    --     elseif key == "D" then
    --         parsed_values.D = tonumber(value)
    --     elseif key == "U" then
    --         parsed_values.U = tonumber(value)
    --     elseif key == "V" then
    --         parsed_values.V = tonumber(value)
    --     elseif key == "W" then
    --         parsed_values.W = tonumber(value)
    --     elseif key == "T" then
    --         parsed_values.T = tonumber(value)
    --     end
    -- end

    -- Match any key-value pair where key is a string and value is a number
    for key, value in buffer:gmatch("(%a+)%s*([%-%.%d]+)") do
        parsed_values[key] = tonumber(value) -- Store key-value pair in the table
    end

    return parsed_values
end

-- the main update function that is used to read in data from serial port
function update()

    if not port then
        gcs:send_text(0, "no Scripting Serial Port")
        return update, 100
    end

    -- gcs:send_text(MAV_SEVERITY.INFO, "update()")

    local n_bytes = port:available()
    while n_bytes > 0 do
        local byte = port:read()
        -- gcs:send_text(MAV_SEVERITY.INFO, "Got bytes...")

        if byte then
            local c = string.char(byte)
            -- gcs:send_text(MAV_SEVERITY.INFO, "Byte " ..  type(c) .. ": '" .. c .. "'")
            buffer = buffer .. c

            if c == "\r" then
                -- gcs:send_text(MAV_SEVERITY.INFO, "End buffer: " .. buffer)
                result = parse_wind_data(buffer)
                local tag_ids = {}
                local values = {}
                -- Build up the logger list of dynamic tag ID's.
                for tag_id, v in pairs(result) do
                    table.insert(tag_ids, tag_id)
                    table.insert(values, v)
                end
                local tag_id_str = table.concat(tag_ids, ',')
                local value_format = string.rep('f', #tag_ids)
                
                logger.write('W3D', tag_id_str, value_format,
                    table.unpack(values))
                
                buffer = ""  -- Clear buffer for the next message
            end
        end
        n_bytes = n_bytes - 1
    end
    return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, "wind-decode.lua running...")

return update, 100


