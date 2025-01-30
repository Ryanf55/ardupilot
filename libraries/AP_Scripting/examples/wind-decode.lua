-- Script decodes and logs wind sensor data.

-- Parameters:
-- SCR_ENABLE 1
-- SERIAL5_PROTOCOL 28

-- In SITL, you can enable serial ports to connect to the real device.
-- https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html#the-8-uarts
-- ./Tools/autotest/sim_vehicle.py -v Plane --console --map -A "--serial5=uart:/dev/ttyUSB0" -D

-- Remember to change baud to 230k in the sensor setup and enable the fields you want.
-- Also, enable 10Hz output instead of the default 5Hz.

-- Example data string (excluding quotes, including the carriage return line feed ending)
-- "S  00.08 S2  00.07 D  245 DV  033 U  00.06 V  00.03 W  00.05 T  55889220.00 C  346.68 H  17.92 DP  03.68 P -099.70 AD  0.0000000 AX  -2913 AY  -3408 AZ -16600 PI  011.4 RO  009.8 MX   -619 MY    845 MZ    782 MD  337 TD  337"

-- Log severities
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- find the serial first (0) scripting serial port instance
local port = serial:find_serial(0)

if not port then
    gcs:send_text(MAV_SEVERITY.EMERGENCY, "No Scripting Serial Port")
    return
end

-- begin the serial port
port:begin(230400)
port:set_flow_control(0)

local buffer = ""

function parse_wind_data(buffer)
    -- Split the string up into key and values splitting on the default space delimiter.

    local parsed_values = {}

    -- Match any key-value pair where key is a string and value is a number
    for key, value in buffer:gmatch("(%a+)%s*([%-%.%d]+)") do
        parsed_values[key] = tonumber(value) -- Store key-value pair in the table
    end

    return parsed_values
end

function log_wind_data(parsed)
    -- Given a table of parsed data, log it.

    local tag_ids = {}
    local values = {}
    -- Build up the logger list of dynamic tag ID's.
    for tag_id, v in pairs(parsed) do
        table.insert(tag_ids, tag_id)
        table.insert(values, v)
    end
    local tag_id_str = table.concat(tag_ids, ',')
    local value_format = string.rep('f', #tag_ids)
    
    assert(#tag_ids < 15, "#tag_ids=" .. #tag_ids)
    logger.write('W3D', tag_id_str, value_format,
        table.unpack(values))
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
                log_wind_data(result)
                buffer = ""
            end
        end
        n_bytes = n_bytes - 1
    end
    return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, "wind-decode.lua running...")

return update, 100


