-- Script decodes and logs wind sensor data for Trisonica LI-550 mini
-- https://www.licor.com/products/trisonica/LI-550-mini

-- Parameters:
-- SCR_ENABLE 1
-- SERIAL5_PROTOCOL 28
-- SERIAL5_BAUD 230400

-- In SITL, you can enable serial ports to connect to the real device.
-- https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html#the-8-uarts
-- ./Tools/autotest/sim_vehicle.py -v Plane --console --map -A "--serial5=uart:/dev/ttyUSB0" -D

-- Remember to change baud to 230k in the sensor setup and enable the fields you want.
-- Also, enable 10Hz output instead of the default 5Hz.

-- Example data string (excluding quotes, excluding the carriage return line feed ending)
-- "S  00.08 S2  00.07 D  245 DV  033 U  00.06 V  00.03 W  00.05 T  55889220.00 C  346.68 H  17.92 DP  03.68 P -099.70 AD  0.0000000 AX  -2913 AY  -3408 AZ -16600 PI  011.4 RO  009.8 MX   -619 MY    845 MZ    782 MD  337 TD  337"

-- Log severities
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- find the serial first (0) scripting serial port instance
local port = serial:find_serial(0)

if not port then
    gcs:send_text(MAV_SEVERITY.EMERGENCY, "no trisonica scripting port")
    return
end

-- begin the serial port
port:begin(230400)
port:set_flow_control(0)

local function parse_wind_data(line)
    local result = {}
    local i = 1
    while true do
        -- Find key (letters)
        local key, key_end = line:match("(%a+)%s*()", i)
        if not key then break end

        -- Find value (signed int/float)
        local val, val_end = line:match("([+-]?%d+%.?%d*)%s*()", key_end)
        if not val then break end

        result[key] = tonumber(val)
        i = val_end
    end
    return result
end

-- Parser Tests

-- SAMPLE_FULL = "U -00.01 V -00.02 W  00.05 T  26.11 H  34.15 P  834.61 AD  0.9644222 AX   -797 AY    148 AZ -17356 PI -000.5 RO  002.6 MD  353 TD  353"
-- SAMPLE_CUT_ON_SPACE = " -00.01 V -00.02 W  00.05 T  26.11 H  34.15 P  834.61 AD  0.9644222 AX   -797 AY    148 AZ -17356 PI -000.5 RO  002.6 MD  353 TD  353"
-- SAMPLE_CUT_ON_NUM = "1 V -00.02 W  00.05 T  26.11 H  34.15 P  834.61 AD  0.9644222 AX   -797 AY    148 AZ -17356 PI -000.5 RO  002.6 MD  353 TD  353"

-- print("SAMPLE_FULL", SAMPLE_FULL)
-- t = parse_kv_line(SAMPLE_FULL)
-- print("Len", #t)
-- for k in pairs(t) do print(k, t[k]) end

-- print("SAMPLE_CUT_ON_SPACE", SAMPLE_CUT_ON_SPACE)
-- t = parse_kv_line(SAMPLE_CUT_ON_SPACE)
-- print("Len", #t)
-- for k in pairs(t) do print(k, t[k]) end


-- print("SAMPLE_CUT_ON_NUM", SAMPLE_CUT_ON_NUM)
-- t = parse_kv_line(SAMPLE_CUT_ON_NUM)
-- print("Len", #t)
-- for k in pairs(t) do print(k, t[k]) end

-- Module state
local last_keys     -- table of keys from previous packet
local tag_ids       -- stable ordered key list
local tag_id_str    -- cached string for logger
local value_format  -- cached format string

local function log_wind_data(parsed)
    -- If we don’t yet have stable tag_ids, check for stability
    if not tag_ids then
        -- Build key list in observed order
        local current_keys, n = {}, 0
        for k, _ in pairs(parsed) do
            n = n + 1
            current_keys[n] = k
        end

        -- First packet: just record
        if not last_keys then
            last_keys = current_keys
            return
        end

        -- Different count → not stable yet
        if n ~= #last_keys then
            last_keys = current_keys
            return
        end

        -- Compare element by element
        for i = 1, n do
            if current_keys[i] ~= last_keys[i] then
                last_keys = current_keys
                return
            end
        end

        -- Stable! Lock in the key order
        tag_ids = current_keys
        tag_id_str = table.concat(tag_ids, ',')
        value_format = string.rep('f', n)

        gcs:send_text(MAV_SEVERITY.INFO,
            "Trisonica detected - Using tag_ids: " .. tag_id_str)
    end

    -- At this point tag_ids are stable
    local values, vn = {}, 0
    for i = 1, #tag_ids do
        vn = vn + 1
        values[vn] = parsed[tag_ids[i]] or 0
    end

    assert(#tag_ids < 15, "#tag_ids=" .. #tag_ids)
    logger:write('W3D', tag_id_str, value_format, table.unpack(values, 1, vn))
end

-- buffer table is persistent across update() calls
local buf_t = {}

-- the main update function that is used to read in data from serial port
local function update()
    local n_bytes = port:available()
    while n_bytes > 0 do
        local byte = port:read()
        if byte > -1 then
            local c = string.char(byte)

            if c == "\r" then
                -- join everything collected so far
                local buffer = table.concat(buf_t)
                buf_t = {} -- reset for next line

                local result = parse_wind_data(buffer)
                log_wind_data(result)

            elseif c ~= "\n" then
                -- append character to buffer table
                buf_t[#buf_t+1] = c
            end
        end
        n_bytes = n_bytes - 1
    end
    return update, 20
end

gcs:send_text(MAV_SEVERITY.INFO, "trisonica-mini.lua running...")

return update, 20
