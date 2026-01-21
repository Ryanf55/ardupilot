-- Replay motor outputs from CSV file
-- Reads RCOU CSV line-by-line and writes motor outputs using SRV_Channels

local PARAM_TABLE_KEY = 136
local PARAM_TABLE_PREFIX = "CSVR_"

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

local CSVR_ENABLE = bind_add_param('ENABLE', 1, 0)
local CSVR_DEBUG  = bind_add_param('DEBUG', 2, 0)  -- Enable debug logging

-- Playback state
local file_handle = nil
local start_time_us = 0
local csv_start_time_us = 0
local replay_started = false
local next_sample = nil
local current_sample = nil  -- Last sample we wrote
local playback_complete = false
local samples_written = 0
local last_log_time = 0
local samples_skipped = 0

-- Parse a CSV line into a sample
function parse_line(line)
   local values = {}
   for val in string.gmatch(line, "([^,]+)") do
      table.insert(values, tonumber(val))
   end
   
   if #values == 5 then
      return {
         time_us = values[1],
         c1 = values[2],
         c2 = values[3],
         c3 = values[4],
         c4 = values[5]
      }
   end
   return nil
end

-- Open CSV file and skip header
function open_csv()
   local file = io.open("rcou_replay.csv", "r")
   if not file then
      gcs:send_text(0, "CSV Replay: Could not open rcou_replay.csv")
      return nil
   end
   
   -- Skip header line
   file:read("*line")
   
   gcs:send_text(6, "CSV Replay: File opened")
   return file
end

-- Read the next sample from file
function read_next_sample()
   if not file_handle then
      return nil
   end
   
   local line = file_handle:read("*line")
   if not line then
      return nil
   end
   
   return parse_line(line)
end

-- Close file and reset state
function stop_playback()
   if file_handle then
      file_handle:close()
      file_handle = nil
   end
   replay_started = false
   next_sample = nil
   current_sample = nil
   playback_complete = false
   samples_written = 0
   samples_skipped = 0
   last_log_time = 0
end

function update()
   local enabled = CSVR_ENABLE:get()
   
   if enabled == 0 then
      if replay_started then
         stop_playback()
         gcs:send_text(6, "CSV Replay: Stopped")
      end
      return update, 100  -- 10Hz when disabled
   end
   
   -- Initialize playback
   if not replay_started then
      stop_playback()  -- Clean up any previous state
      
      file_handle = open_csv()
      if not file_handle then
         CSVR_ENABLE:set(0)
         return update, 1000
      end
      
      -- Read first sample to get start time
      next_sample = read_next_sample()
      if not next_sample then
         gcs:send_text(0, "CSV Replay: Empty file")
         stop_playback()
         CSVR_ENABLE:set(0)
         return update, 1000
      end
      
      csv_start_time_us = next_sample.time_us
      start_time_us = millis():tofloat() * 1000
      replay_started = true
      gcs:send_text(6, "CSV Replay: Starting playback")
   end
   
   -- Calculate current time relative to start
   local current_time_us = millis():tofloat() * 1000
   local elapsed_us = current_time_us - start_time_us
   local target_time_us = csv_start_time_us + elapsed_us
   
   -- Debug logging every second
   local debug_enabled = CSVR_DEBUG:get() == 1
   if debug_enabled and (current_time_us - last_log_time) > 1000000 then
      local time_diff = next_sample and (next_sample.time_us - target_time_us) or 0
      gcs:send_text(6, string.format("CSV: wrote=%d skip=%d diff=%d", 
         samples_written, samples_skipped, math.floor(time_diff)))
      last_log_time = current_time_us
   end
   
   -- Process all samples up to current time
   local wrote_this_cycle = 0
   while next_sample and next_sample.time_us <= target_time_us do
      -- Update current sample to the new one
      current_sample = next_sample
      
      samples_written = samples_written + 1
      wrote_this_cycle = wrote_this_cycle + 1
      
      -- Warn if writing too many samples in one cycle (catching up)
      if wrote_this_cycle > 5 then
         samples_skipped = samples_skipped + 1
      end
      
      -- Read next sample
      next_sample = read_next_sample()
      
      -- Check if we've reached the end
      if not next_sample then
         gcs:send_text(6, string.format("CSV Replay: Complete. Total samples: %d", samples_written))
         stop_playback()
         CSVR_ENABLE:set(0)
         break
      end
   end
   
   -- Always write the current sample (last known values) to prevent timeout
   if current_sample then
      SRV_Channels:set_output_pwm_chan_timeout(0, current_sample.c1, 100)
      SRV_Channels:set_output_pwm_chan_timeout(1, current_sample.c2, 100)
      SRV_Channels:set_output_pwm_chan_timeout(2, current_sample.c3, 100)
      SRV_Channels:set_output_pwm_chan_timeout(3, current_sample.c4, 100)
   end
   
   return update, 5  -- 200Hz
end

gcs:send_text(6, "CSV Replay: Script loaded")
return update()