--[[
  Wireshark dissector for the Freefly AltaX8 DJI-ESC CAN protocol, reverse
  engineered from libraries/AP_Scripting/drivers/AltaX.lua and a PX4 bootup
  candump capture (altax_bootup_stock.candump / altax_px4_bootup.pcapng).

  This runs as a postdissector. Wireshark's built-in "can" (SocketCAN)
  dissector on this build (4.6.6) does NOT expose the CAN ID or DLC as
  named/filterable fields for classic CAN frames - "can.id"/"can.len" turned
  out to belong to an unrelated protocol (CAN-over-AVTP, "acf-can"), and
  "can.data" doesn't exist at all (confirmed with `tshark -G fields`). What
  IS reliable is that the payload always ends up handed to the generic Data
  dissector, so "data.data" exists and its length matches the frame's DLC.

  Rather than depending on any named CAN field, we anchor on "data.data"
  and read the CAN header directly from the raw frame bytes 8 bytes before
  it - matching the fixed classic-CAN capture layout confirmed with
  `tshark -r altax_px4_bootup.pcapng -x`:
      [can_id: 4 bytes, little-endian, top bits are EFF/RTR/ERR flags]
      [can_dlc: 1 byte][3 bytes padding]
      [data: up to 8 bytes]  <- this part is what "data.data" points at
  This is exactly the kernel `struct can_frame` layout (SocketCAN), and it
  holds regardless of what link layer wraps it - confirmed against
  altax_px4_bootup.pcapng, captured by pointing Wireshark directly at the
  `cangs0` SocketCAN interface (Linux cooked capture / SLL wrapping it).

  Install: copy this file into your Wireshark personal plugins folder
  (Help > About Wireshark > Folders > Personal Lua Plugins), then restart
  Wireshark (or Analyze > Reload Lua Plugins) and open the capture.

  Known frame IDs (all standard, non-extended):
    0x010  host -> ESC   init/config command   (5 bytes: subcmd + 4 param bytes)
    0x02A  host -> ESC   request feedback       (1 byte:  target esc index 1-4)
    0x04D  ESC  -> host  feedback: temp/voltage/rpm
    0x04E  ESC  -> host  feedback: current

  A first candump capture (altax_bootup_stock.candump) also showed one-off
  frames on 0x04F/0x050/0x051 (ASCII-looking identification/version strings)
  right at ESC power-up. Two later full-boot captures (altax_px4_bootup.pcapng,
  altax_px4_bootup2.pcapng) never showed them, so support for decoding them
  was removed here rather than being kept on a single unconfirmed sighting.

  Fields not decoded by the ArduPilot lua driver (e.g. 04D bytes 6-7) are
  shown as raw reserved bytes - they may carry duty cycle/status flags that
  simply aren't consumed yet.
--]]

local altax = Proto("altax", "AltaX8 ESC CAN")

local VS_ESC_INDEX = {
    [0] = "broadcast/invalid",
    [1] = "ESC 1",
    [2] = "ESC 2",
    [3] = "ESC 3",
    [4] = "ESC 4",
}

local f_msg_name    = ProtoField.string("altax.msg_name", "Message")

local f_esc_index   = ProtoField.uint8("altax.esc_index", "ESC index", base.DEC, VS_ESC_INDEX, 0x0F)

local f_temp        = ProtoField.uint8("altax.temperature", "ESC temperature (raw, likely deg C)", base.DEC)
local f_voltage_raw = ProtoField.uint16("altax.voltage_raw", "Voltage (raw)", base.DEC)
local f_voltage     = ProtoField.float("altax.voltage", "Voltage (V)")
local f_rpm         = ProtoField.uint16("altax.rpm", "RPM", base.DEC)
local f_reserved    = ProtoField.bytes("altax.reserved", "Reserved / undecoded")

local f_current_raw = ProtoField.uint16("altax.current_raw", "Current (raw)", base.DEC)
local f_current     = ProtoField.float("altax.current", "Current (A)")

local f_req_index   = ProtoField.uint8("altax.request_index", "Requested ESC index", base.DEC)

local f_init_subcmd = ProtoField.uint8("altax.init_subcmd", "Init subcommand", base.HEX)
local f_init_params = ProtoField.bytes("altax.init_params", "Init parameters")

altax.fields = {
    f_msg_name,
    f_esc_index,
    f_temp, f_voltage_raw, f_voltage, f_rpm, f_reserved,
    f_current_raw, f_current,
    f_req_index,
    f_init_subcmd, f_init_params,
}

local ID_INIT           = 0x010
local ID_REQUEST_FB     = 0x02A
local ID_FB_VOLT_RPM    = 0x04D
local ID_FB_CURRENT     = 0x04E

local MSG_NAMES = {
    [ID_INIT]         = "Init/config command",
    [ID_REQUEST_FB]   = "Request feedback",
    [ID_FB_VOLT_RPM]  = "Feedback: temp/voltage/rpm",
    [ID_FB_CURRENT]   = "Feedback: current",
}

local f_can_data_field = Field.new("data.data")

-- classic `struct can_frame`: 4-byte LE id, 1-byte dlc, 3 bytes padding, then data.
local CAN_HEADER_LEN = 8
local CAN_ID_MASK = 0x1FFFFFFF -- strips EFF/RTR/ERR flag bits, leaves 11 or 29-bit ID

-- postdissector: runs on every packet after "can"/"data" have already dissected it.
function altax.dissector(buffer, pinfo, tree)
    local data_finfo = f_can_data_field()
    if not data_finfo or data_finfo.len == 0 then
        -- no CAN payload in this frame (not a CAN frame, or an empty/RTR one)
        return
    end

    local data_len = data_finfo.len
    local data_offset = data_finfo.offset
    local can_offset = data_offset - CAN_HEADER_LEN
    if can_offset < 0 then
        return
    end

    local can_id = buffer(can_offset, 4):le_uint() & CAN_ID_MASK

    if not MSG_NAMES[can_id] then
        -- CAN frame, but not one of ours - leave it alone
        return
    end
    local msg_name = MSG_NAMES[can_id]
    local data = buffer(data_offset, data_len)

    pinfo.cols.protocol:append(" / AltaX")

    local subtree = tree:add(altax, buffer(can_offset, CAN_HEADER_LEN + data_len),
        string.format("AltaX8 ESC CAN: %s", msg_name))
    subtree:add(f_msg_name, msg_name):set_generated()

    if can_id == ID_FB_VOLT_RPM and data_len >= 6 then
        subtree:add(f_esc_index, data(0, 1))
        subtree:add(f_temp, data(1, 1))

        local v_raw = data(2, 1):uint() + (data(3, 1):uint() * 256)
        subtree:add(f_voltage_raw, data(2, 2), v_raw)
        subtree:add(f_voltage, data(2, 2), v_raw * 0.1):append_text(" V")

        local rpm = data(4, 1):uint() + (data(5, 1):uint() * 256)
        subtree:add(f_rpm, data(4, 2), rpm)

        if data_len > 6 then
            subtree:add(f_reserved, data(6, data_len - 6))
        end

        pinfo.cols.info:append(string.format(
            "  AltaX ESC %d: %.1fV  %d RPM  temp=%d",
            data(0, 1):uint() & 0x0F, v_raw * 0.1, rpm, data(1, 1):uint()))

    elseif can_id == ID_FB_CURRENT and data_len >= 3 then
        subtree:add(f_esc_index, data(0, 1))

        local i_raw = (data(1, 1):uint() * 256) + data(2, 1):uint()
        subtree:add(f_current_raw, data(1, 2), i_raw)
        subtree:add(f_current, data(1, 2), i_raw * 0.0001):append_text(" A")

        if data_len > 3 then
            subtree:add(f_reserved, data(3, data_len - 3))
        end

        pinfo.cols.info:append(string.format(
            "  AltaX ESC %d: %.4fA",
            data(0, 1):uint() & 0x0F, i_raw * 0.0001))

    elseif can_id == ID_REQUEST_FB and data_len >= 1 then
        subtree:add(f_req_index, data(0, 1))
        pinfo.cols.info:append(string.format("  AltaX request feedback: ESC %d", data(0, 1):uint()))

    elseif can_id == ID_INIT and data_len >= 1 then
        subtree:add(f_init_subcmd, data(0, 1))
        if data_len > 1 then
            subtree:add(f_init_params, data(1, data_len - 1))
        end
        pinfo.cols.info:append(string.format("  AltaX init command: subcmd=0x%02X", data(0, 1):uint()))

    else
        subtree:add(f_reserved, data)
    end
end

register_postdissector(altax)
