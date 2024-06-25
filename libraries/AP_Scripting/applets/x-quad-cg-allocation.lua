-- x-quad-cg-allocation.lua: Adjust the control allocation matrix for offset CoG.
--
-- WARNING: This script is applicable only to X-type quadrotors and quadplanes.
--
-- How To Use
--   1. Place this script in the "scripts" directory.
--   2. Set the correct value of the ??? parameter.
--   3. Enable the script via the ??? parameter.
--   4. Fly the vehicle.
--
-- How It Works
--   1. The control allocation matrix is adjusted for thrust and pitch based on the ??? parameter value.

--[[
Global definitions.
--]]
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local SCRIPT_NAME = "CoG adjust script"
local LOOP_RATE_HZ = 10

-- State machine states.
local FSM_STATE = {
    INACTIVE = 0,
    ACTIVE = 1,
}
local current_state = FSM_STATE.INACTIVE
local next_state = FSM_STATE.INACTIVE


--[[
New parameter declarations
--]]
local PARAM_TABLE_KEY = 139
local PARAM_TABLE_PREFIX = "CGA_"

-- Bind a parameter to a variable.
function bind_param(name)
   return Parameter(name)
end

-- Add a parameter and bind it to a variable.
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('Could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: CGA_RATIO
  // @DisplayName: CoG adjustment ratio
  // @Description: The ratio between the front and back motor outputs during steady-state hover. Positive when the CoG is in front of the motors midpoint (front motors work harder).
  // @Range: -0.5 0.5
  // @User: Advanced
--]]
CGA_RATIO = bind_add_param('RATIO', 1, 1)

-- Bindings to existing parameters
-- SCR_ENABLED = bind_param("SCR_ENABLED")

--[[
Potential additions:
--]]

--[[
Activation conditions
--]]
-- Check for script activating conditions here.
-- Check frame types.
function can_start()
    return true
end

--[[
Deactivation conditions
--]]
-- Check for script deactivating conditions here.
function must_stop()
    local result = not arming:is_armed()
    return result
end

--[[
State machine
--]]
function fsm_step()
    if current_state == FSM_STATE.INACTIVE then
        if can_start() then
            next_state = FSM_STATE.ACTIVE
        end

    elseif current_state == FSM_STATE.ACTIVE then
        -- Do stuff here.
        if must_stop() then
            next_state = FSM_STATE.INACTIVE
        end
    else
        gcs:send_text(MAV_SEVERITY.CRITICAL, "Unexpected FSM state!")
    end

    current_state = next_state
end

--[[
Main loop function
--]]
function update()
    fsm_step()
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. string.format(" loaded."))

-- Wrapper around update() to catch errors.
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY.EMERGENCY, "Internal Error: " .. err)
     return protected_wrapper, 1000
  end
  return protected_wrapper, 1000.0/LOOP_RATE_HZ
end

-- Start running update loop
return protected_wrapper()