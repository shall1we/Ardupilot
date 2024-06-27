-- x-quad-cg-allocation.lua: Adjust the control allocation matrix for offset CoG.
--
-- WARNING: This script is applicable only to X-type quadrotors and quadplanes.
--
-- How To Use
--   1. Place this script in the "scripts" directory.
--   2. Enable the script via the ??? parameter.
--   3. Reboot.
--   4. Set the correct value of the ??? parameter.
--   5. Fly the vehicle.
--
-- How It Works
--   1. The control allocation matrix is adjusted for thrust and pitch based on the ??? parameter value.

--[[
Global definitions.
--]]
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local SCRIPT_NAME = "CoG adjust script"
local LOOP_RATE_HZ = 10
local last_warning_time_ms = uint32_t() -- Time we last sent a warning message to the user.
local WARNING_DEADTIME_MS = 1000 -- How often the user should be warned.
local is_frame_mc_ok = false
local is_frame_fw_ok = false

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

-- Add param table.
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), SCRIPT_NAME .. ': Could not add param table.')

--[[
  // @Param: CGA_RATIO
  // @DisplayName: CoG adjustment ratio
  // @Description: The ratio between the front and back motor outputs during steady-state hover. Positive when the CoG is in front of the motors midpoint (front motors work harder).
  // @Range: 0.5 2
  // @User: Advanced
--]]
CGA_RATIO = bind_add_param('RATIO', 1, 1)

-- Bindings to existing parameters

--[[
Potential additions:
--]]
-- Warn the user, throttling the message rate.
function warn_user(msg, severity)
    severity = severity or MAV_SEVERITY.WARNING -- Optional severity argument.
    if millis() - last_warning_time_ms > WARNING_DEADTIME_MS then
            gcs:send_text(severity, SCRIPT_NAME .. ": " .. msg)
        last_warning_time_ms = millis()
    end
end

-- Decide if the given ratio value makes sense.
function sanitize_ratio(ratio)
    if (ratio < 0.1) or (ratio > 10) then
        warn_user("CGA_RATIO value out of bounds.")
        return 1.0 -- Return default.
    else
        return ratio
    end
end

-- Adjust the dynamic motor mixer.
function update_mixer(ratio)

    Motors_dynamic:add_motor(0, 1)
    Motors_dynamic:add_motor(1, 3)
    Motors_dynamic:add_motor(2, 4)
    Motors_dynamic:add_motor(3, 2)

    factors = motor_factor_table()

    -- Roll stays as-is.
    factors:roll(0, -0.5)
    factors:roll(1,  0.5)
    factors:roll(2,  0.5)
    factors:roll(3, -0.5)

    -- Pitch is modulated by the ratio.
    factors:pitch(0, 1/(1+ratio))
    factors:pitch(1, -ratio/(1+ratio))
    factors:pitch(2, 1/(1+ratio))
    factors:pitch(3, -ratio/(1+ratio))

    -- Yaw stays as-is
    factors:yaw(0,  0.5)
    factors:yaw(1,  0.5)
    factors:yaw(2, -0.5)
    factors:yaw(3, -0.5)

    -- Throttle stays as-is.
    factors:throttle(0,  1)
    factors:throttle(1,  1)
    factors:throttle(2,  1)
    factors:throttle(3,  1)

    Motors_dynamic:load_factors(factors)

    if not Motors_dynamic:init(4) then
        warn_user("Failed to initialize motor matrix!", MAV_SEVERITY.EMERGENCY)
    else
        warn_user("Set ratio to " .. tostring(ratio), MAV_SEVERITY.INFO)
    end

end

function update_mixer_2(ratio)
    local r1 = 1/(1+ratio)
    local r2 = ratio/(1+ratio)
    -- MotorsMatrix:add_motor_raw(0, -0.5,  r1,  1, 2)
    -- MotorsMatrix:add_motor_raw(1,  0.5, -r2,  1, 4)
    -- MotorsMatrix:add_motor_raw(2,  0.5,  r1, -1, 1)
    -- MotorsMatrix:add_motor_raw(3, -0.5, -r2, -1, 3)
    MotorsMatrix:add_motor_raw(0, -0.5,  0.5,  1, 2)
    MotorsMatrix:add_motor_raw(1,  0.5, -0.5,  1, 4)
    MotorsMatrix:add_motor_raw(2,  0.5,  0.5, -1, 1)
    MotorsMatrix:add_motor_raw(3, -0.5, -0.5, -1, 3)
    MotorsMatrix:set_throttle_factor(0, 2*r2)
    MotorsMatrix:set_throttle_factor(1, r1)
    MotorsMatrix:set_throttle_factor(2, 2*r2)
    MotorsMatrix:set_throttle_factor(3, r1)

    if not MotorsMatrix:init(4) then
        warn_user("Failed to initialize motor matrix!", MAV_SEVERITY.EMERGENCY)
    else
        warn_user("Set ratio to " .. tostring(ratio), MAV_SEVERITY.INFO)
    end
end

-- Decide if the UA is a Quad X quadplane.
function inspect_frame_class_fw()
    local result = false

    Q_ENABLE = bind_param("Q_ENABLE")
    Q_FRAME_CLASS = bind_param("Q_FRAME_CLASS")

    if FWVersion:type()==3 then
        -- Test for the validity of the parameters.
        if Q_ENABLE:get()==1 and Q_FRAME_CLASS:get()==15 then
            result = true
        end
    end
    is_frame_fw_ok = result
end

-- Decide if the UA is a Quad X multicopter.
function inspect_frame_class_mc()
    local result = false

    FRAME_CLASS = bind_param("FRAME_CLASS")

    if FWVersion:type()==2 then 
        if FRAME_CLASS:get()==15 then
            result = true
        end
    end
    is_frame_mc_ok = result
end

--[[
Activation conditions
--]]
-- Check for script activating conditions here.
-- Check frame types.
function can_start()
    result = is_frame_mc_ok or is_frame_fw_ok
    return result
end

--[[
Deactivation conditions
--]]
-- Check for script deactivating conditions here.
function must_stop()
    return false
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
        -- Assert the parameter limits.
        local ratio = sanitize_ratio(CGA_RATIO:get())
        -- Create the control allocation matrix parameters.
        update_mixer_2(ratio)

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

-- Check once on boot if the frame type is suitable for this script.
pcall(inspect_frame_class_mc)
pcall(inspect_frame_class_fw)
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