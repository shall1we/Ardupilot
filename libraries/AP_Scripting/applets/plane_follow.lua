--[[
 support follow "mode" in plane. Theis will actually use GUIDED mode with 
 a scripting switch to allow guided to track the vehicle id in FOLL_SYSID
--]]

SCRIPT_VERSION = "4.6.0-001"
SCRIPT_NAME = "Plane Follow"
SCRIPT_NAME_SHORT = "PFollow"

REFRESH_RATE = 0.1   -- in seconds, so 10Hz
LOST_TARGET_TIMEOUT = 5 / REFRESH_RATE -- 5 seconds
AIRSPEED_GAIN = 1.2

local PARAM_TABLE_KEY = 83
local PARAM_TABLE_PREFIX = "FOLL_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local MODE_GUIDED = 15
local MODE_RTL = 11
local MODE_LOITER = 12

local follow_enabled = false
local too_close_follow_up = 0
local lost_target_countdown = LOST_TARGET_TIMEOUT
DISTANCE_LOOKAHEAD_SECONDS = 10

-- bind a parameter to a variable
function bind_param(name)
   return Parameter(name)
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

-- This uses the exisitng FOLL_* parameters and just adds a couple specific to this script
-- but because most of the logic is already in AP_Follow (called by binding to follow:) there
-- is no need to access them in the scriot
-- FOLL_

-- we need these existing FOLL_ parametrs
FOLL_ALT_TYPE = bind_param('FOLL_ALT_TYPE')

-- Add these FOLL_ parameters specifically for this script
--[[
  // @Param: FOLL_FAIL_MODE
  // @DisplayName: Plane Follow lost target mode
  // @Description: Mode to switch to if the target is lost (no signal or > FOLL_DIST_MAX). 
  // @User: Standard
--]]
FOLL_FAIL_MODE = bind_add_param('FAIL_MODE', 7, MODE_RTL)

--[[
  // @Param: FOLL_EXIT_MODE
  // @DisplayName: Plane Follow exit mode
  // @Description: Mode to switch to when follow mode is exited normally
  // @User: Standard
--]]
FOLL_EXIT_MODE = bind_add_param('EXIT_MODE', 8, MODE_LOITER)

--[[
    // @Param: FOLL_ACT_FN
    // @DisplayName: Plane Follow Scripting ActivationFunction
    // @Description: Setting an RC channel's _OPTION to this value will use it for Plane Follow enable/disable
    // @Range: 300 307
--]]
FOLL_ACT_FN = bind_add_param("ACT_FN", 9, 301)

last_follow_active_state = rc:get_aux_cached(FOLL_ACT_FN:get())

AIRSPEED_MIN = bind_param('AIRSPEED_MIN')
FOLL_OFS_Y = bind_param('FOLL_OFS_Y')
--[[
   return true if we are in a state where follow can apply
--]]
local function follow_active()
   local mode = vehicle:get_mode()

   if mode == MODE_GUIDED then
      if follow_enabled then
        if follow:have_target() then
            return true
        else
            return false
        end
      end
   end

   return false
end

--[[
   check for user activating follow using an RC switch set HIGH
--]]
local function follow_check()
   local active_state = rc:get_aux_cached(FOLL_ACT_FN:get())
   if (active_state ~= last_follow_active_state) then
      if( active_state == 0) then
         if follow_enabled then
            -- Follow disabled - return to EXIT mode
            vehicle:set_mode(FOLL_EXIT_MODE:get())
            follow_enabled = false
            gcs:send_text(MAV_SEVERITY.INFO, "Plane Follow: disabled")
         end
      elseif (active_state == 2) then
         if not (arming:is_armed()) then
            gcs:send_text(MAV_SEVERITY.INFO, "Plane Follow: must be armed")
         end
         -- Follow enabled - switch to guided mode
         vehicle:set_mode(MODE_GUIDED)
         -- Force Guided Mode to not loiter by setting a tiny loiter radius. Otherwise Guide Mode will try loiter around the target vehichle when it gets inside WP_LOITER_RAD
         vehicle:set_guided_radius_and_direction(5, false)
         follow_enabled = true
         gcs:send_text(MAV_SEVERITY.INFO, "Plane Follow: enabled")
      end
      -- Don't know what to do with the 3rd switch position right now.
      last_follow_active_state = active_state
   end
end

-- calculate difference between the target heading and the following vehicle heading
function follow_target_angle(target_location)

   local follow_target_heading = follow:get_target_heading_deg()

   -- find the current location of the vehicle and calculate the bearing to its current target
   local vehicle_location = ahrs:get_location()
   local vehicle_heading = math.deg(vehicle_location:get_bearing(target_location))

   local angle_target = follow_target_heading - vehicle_heading + 360
   if follow_target_heading > vehicle_heading then
      angle_target = follow_target_heading - vehicle_heading
   end
   if angle_target > 180 then
      angle_target = 360 - angle_target
   end
   -- gcs:send_text(MAV_SEVERITY.NOTICE, string.format("heading target: %f vehicle %f angle: %f", follow_target_heading, vehicle_heading, angle_target))

   return angle_target
end

-- main update function
local function update()
   follow_check()
   if not follow_active() then
    return
   end

   --[[
      get the current navigation target. Note that we must get this
      before we check if we are in a follow to prevent a race condition
      with vehicle:update_target_location()
   --]]
   local target_location -- = Location()     of the target
   local target_velocity -- = Vector3f()     -- velocity of lead vehicle
   local target_distance -- = Vector3f()     -- vector to lead vehicle
   local target_offsets -- = Vector3f()      -- vector to lead vehicle + offsets

   local vehicle_airspeed = ahrs:airspeed_estimate()
   local current_target = vehicle:get_target_location()

   -- just because of the methods available on AP_Follow, need to call these two methods 
   -- to get target_location, target_velocity, target distance and target 
   -- and yes target_offsets (hopefully the same value) is returned by both methods
   -- even worse - both internally call get_target_location_and_Velocity, but making a single method
   -- in AP_Follow is probably a high flash cost, so we just take the runtime hit
   target_location, target_offsets = follow:get_target_location_and_velocity_ofs()
   target_distance, target_offsets, target_velocity = follow:get_target_dist_and_vel_ned()
   if target_distance == nil or target_offsets == nil or target_velocity == nil then
      lost_target_countdown = lost_target_countdown - 1
      if lost_target_countdown <= 0 then
         if FOLL_FAIL_MODE:get() ~= nil then
            vehicle:set_mode(FOLL_FAIL_MODE:get())
         end
         follow_enabled = false
         gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": lost target")
         return
      end
   else
      -- have a good target so reset the countdown 
      lost_target_countdown = LOST_TARGET_TIMEOUT
   end


   if target_location == nil or target_velocity == nil or target_offsets == nil or current_target == nil then
      return
   end

   -- default the desired airspeed to the value returned by AP_Follow (this mgith get overrride below)
   local xy_dist = target_offsets:length()
   local target_airspeed = target_velocity:length()
   local desired_airspeed = target_airspeed
   local airspeed_difference = desired_airspeed - vehicle_airspeed
   -- this is the projected distance to the target if we acheive the incremental airspeed increase targeted by AP_Follow
   local projected_distance = xy_dist + (target_airspeed * DISTANCE_LOOKAHEAD_SECONDS - vehicle_airspeed * DISTANCE_LOOKAHEAD_SECONDS)
   --projected_distance = xy_dist + airspeed_difference * DISTANCE_LOOKAHEAD_SECONDS

   -- set the target frame as per user set parameter this might be a bad idea
   --[[ 
   local altitude_frame = FOLL_ALT_TYPE:get()
   if altitude_frame == nil then
      altitude_frame = 0
   end
   target_location:change_alt_frame(altitude_frame)
   --]]

   -- next we try to match the airspeed of the target vehicle, calculating if we
   -- need to speed up if too far behind, or slow down if too close
   
   -- if the current velocity will not catch up to the target then we need to speed up 
   -- use DISTANCE_LOOKAHEAD_SECONDS seconds as a reasonable time to try to catch up
   -- first figure out how far away we will be from the required location in DISTANCE_LOOKAHEAD_SECONDS seconds if we maintain the current vehicle and target airspeeds
   -- There is nothing magic about 12, it is just "what works" 
   local target_angle = follow_target_angle(target_location)
   
   --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": heading: " .. target_heading )
   --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": bearing: " .. vehicle_heading )
   --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": angle: " .. target_angle )

   --[[
   local overshot = (xy_dist > target_airspeed * 2 and xy_dist < (target_airspeed) * DISTANCE_LOOKAHEAD_SECONDS / 1 and target_angle > 45) 
   if overshot then
      gcs:send_text(MAV_SEVERITY.NOTICE, string.format("OVERSHOT xy_dist: %f target arspeed: %f max target airspeed %f target angle %f", xy_dist, target_airspeed, target_airspeed * DISTANCE_LOOKAHEAD_SECONDS / 1, target_angle))
   end
   local too_close = xy_dist < target_airspeed * 2
   if too_close then
      gcs:send_text(MAV_SEVERITY.NOTICE, string.format("TOO CLOSE xy_dist: %f target arspeed: %f max target airspeed %f target angle %f", xy_dist, target_airspeed, target_airspeed * DISTANCE_LOOKAHEAD_SECONDS / 1, target_angle))
   end
   ]]--
   -- gcs:send_text(MAV_SEVERITY.NOTICE, string.format("TOO CLOSE xy_dist: %f target arspeed: %f target angle %f", xy_dist, airspeed_difference, target_angle))
   local too_close = airspeed_difference < 0 or xy_dist < vehicle_airspeed * 2
   --if too_close then
   --   gcs:send_text(MAV_SEVERITY.NOTICE, string.format("TOO CLOSE xy_dist: %f target arspeed: %f max target airspeed %f target angle %f", xy_dist, airspeed_difference, target_airspeed * DISTANCE_LOOKAHEAD_SECONDS / 1, target_angle))
   --end
   -- we've overshot if the distance to the target location is not too_close but will be hit in DISTANCE_LOOKAHEAD_SECONDS
   -- AND the vehicles are heading in the same direction (more or less) - based on target_angle
   if not too_close then
      --gcs:send_text(MAV_SEVERITY.NOTICE, string.format("OVERSHOT? xy_dist: %f airspeed difference: %f max distance %f target angle %f", xy_dist, vehicle_airspeed, vehicle_airspeed * DISTANCE_LOOKAHEAD_SECONDS, target_angle))
   end
   local overshot = not too_close and (xy_dist < vehicle_airspeed * DISTANCE_LOOKAHEAD_SECONDS) and target_angle > 30
   --if overshot then
   --   gcs:send_text(MAV_SEVERITY.NOTICE, string.format("OVERSHOT xy_dist: %f airspeed difference: %f max distance %f target angle %f", xy_dist, vehicle_airspeed, vehicle_airspeed * DISTANCE_LOOKAHEAD_SECONDS, target_angle))
   --end

   if overshot or too_close or too_close_follow_up > 0 then
      -- we have overshot or are just about to hit the target, so artificially jump it ahead 500m
      -- note that is after setting xy_dist so that we still calculate airspeed below
      -- according to the desired location not the one we are telling vehicle about
      
      local target_heading = follow:get_target_heading_deg()
      target_location:offset_bearing(target_heading,500)

      if overshot or too_close then
         too_close_follow_up = too_close_follow_up - 1
      else
         too_close_follow_up = 20
         -- try to reduce the amount we ajust the airspeed also 
      end
      desired_airspeed = target_airspeed
      -- now calculate what airspeed we will need to fly for a few seconds to catch up the projected distance
      -- need to slow down dramatically
      --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": desired speed: " .. desired_airspeed )
      --desired_airspeed = desired_airspeed * (xy_dist / target_airspeed)
      if overshot then
         desired_airspeed = target_airspeed - (target_airspeed - AIRSPEED_MIN:get())/2
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": OVERSHOT desired speed: " .. desired_airspeed )
      else
         desired_airspeed = target_airspeed - (target_airspeed - AIRSPEED_MIN:get())/8
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": TOO CLOSE desired speed: " .. desired_airspeed )
      end
   else
      -- AP_Follow doesn't speed up enough if wa are a long way from the target
      if projected_distance > xy_dist * 2 then
         --gcs:send_text(MAV_SEVERITY.INFO, string.format("Plane Follow: long distance %f projected %f", xy_dist, projected_distance ))
         desired_airspeed = target_airspeed * AIRSPEED_GAIN
      else 
         desired_airspeed = target_airspeed
      end
   end

   if vehicle:update_target_location(current_target, target_location) then
      local foll_ofs_y = FOLL_OFS_Y:get()
      local is_ccw = foll_ofs_y ~= nil and foll_ofs_y > 0

      if not vehicle:set_guided_radius_and_direction(2, is_ccw) then
         gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. string.format(": failed set_guided_radius_and_direction(2, %d)", is_ccw))
      end
   else
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": failed update_target_location")
   end
   -- gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": desired speed: " .. desired_airspeed )
   vehicle:set_desired_airspeed(desired_airspeed)
   -- the desired airspeed will never be acheived, don't worry -we will do this calculation again in REFRESH_RATE seconds, so we can adjust
end

-- wrapper around update(). This calls update() at 20Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
local function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 1000 * REFRESH_RATE
end

gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
  
-- start running update loop
return protected_wrapper()

