--[[
This script should only be used on multicopters
The gripper is released when thrust loss is detected
Note that thrust loss false positives are common meaning the gripper may be released unnecessarily
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local update_interval_ms = 100
local thrust_loss_counter = 0

function update()

   -- return immediately if not armed
   if not arming:is_armed() then
      thrust_loss_counter = 0
      return update, update_interval_ms
   end

   -- return if no thrust loss
   if not MotorsMatrix:get_thrust_boost() then
      thrust_loss_counter = 0
      return update, update_interval_ms
   end

   -- return if vehicle is not descending (or cannot retrieve climb rate)
   local vel_NED = ahrs:get_velocity_NED()
   if vel_NED == nil or vel_NED:z() <= 0 then
      thrust_loss_counter = 0
      return update, update_interval_ms
   end

   -- vehicle is descending and thrust loss is detected
   thrust_loss_counter = thrust_loss_counter + 1
   if thrust_loss_counter < 10 then
      return update, update_interval_ms
   end

   -- release gripper and warn user
   if not gripper:released() then
      gripper:release()
      gcs:send_text(MAV_SEVERITY.WARNING, "Thrust loss, gripper released!")
   end

   return update, update_interval_ms
end

return update, update_interval_ms
