-- command a Copter to takeoff to 50m and fly a square pattern
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and RC7 input > 1800 and then:
--    a) switches to Guided mode
--    b) takeoff to 50m
--    c) flies a 10m x 10m square pattern using the velocity controller
--    d) switches to Circle mode
--    e) switches to RTL mode

local takeoff_alt_above_home = 50
local return_alt_above_home = 5
local copter_guided_mode_num = 4
local copter_rtl_mode_num = 6
local copter_circle_mode_num = 7
local stage = 0
local bottom_left_loc   -- vehicle location when starting square
local square_side_length = 10   -- length of each side of square
local circle_ground_speed = 6 -- the ground speed in circle mode (m/s)

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
function update()
  if not arming:is_armed() then -- reset state when disarmed
    stage = 0
  else
    pwm7 = rc:get_pwm(7)
    if pwm7 and pwm7 > 1800 then    -- check if RC7 input has moved high
      if (stage == 0) then          -- change to guided mode
        if (vehicle:set_mode(copter_guided_mode_num)) then     -- change to Guided mode
          stage = stage + 1
        end
      elseif (stage == 1) then      -- Stage1: takeoff
        if (vehicle:start_takeoff(takeoff_alt_above_home)) then
          stage = stage + 1
        end
      elseif (stage == 2) then      -- Stage2: check if vehicle has reached target altitude
        local home = ahrs:get_home()
        local curr_loc = ahrs:get_location()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          gcs:send_text(0, "alt above home: " .. tostring(math.floor(-vec_from_home:z())))
          if (math.abs(takeoff_alt_above_home + vec_from_home:z()) < 1) then
            stage = stage + 1
            bottom_left_loc = curr_loc          -- record location when starting square
          end
        end
      elseif (stage >= 3 and stage <= 6) then   -- fly a square using velocity controller
        local curr_loc = ahrs:get_location()
        local target_vel = Vector3f()           -- create velocity vector
        if (bottom_left_loc and curr_loc) then
          local dist_NE = bottom_left_loc:get_distance_NE(curr_loc)

          -- Stage3 : fly North at 2m/s
          if (stage == 3) then
            target_vel:x(2)
            if (dist_NE:x() >= square_side_length) then
              stage = stage + 1
            end
          end

          -- Stage4 : fly East at 2m/s
          if (stage == 4) then
            target_vel:y(2) 
            if (dist_NE:y() >= square_side_length) then
              stage = stage + 1
            end
          end

          -- Stage5 : fly South at 2m/s
          if (stage == 5) then
            target_vel:x(-2)
            if (dist_NE:x() <= 2) then
              stage = stage + 1
            end
          end

          -- Stage6 : fly West at 2m/s
          if (stage == 6) then
            target_vel:y(-2)
            if (dist_NE:y() <= 2) then
              stage = stage + 1
            end
          end

          -- send velocity request
          if (vehicle:set_target_velocity_NED(target_vel)) then   -- send target velocity to vehicle
            gcs:send_text(0, "pos:" .. tostring(math.floor(dist_NE:x())) .. "," .. tostring(math.floor(dist_NE:y())) .. " sent vel x:" .. tostring(target_vel:x()) .. " y:" .. tostring(target_vel:y()))
          else
            gcs:send_text(0, "failed to execute velocity command")
          end
        end

      elseif (stage == 7) then  --Stage7: change to CIRCLE mode
        vehicle:set_mode(copter_circle_mode_num)
        if (not arming:is_armed()) or (not vehicle:get_likely_flying()) or (vehicle:get_mode() ~= 7) then
          return update, 1000 -- reschedules the loop, 1hz
        end
      
        -- get circle radius
        local radius = vehicle:get_circle_radius()
        if not radius then
          gcs:send_text(0, "failed to get circle radius")
          return update, 1000
        end
      
        -- set the rate to give the desired ground speed at the current radius
        local new_rate = 360 * (circle_ground_speed / (radius*math.pi*2))
        new_rate = math.max(new_rate, -90)
        new_rate = math.min(new_rate, 90)
        if not vehicle:set_circle_rate(new_rate) then
          gcs:send_text(0, "failed to set rate")
        else
          gcs:send_text(0, string.format("radius:%f new_rate=%f", radius, new_rate))
        end

        local home = ahrs:get_home()
        local curr_loc = ahrs:get_location()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          gcs:send_text(0, "alt above home: " .. tostring(math.floor(-vec_from_home:z())))
          if (math.abs(return_alt_above_home + vec_from_home:z()) < 1) then
            stage = stage + 1
          end
        end

        return update, 100 -- reschedules the loop, 10hz

      elseif (stage == 8) then  -- Stage8: change to RTL mode
        vehicle:set_mode(copter_rtl_mode_num)
        stage = stage + 1
        gcs:send_text(0, "finished circle, switching to RTL")
      end
    end
  end

  return update, 1000
end

return update()
