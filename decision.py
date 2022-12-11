import numpy as np
import time



# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            #---#
            #if the terrain that the rover drives through would reach an end it will take one of the other angles
            #if the distances of the other angles > 20 then obtain their mean.
            if (len(Rover.nav_angles) >= Rover.stop_forward) and \
                    (np.mean(Rover.nav_dists > 20)):
                # If mode is forward, navigable terrain looks good
                # and velocity is below max and not turning, then throttle
                if (Rover.vel < Rover.max_vel) and (Rover.steer < 5):
                    # Set throttle value to throttle setting

                    #---#
                    #Steer in the direction of the mean angles that have distances > 20

                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                    Rover.throttle = Rover.throttle_set
                    #---# if the rover is steering or over speed limit stop in order to take a desicion
                else:  # Else coast
                    Rover.throttle = 0
                #---# if all terrains will reach an end brake
                Rover.brake = 0
                #---# if the velocity > vel_limit then we will never be able to steer so we must stop first
                #---# to get the mean of the angles that have distances > 20 to navigate through
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            #---# if the magnitude of the angles is < distances between the rest of the pixels before crash
            #---# and there is no distances > 20 there to navigate through then stop the acceleration and steering
            #and everything
            elif (len(Rover.nav_angles) < Rover.stop_forward) or \
                    (np.mean(Rover.nav_dists < 20)):
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
            # --------------------------------------------------------------------------
            # This will determine if the rover get stuck
            # Rover.stuck_time is a commulative frame counter that rover get stuck
            #---# if the velocity < 0.05 and its in the move forward mode then it is stuck so we make stuck time
            #---# and increment it until it's not stuck
            #---# we compare the stuck time with the error limit if the number of franes > 120 (4.8 seconds)
            #---# set throttle to -ve in order to navigate through the opposite direction and in order to make this
            #---# it must steer in the opposite direction hence negative


            if (np.abs(Rover.vel) <= 0.05):
                Rover.stuck_time += 1
            # if it get can get out then fine, reset counter
            else:
                Rover.stuck_time = 0
            # if the rover get stuck, move backward in oppospite direction
            if Rover.stuck_time > Rover.error_limit:
                Rover.throttle = -Rover.throttle_set
                Rover.steer = -np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
               # time.sleep(0.5)
            # --------------------------------------------------------------------------

        # If we're already in "stop" mode then make different decisions
        #---# if we are in the stop mode and the velocity > 0.5 stop it and will not be able to steer
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.5:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving then do something else
            elif Rover.vel <= 0.5:
                # Now we're stopped and there is no path forward
                #---# if we are in stop mode, not moving and all of the available angles < threshold to start moving
                #---# then throttle = 0 and we will release the brakes in order to begin steering with the 4 wheels -15 degrees
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15
                # If we're stopped but see sufficient navigable terrain in front then go!
                #---# if we are not moving and there is a navigable terrain available throttle will begin to increase
                #---# and we will release the brake
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    #---#
                    # np.clip takes the mean angles and converts them to radians then the -15 is the minimum
                    # and the 15 is maximum so we clip all angles under -15 and above 15
                    #---# after it is stuck it figures out the way that will navigate through in range from
                    #--# -15 to 15 degrees
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                    Rover.mode = 'forward'
        # If any sample detected, go to sample
        elif Rover.mode == 'goto_rock':
            # if the rover can pick up sample, stop and pick it up
            if Rover.near_sample:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
            # if cannot pick up, but the rover is getting close to the sample, start to brake
            elif Rover.vel > np.mean(Rover.nav_dists):
                Rover.throttle = 0
                Rover.brake = Rover.brake_set / 2
            # if the rover is not close to the sample yet, continue going with speed limit
            elif Rover.vel < Rover.max_vel / 2:
                Rover.throttle = Rover.throttle_set / 2
                Rover.brake = 0
            # if the rover is over the speed limit, brake slowly
            elif Rover.vel > Rover.max_vel / 2:
                Rover.throttle = 0
                Rover.brake = Rover.throttle_set / 3
            # --------------------------------------------------------------------------
            # This will determine if the rover get stuck
            # Rover.stuck_time is a commulative frame counter that rover get stuck
            if (np.abs(Rover.vel) <= 0.05):
                Rover.stuck_time += 1
            # if it get can get out then fine, reset counter
            else:
                Rover.stuck_time = 0
            # if the rover get stuck, move backward in oppospite direction
            if Rover.stuck_time > Rover.error_limit:
                Rover.throttle = -Rover.throttle_set
                Rover.steer = -np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                #time.sleep(1)
            # --------------------------------------------------------------------------
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
            # if the sample is picked up, exit this mode
            if Rover.picking_up:
                Rover.mode = 'stop'
        # elif Rover.mode=='HasVisited':
        #     Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi),-15,15)

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        # Enter mode 'stop' after picking up
        Rover.mode = 'stop'
    # if  Rover.worldmap == np.zeros((200, 200, 3), dtype=np.float):
    #     Rover.mode = 'HasVisited'

    return Rover


