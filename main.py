#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import json

ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
medium_motor = Motor(Port.A)

light_sensor = ColorSensor(Port.S4)
secondary_light_sensor = ColorSensor(Port.S3)

ev3.speaker.set_speech_options("en", "m1", 100, 75)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=117)
robot.settings(700, 300, 250, 250)
#Start up and intialize the screen, speakers, motors, sensors and the robot drivebase.

def rgb_reflection():
    x = light_sensor.rgb()
    return ((x[0]+x[1]+x[2])/3)
# Give an average of all three colors lighted up by the first light sensor to get an reliable average of the light reflection.

def secondary_rgb_reflection():
    x = secondary_light_sensor.rgb()
    return ((x[0]+x[1]+x[2])/3)
# Give an average of all three colors lighted up by the second light sensor to get an reliable average of the light reflection.

calibrate = True
prompt = True
firstblack = True
firstwhite = True
secondblack = True
secondwhite = True
benchrun = True
sliderun = True
steptreadrun = True
basketballrun = True
# Set all these variables to True so they activate their respective function until told they are False.

ev3.screen.clear()
ev3.screen.draw_text(3, 20, "   Do you want to")
ev3.screen.draw_text(5, 40, "       calibrate?")
ev3.screen.draw_text(6, 60, "(Press Up for Yes")
ev3.screen.draw_text(8, 80, "and Down for No)")
# Display on the screen if we want to calibrate the light sensors.
while prompt:
    if Button.UP in ev3.buttons.pressed():
        prompt = False
    if Button.DOWN in ev3.buttons.pressed():
        calibrate = False
        prompt = False
        wait(225)
        # Wait to get into the program UI because then if there is no wait it'll immediately register as you wanting to do program #5.

# This while loop checks if you want to calibrate or not, and if you press up it will take you to the calibrate loop.

if calibrate:
    ev3.screen.clear()
    ev3.screen.draw_text(15, 45, "Place right light")
    ev3.screen.draw_text(15, 65, "sensor on black!")
    # Display on the screen to put the right light sensor on black and press the center button to confirm.

    while firstblack:
        if Button.CENTER in ev3.buttons.pressed():
            value_dict = {
                "black" : rgb_reflection(),
                "white" : 100
            }
            json_object = json.dumps(value_dict)
            with open("light.json", "w") as light:
                light.write(json_object)
            wait(275)
            firstblack = False
    # Saves this value as black for the right light sensor to a json file that can be saved even when the program terminates.

    ev3.screen.clear()
    ev3.screen.draw_text(15, 45, "Place right light")
    ev3.screen.draw_text(15, 65, "sensor on white!")
    # Display on the screen to put the right light sensor on white and press the center button to confirm.

    while firstwhite:
        if Button.CENTER in ev3.buttons.pressed():
            value_dict["white"] = (rgb_reflection())
            json_object = json.dumps(value_dict)
            with open("light.json", "w") as light:
                light.write(json_object)
            wait(275)
            firstwhite = False
    # Saves this value as white for the right light sensor to a json file that can be saved even when the program terminates.

    ev3.screen.clear()
    ev3.screen.draw_text(15, 45, "Place left light")
    ev3.screen.draw_text(15, 65, "sensor on black!")
    # Display on the screen to put the left light sensor on black and press the center button to confirm.

    while secondblack:
        if Button.CENTER in ev3.buttons.pressed():
            value_dict2 = {
                "black" : secondary_rgb_reflection(),
                "white" : 100
            }
            json_object = json.dumps(value_dict2)
            with open("secondary_light.json", "w") as light:
                light.write(json_object)
            wait(275)
            secondblack = False
    # Saves this value as black for the left light sensor to a second json file that can be saved even when the program terminates.
    
    ev3.screen.clear()
    ev3.screen.draw_text(15, 45, "Place left light")
    ev3.screen.draw_text(15, 65, "sensor on white!")
    # Display on the screen to put the left light sensor on white and press the center button to confirm.

    while secondwhite:
        if Button.CENTER in ev3.buttons.pressed():
            value_dict2["white"] = (secondary_rgb_reflection())
            json_object = json.dumps(value_dict2)
            with open("secondary_light.json", "w") as light:
                light.write(json_object)
            wait(275)
            secondwhite = False
    # Saves this value as white for the left light sensor to a second json file that can be saved even when the program terminates.

# This big if statement checks the light sensor values when you put the sensors on white/black, and then tells the line follow function what values are considered black and what values are considered white.

def line_follow(rotations, speed, kp, kd): # A pd line follow function (using only proporiontal and the derivitive).
    control = 0
    last_deviation = 0
    left_motor.reset_angle(0) # Reset left motor so no previous angle measurements interfere
    right_motor.reset_angle(0) # Reset right  motor so no previous angle measurements interfere
    with open("light.json", "r") as light: # Open our json file of the light sensor readings when they are on black/white
        light_dict = json.load(light)
    middle =  ((light_dict["black"]) + (light_dict["white"]))/2 # Take the middle of the black and white readings 
    #so you follow the middle of a line.
    # --v This is not allowed in lego LabView, and with micropython we were able to line follow more accurately.
    while ((left_motor.angle())+(right_motor.angle()))/2 < 360*(rotations): # Check if the robot goes more than target every tick
        deviation = rgb_reflection() - middle # Calculate deviation from the middle of the line
        proportional = (kp) * deviation # Find out how much the robot needs to turn based off of deviation.
        derivitive = (deviation - last_deviation) * (kd)
        turn_rate = proportional + derivitive
        last_deviation = deviation # get this to be recursive
        robot.drive((speed), turn_rate) # Actually turn the robot to correct the errors.
    robot.stop() # Once target is reached, tell the motors that it is done so it's ready to do the next task.


def secondary_line_follow(rotations, speed, kp, kd): # A pd line follow function (using only proporiontal and the derivitive).
    control = 0
    last_deviation = 0
    left_motor.reset_angle(0) # Reset left motor so no previous angle measurements interfere
    right_motor.reset_angle(0) # Reset right  motor so no previous angle measurements interfere
    with open("secondary_light.json", "r") as light:
        light_dict2 = json.load(light)
    # Open up the second json file to find the values of black and white for the left light sensor
    middle =  (((light_dict2["black"]) + (light_dict2["white"]))/2) # Take the average of readings between both motors for more accurate measurement.
    # --v This is not allowed in lego LabView, and with micropython we were able to line follow more accurately.
    while ((left_motor.angle())+(right_motor.angle()))/2 < 360*(rotations): # Check if the robot goes more than target every tick
        deviation = secondary_rgb_reflection() - middle # Calculate deviation from the middle of the line
        proportional = (kp) * deviation # Find out how much the robot needs to turn based off of deviation.
        derivitive = (deviation - last_deviation) * (kd)
        turn_rate = proportional + derivitive
        control += 1
        if control==5:
            control = 0
        if control==4:
            last_deviation = deviation # get this to be recursive
        robot.drive((speed), turn_rate) # Actually turn the robot to correct the errors.
    robot.stop() # Once target is reached, tell the motors that it is done so it's ready to do the next task.

ev3.screen.clear()

while benchrun or sliderun or steptreadrun or basketballrun:
# When any of the mission runs haven't been done, this master program will not terminate. The program will only end when all of the missions have been done.
    ev3.screen.draw_text(0, 0, "Pick Program")
    ev3.screen.draw_text(85, 28, "1")
    ev3.screen.draw_text(41, 60, "2")
    ev3.screen.draw_text(85, 60, "3")
    ev3.screen.draw_text(130, 60, "4")
    ev3.screen.draw_text(85, 92, "5")
    # ^ to draw a little map of which button goes to each program
    if Button.UP in ev3.buttons.pressed(): # If you press the UP button, then the bench mission, innovation project, and health units program will run. f
        ev3.screen.clear()
        ev3.screen.draw_text(55, 50, "Bench")
        ev3.screen.draw_text(42, 70, "is running.")
        # Write on the screen the Bench program is running

        robot.settings(30, 250, 250, 250)
        robot.straight(25)
        robot.stop()
        # Go straight slowly so the robot doesn't slip and has a good start, and brake so we are ready for the next action.

        robot.settings(100, 250, 250, 250)
        robot.straight(350)
        robot.stop()
        # Go straight at a more moderate pace and brake/stop.

        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        while ((left_motor.angle())+(right_motor.angle()))/2 <= 100:
            robot.drive(50, -20)
        robot.stop()
        # Drive forward at a little bit of an angle to activate the bench mission

        robot.settings(400, 300, 250, 250)
        robot.straight(-250)
        robot.stop()
        # Go backwards after activating the bench moving the innovation project/health units to the gray area.

        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        while ((left_motor.angle())+(right_motor.angle()))/2 >= -500:
            robot.drive(-500, 80)
        robot.stop()
        # Come backwards at an extreme angle so the attachment comes home.

        ev3.speaker.play_file(SoundFile.KUNG_FU) # Play a fun sound so we are exitced and we know the program is fully done!
        ev3.screen.clear() # Clear the screen from saying that "Bench is running" so it can switch back to the main screen where it maps out the buttons.
        benchrun = False # Set this variable to false so it knows that we completed this if statement. 

    if Button.LEFT in ev3.buttons.pressed():
        ev3.screen.clear()
        ev3.screen.draw_text(42, 50, "Boccia&Slide")
        ev3.screen.draw_text(42, 70, "is running.")
        # Write on the screen the Boccia and Slide mission is running.

        robot.straight(510)
        robot.stop()
        # Go straight so we can reach the line at a fast pace.

        line_follow(2.4, 160, -0.4, 1)
        # Line follow with the right light sensor until we reach the Pull Up Bar.

        right_motor.reset_angle(0)
        right_motor.run_target(500, 400, then=Stop.BRAKE, wait=True)
        right_motor.stop()
        # Turn with one motor so we go under the Pull Up Bar and are parallel to it.

        line_follow(1.6, 190, -0.4, 1)
        # Line follow under the Pull Up Bar at a fast pace.

        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        while ((left_motor.angle())+(right_motor.angle()))/2 <= 200:
            robot.drive(75, 10)
        robot.stop()
        #Go forward at an angle to combat the backlash of our actachment holding onto the pull up bar.

        line_follow(0.5, 100, -0.4, 1)
        # Line follow for a little strip to re-align ourselves parallel to the pull up bar.

        robot.straight(105)
        # Go straight so we are at the Boccia mission.

        medium_motor.reset_angle(0)
        medium_motor.run_target(100, -130, then=Stop.BRAKE, wait=True)
        medium_motor.stop()
        # Drop the 8 cubes into the Boccia target.
        
        robot.straight(-90)
        robot.stop()
        # Go backwards a little so we have clearance to turn.

        right_motor.reset_angle(0)
        right_motor.run_target(500, 495, then=Stop.BRAKE, wait=True)
        right_motor.stop()
        # Turn so we are facing the slide mission

        robot.straight(250)
        robot.stop()
        # Go forward until we reach the line for the slide mission.

        for x in range(0, 14):
            secondary_line_follow(0.1, 90, 1, 1)
            wait(20)
        # Line follow in 0.1 rotation bits so we activate and get both slide figures off the slide.

        robot.settings(100, 300, 100, 100)
        robot.straight(100)
        robot.stop()
        # Go forward a little more so we finish activating the slide mission.

        robot.settings(750, 275, 100, 100)
        robot.straight(650)
        robot.stop()
        robot.settings(400, 300, 250, 250)
        # Go forward home at a fast pace so we finish the run.

        ev3.speaker.play_file(SoundFile.MAGIC_WAND)
        # Play a magic sound so we know the run is done!
        ev3.screen.clear()
        # Clear the screen so we go back to the pick program screen.
        sliderun = False
        # Tell the while loop that we have finished the slide run.

    if Button.CENTER in ev3.buttons.pressed():
        
        ev3.screen.clear() # Clear the pick program screen
        ev3.screen.draw_text(15, 50, "Step+Treadmill") # Draw some text on the screen that says Step+Treadmill is running.
        ev3.screen.draw_text(45, 70, "is running.")

        robot.straight(465) #Fist align the robot facing East. Move forwad until the robot reaches the line to follow.

        robot.stop() # Stop any motor action so we can line follow.

        line_follow(0.5, 75, -0.6, 0.5) # Line follow slowly so we can precisely follow the line with black on left.
        
        line_follow(1.5, 175, -0.45, 0.5) # The robot follows the line at a more moderate pace.

        line_follow(1.3, 24, -0.7, 1.8) # The robot is following the line but its going at a slower speed because it is doing the step counter mission.

        robot.settings(500, 500, 500, 500) # Change the forward/backward/turn speed and accleration to be faster.

        robot.straight(-87) # The robot backwards so it would not bump in to the the step counter. 

        robot.turn(-15) # The robot is turning backwords so it can switch sides of the line to line follow.

        robot.straight(37) # Go straight a little so it will clear the step counter

        robot.turn(13) # Turn again in the opposite direction to now it is parallel to the step counter again.

        robot.stop() # Stop the motor action so we can line follow.

        robot.settings(400, 300, 250, 250) # Put speed settings back to normal

        line_follow(0.35, 100, 0.5, 1) # Line follow for a bit at a slow pace

        line_follow(1.55, 75, 0.45, 0) # Line follow moderately while it is going down an angle

        line_follow(1.15, 190, 0.4, 1) # Line follow quickly while the roobt is moving towards the step counter

        line_follow(0.5, 40, 0.4, 1) # Line follow slowly so we know for sure what position our robot is for future movements.

        medium_motor.reset_angle(0) # Reset the angle on the medium motor so we can accurately measure it in the next step.
        medium_motor.run_target(1500, -2500, then=Stop.BRAKE, wait="True") # The medium motor turn on and spins the tire to activate the treadmill until the pointer is past the dark green. This will wait until the medium motor is done reaching the target, and will actively hold the motor so there is no drag afterwards.
        medium_motor.stop()

        robot.straight(-110) # Move backwards afte doing the step counter to get in position for a turn towards the row machine.
        robot.stop()

        right_motor.reset_angle(0)
        right_motor.run_target(360, 145, then=Stop.BRAKE, wait=True) # Turn the robot with one motor to be the correct angle catch the row machine wheel.
        right_motor.stop()

        robot.straight(215) # Go forward with the one way gate being latched into the row machine wheel
        robot.stop()

        robot.settings(100, 100, 100, 100) # Make the robot speed slow for the next movement
        robot.straight(-210) # Go backwards to pull the row machine wheel out of the lage circle
        robot.stop()

        robot.straight(22.5) # Go straight a little so the center of the attached big tire on our robot is on center with the row machine wheel.
        robot.stop()

        right_motor.reset_angle(0)
        right_motor.run_target(50, 175, then=Stop.BRAKE, wait=True) # Turn the robot so the big wheel attachment will move the row machine wheel into the small circle.
        right_motor.stop()

        robot.settings(350, 200, 250, 250) # Change the robot speed to normal.

        robot.straight(-100) # Go backwards after completing the row machine mission.
        robot.stop()
        
        robot.turn(-53.25) # Turn to face the tire flip area.
        robot.stop()

        robot.straight(215) # Go straight to reach the tire flip area.
        robot.stop()

        robot.turn(-75) # Turn so the big wheel attachment will catch the blue tire and bring it to home so we can flip it there.
        robot.stop()

        robot.settings(1200, 250, 1000, 100) #Change robot speed for movements ahead

        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        while ((left_motor.angle())+(right_motor.angle()))/2 <= 1500:
            robot.drive(1200, 15)
        # Drive home at an angle instead of going forward and turning.

        robot.straight(1050)
        robot.stop()
        robot.settings(400, 300, 250, 250)
        # Go straight to home after the curve movement is done.

        ev3.speaker.play_file(SoundFile.LASER) # Play a fun sound to know that the program is done.
        ev3.screen.clear() # Clear the screen so it become the program select menu again.
        steptreadrun = False # Let the robot know that we have completed the Step Counter, Treadmill, Row Machine, and Tire Flip run.

    if Button.RIGHT in ev3.buttons.pressed():
        ev3.screen.clear()
        ev3.screen.draw_text(42, 50, "Basketball")
        ev3.screen.draw_text(42, 70, "is running.")
        # Set the screen to say the basketball is running
        
        robot.settings(450, 150, 400, 400)
        # Set the speed to normal

        robot.straight(160)
        robot.stop()
        # Go forward a little until the start of the line.

        line_follow(1.28, 100, 0.4, 1) # Line follow until the turn.

        right_motor.stop()
        right_motor.reset_angle(0)
        right_motor.run_target(360, 250, then=Stop.BRAKE, wait=True)
        right_motor.stop()
        # Use the right motor to do a turn around the angle in the line.

        line_follow(2.4, 150, 0.35, 0.5)
        line_follow(0.325, 40, 0.35, 0.5)
        # Line follow forward facing the boccia share, and once you get close, slow down so the boccia share block safely goes down.

        robot.straight(-100)
        robot.stop()
        # Go backwards after activating the boccia share in order to clear the next turn to face the basketball

        right_motor.reset_angle(0)
        right_motor.run_target(360, 175, then=Stop.BRAKE, wait=True)
        right_motor.stop()
        # Turn on the right motor to face the basketball.

        robot.settings(55, 250, 250, 250) # Make the speed slower for the next movement
        robot.straight(100) # Slowly go into the basketball mission
        robot.stop()

        medium_motor.reset_angle(0)
        medium_motor.run_target(1400, 1750, then=Stop.BRAKE, wait=True)
        medium_motor.stop()
        # Turn on the medium motor until you pass the first stopper

        robot.settings(100, 300, 250, 250) # Set speed slow
        robot.straight(-10)
        robot.stop()

        robot.straight(22.5)
        robot.stop()
        # Go forward and backwards minimal amounts in order to regain control needed to get over the next stopper.

        robot.settings(400, 300, 250, 250) # Set speed back to normal

        medium_motor.reset_angle(0)
        medium_motor.run_target(1400, 750, then=Stop.BRAKE, wait=True)
        medium_motor.stop()
        # Turn the medium motor to make the rack and pinon go over the second stopper

        medium_motor.reset_angle(0)
        medium_motor.run_target(1400, -250, then=Stop.BRAKE, wait=True)
        medium_motor.stop()
        # Turn the medium motor down a little bit after finishing the basketball mission in order to clear going back

        robot.straight(-150)
        robot.stop()
        # Go backwards from the basketball mission to clear the turn to get to the dance floor.

        robot.turn(85)
        robot.stop()
        # Turn the robot so the robot is facing the dance floor.

        robot.straight(275)
        robot.stop()
        # Go forward until the robot controller is on the dance floor

        while True:
            robot.settings(400, 300, 250, 100000)
            robot.turn(30)
            robot.turn(-15)
            robot.turn(15)
            robot.turn(-30)
            robot.stop()
            ev3.speaker.play_file(SoundFile.CHEERING)
        # Infinite dance sequence until the program is ended. The 4th value "100000" is so high because we do not want any accleration factor, as we want the robot to be jerky not smooth.
        # The dance sequence will turn the robot at essentially constant velocity

        ev3.screen.clear()
        basketballrun = False

    if Button.DOWN in ev3.buttons.pressed():
        medium_motor.run_target(1000, -2550, then=Stop.COAST, wait=True)
        medium_motor.reset_angle(0)

        