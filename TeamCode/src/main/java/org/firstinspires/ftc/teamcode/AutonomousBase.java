/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// AutonomousBase is a LinearOpMode program that contains all of the methods used in Autonomous
// programs based on this one.
public class AutonomousBase extends LinearOpMode {

    // Constants for speed and encoders used during Autonomous for the sake of cleanliness.
    // After trial and error, it was determined that 120 encoder counts is roughly equal to 1 inch.
    // The actual amount is closer to 119 counts, but 120 is a nicer number for use in calculations
    // and has no reason to be changed.
    static final int     COUNTS_PER_INCH    = 120;
    static final double  SLOW_DRIVE_SPEED   = .5;
    static final double  STONE_BACKUP_SPEED = .075;

    public String team;
    public String starting_location;
    public String parking_location;
    public boolean autonomous;

    // Create a timer named "runtime" based on the ElapsedTime class.
    ElapsedTime     runtime = new ElapsedTime();

    // Create a robot named "robot" based on the RobotTemplate class.
    RobotTemplate robot = new RobotTemplate();


    // Template for the "code block" that is ran in all of the Autonomous programs based on this
    // one.
    @Override
    public void runOpMode() {}

    // preciseDrive is a function used to control the robot using encoders, based on the
    // PushbotAutoDriveByEncoderLinear class. Put simply, it takes a distance, direction, and speed
    // for the four drive motors on the robot (as well as a timeout), and uses encoders to drive
    // the robot a specific amount. Note that because our robot uses Mecanum wheels, values that
    // would otherwise be useless can be "passed in" to the function in order to strafe during
    // Autonomous.
    public void preciseDrive(double speed, 
                             double leftFrontInches, double rightFrontInches, 
                             double leftBackInches, double rightBackInches,
                             double timeoutS) {

        // Initialize integers to be defined as the "targets" for the encoders of the motors to
        // reach.
        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        // If the opMode is currently active:
        if (opModeIsActive()) {

            // Determine the target position of the motors, based on the current position of
            // said motors.
            leftFrontTarget = robot.leftFront.getCurrentPosition()
                    + (int)(leftFrontInches * COUNTS_PER_INCH);
            rightFrontTarget = robot.rightFront.getCurrentPosition()
                    + (int)(rightFrontInches * COUNTS_PER_INCH);
            leftBackTarget = robot.leftBack.getCurrentPosition()
                    + (int)(leftBackInches * COUNTS_PER_INCH);
            rightBackTarget = robot.rightBack.getCurrentPosition()
                    + (int)(rightBackInches * COUNTS_PER_INCH);

            // Set the targets of the four motors to their respective targets.
            robot.leftFront.setTargetPosition(leftFrontTarget);
            robot.rightFront.setTargetPosition(rightFrontTarget);
            robot.leftBack.setTargetPosition(leftBackTarget);
            robot.rightBack.setTargetPosition(rightBackTarget);

            // Turn On RUN_TO_POSITION.
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and turn the motors on to their respective speeds.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));

            // The following is directly from PushbotAutoDriveByEncoder_Linear:
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                            && robot.leftFront.isBusy() && robot.rightFront.isBusy()
                            && robot.leftBack.isBusy() && robot.rightBack.isBusy()) {}

            // Stop all of the motors after all of the moves have completed.
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Wait for .15 seconds for the motors to stop moving.
            pause(.15);
        }
    }

    // Control the "clamps" on the robot to grab the Foundation during Autonomous.
    public void clampSet(String clampPosition) {

        // Reset the timer on the robot.
        runtime.reset();

        // Because the position of the clamps is controlled by "360" servos, they have to be
        // controlled like motors; i.e. ran like a motor, in this case for 8 seconds. Turn them on
        // based on the argument clampPosition when the function is called.
        if(clampPosition == "up") {

            robot.leftClamp.setPower(-1);
            robot.rightClamp.setPower(-1);
        }
        else if (clampPosition == "down") {

            robot.leftClamp.setPower(1);
            robot.rightClamp.setPower(1);
        }

        // Wait for 8 seconds.
        pause(8);


        // Set the power of the servos to 0 (or in the case of leftClamp with a drift, -.3970)
        robot.leftClamp.setPower(-.3970);
        robot.rightClamp.setPower(0);
    }

    // pause is a (probably unnecessary) function that uses the timer "runtime" to wait for a
    // specific amount of time.
    public void pause(double seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds) {}
    }

    // Outdated (though still used) method that runs the drive motors for a specific distance
    // using time instead of distance. This is used in the "Foundation side" Autonomous programs,
    // before encoders were implemented into the robot. Note that because our robot uses Mecanum
    // wheels, values that would otherwise be useless can be "passed in" to the function in order
    // to strafe during Autonomous.
    public void timeDrive(double speed, double leftFrontDirection, double rightFrontDirection,
                          double leftBackDirection, double rightBackDirection, double time) {

        robot.leftFront.setPower(leftFrontDirection);
        robot.rightFront.setPower(rightFrontDirection);
        robot.leftBack.setPower(leftBackDirection);
        robot.rightBack.setPower(rightBackDirection);

        // Reset the "timer" in the code, and wait until the timer reaches the indicated number
        // of seconds.
        runtime.reset();
        while(runtime.seconds() < time) {}

        // Set the power of all of the drive motors to 0.
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        // Pause for a short amount of time to make sure the motors are completely stopped.
        pause(.25);
    }

    // Method that returns whether a Stone that the robot is "scanning" is a Skystone.
    public boolean isSkystone() {

        // Wait for one second to make sure the color sensor is in a "stable" position.
        pause(1);

        // Assign the "red" value of the color sensor to a variable for later use.
        int Red = robot.colorSensor.red();

        // Display the "color" value returned by the color sensor in Telemetry, for debugging
        // purposes and the sake of the drivers.
        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("Green", robot.colorSensor.green());
        telemetry.addData("Blue ", robot.colorSensor.blue());
        telemetry.update();

        // The "red" value of a Skystone is generally less than 120; check to see if this is true.
        // Funny thing: This line initially read "return !(Red >= 120)" until it was "discovered"
        // that "not greater than or equal to" is an equivalent statement to "less than."
        return Red < 120;
    }

    // Because the statements in initRobot were being called at the beginning of every Autonomous
    // program, this method runs all of them for the sake of cleanliness.
    public void initRobot() {

        // Initialize the already-defined "robot" based on the RobotTemplate class.
        robot.init(hardwareMap);

        determineAutonomous();

        // Display a statement in Telemetry indicating that the robot is ready to be started.

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team", team);
        telemetry.addData("Starting Location", starting_location);
        telemetry.addData("Parking Location", parking_location);
        telemetry.addData("Autonomous", autonomous);
        telemetry.update();

        // Wait until the "start" button is pressed.
        waitForStart();

        // Because of "drift" in the 360 servo used as the leftClamp, the power of said clamp
        // has to be set to a constant negative value. The only major downside to this fix
        // is that the servo moves more slowly in the opposite direction, as it is capped at
        // effectively 60% power.
        robot.leftClamp.setPower(-.3970);

    }
    public void determineAutonomous() {

        telemetry.addData("Press The Button of the Color of Your Alliance", "");
        telemetry.addData("Use Controller 1 (Press Start + A)", "");
        telemetry.update();

        while(!(gamepad1.b || gamepad1.x)) {}
        if(gamepad1.x){
            team = "Blue";
        }
        else if(gamepad1.b){
            team = "Red";
        }
        while(gamepad1.b || gamepad1.x) {}

        telemetry.addData("Team", team);
        telemetry.addData("Press Up if Parking by Far Side of Bridge", "");
        telemetry.addData("Press Down if Parking by Wall", "");
        telemetry.addData("Use Controller 1 (Press Start + A)", "");
        telemetry.update();

        while(!(gamepad1.dpad_up || gamepad1.dpad_down)) {}
        if(gamepad1.dpad_up){
             parking_location = "Bridge";
        }
        else if(gamepad1.dpad_down){
            parking_location = "Wall";
        }

        while(gamepad1.dpad_up || gamepad1.dpad_down) {}

        telemetry.addData("Team", team);
        telemetry.addData("Parking Location", parking_location);
        telemetry.addData("Press The Direction of the Side of the " +
                "Bridge you Are Starting On", "");
        telemetry.addData("Use Controller 1 (Press Start + A)", "");
        telemetry.update();

        while(!(gamepad1.dpad_left || gamepad1.dpad_right)) {}

        if(gamepad1.dpad_left){
            if(team == "Red") {
                starting_location = "Loading Zone";
            }
            else if(team == "Blue") {
                starting_location = "Building Zone";
            }
        }
        else if(gamepad1.dpad_right){

            if(team == "Red") {
                starting_location = "Building Zone";
            }
            else if(team == "Blue") {
                starting_location = "Loading Zone";
            }
        }

        while(gamepad1.dpad_left || gamepad1.dpad_right) {}

        telemetry.addData("Team", team);
        telemetry.addData("Starting Location", starting_location);
        telemetry.addData("Parking Location", parking_location);
        telemetry.addData("Press X if Doing Autonomous ", "");
        telemetry.addData("Press B if Just Parking", "");
        telemetry.addData("Use Controller 1 (Press Start + A)", "");
        telemetry.update();

        while(!(gamepad1.x || gamepad1.b)) {}

        if(gamepad1.x) {
            autonomous = true;
        }
        else if(gamepad1.b){
            autonomous = false;
        }

        while(gamepad1.x || gamepad1.b) {}

        telemetry.addData("Team", team);
        telemetry.addData("Starting Location", starting_location);
        telemetry.addData("Parking Location", parking_location);
        telemetry.addData("Autonomous", autonomous);
        telemetry.addData("Press X to Confirm", "");
        telemetry.addData("Press B to Start Over", "");
        telemetry.addData("Use Controller 1 (Press Start + A)", "");
        telemetry.update();

        while(!(gamepad1.x || gamepad1.b)) {}

        if(gamepad1.x) {}
        else if(gamepad1.b){
            determineAutonomous();
        }

        while(gamepad1.x || gamepad1.b) {}
    }
}