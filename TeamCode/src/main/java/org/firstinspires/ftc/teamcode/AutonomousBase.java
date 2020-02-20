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

    // The CPM of our new motors was determined to be 23 after a WILDLY incorrect estimation of 620.
    static final double     COUNTS_PER_INCH         = 23;

    // Since .1 speed is used for most driving implementatinos during Autonomous, simply define
    // it as a constant for easy modification if necessary.
    static final double  DRIVE_SPEED   = .1;

    public String parking_location;

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
            telemetry.addData("LeftFront Encoder Start", robot.leftFront.getCurrentPosition());
            telemetry.addData("LeftFront Encoder Desired : ", robot.leftFront.getTargetPosition());
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                            && robot.leftFront.isBusy() && robot.rightFront.isBusy()
                            && robot.leftBack.isBusy() && robot.rightBack.isBusy()) {}

            // Stop all of the motors after all of the moves have completed.
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            telemetry.addData("LeftFront Encoder Current:", robot.leftFront.getCurrentPosition());
            telemetry.update();

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Wait for .15 seconds for the motors to stop moving.
            pause(.15);
        }
    }

    // Control the "clamps" the robot to grab the Foundation during Autonomous. The position
    // the clamps are supposed to move to is passed in as an argument.
    public void clampSet(String clampPosition) {

        // Note that the servos move in "opposite" directions, though for the same amount of time;
        // this is because said servos are mounted on opposite sides of the robot.
        if(clampPosition == "up") {

            robot.rightClamp.setPosition(.275);
            robot.leftClamp.setPosition(.775);
        }
        else if (clampPosition == "down") {

            robot.rightClamp.setPosition(0);
            robot.leftClamp.setPosition(1);
        }

        // Wait for one second for the servos to move to *actually* attach or detach from the
        // Foundation before the robot starts moving.
        sleep(1000);
    }

    // pause is a (probably unnecessary) function that uses the timer "runtime" to wait for a
    // specific amount of time.
    public void pause(double seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds) {}
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

        // The "red" value of a Skystone is generally less than 200; check to see if this is true.
        // Funny thing: This line initially read "return !(Red >= 200)" until it was "discovered"
        // that "not greater than or equal to" is an equivalent statement to "less than."
        return Red < 200;
    }

    // Because the statements in initRobot were being called at the beginning of every Autonomous
    // program, this method runs all of them for the sake of cleanliness.
    public void initRobot() {

        // Initialize the already-defined "robot" based on the RobotTemplate class.
        robot.init(hardwareMap);

        // Run determineParking, as outlined below.
        determineParking();

        // Display a statement in Telemetry indicating that the robot is ready to be started.

        telemetry.addData("Parking Location", parking_location);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait until the "start" button is pressed.
        waitForStart();
    }

    // determineParking is a version of an old function made to "cut down" on the number of
    // Autonomous programs. After realizing that controlling literally every parameter of Autonomous
    // was infeasible, it was simply cut down to determining where the robot would park at the
    // end of said Autonomous.
    public void determineParking() {

        // Display information to the user for ease of use.
        telemetry.addData("Press ↑", "Parking by the Far Side of the Bridge");
        telemetry.addData("Press ↓", "Parking by the Wall");
        telemetry.addData("Use", "Controller 1");
        telemetry.update();

        // Wait until either the "up" or "down" buttons are pressed before continuing.
        while(!(gamepad1.dpad_up || gamepad1.dpad_down)) {}

        // Set the "position" to park during Autonomous accordingly, with "up" corresponding to
        // the Alliance-specific bridge closest to the Neutral bridge, and "down" corresponding
        // to the wall.

        if(gamepad1.dpad_up){
            parking_location = "Bridge";
        }
        else if(gamepad1.dpad_down){
            parking_location = "Wall";
        }

        // Display information to the user for ease of use.
        telemetry.addData("Parking Location", parking_location);
        telemetry.addData("Press X", "Confirm");
        telemetry.addData("Press B", "Start Over");
        telemetry.addData("Use", "Controller 1");
        telemetry.update();

        // Wait until either the "x" or "b" buttons are pressed before continuing.
        while(!(gamepad1.x || gamepad1.b)) {}

        // If "x" is pressed, exit the function; otherwise, recurse it if the operator needs to
        // start over.
        if(gamepad1.x) {}
        else if(gamepad1.b){
            determineParking();
        }
    }

    // Controls the "t-bar" servo on the robot during Autonomous. The position the bar
    // is supposed to move to is passed in as an argument.
    public void grabStone(String position) {

        if(position == "up") {

            robot.stoneGrabber.setPosition(1);
        }
        else if(position == "down") {

            robot.stoneGrabber.setPosition(.5);

            // Only delay if grabbing onto a stone, as dragging the stone slightly when "letting go"
            // from it is acceptable.
            sleep(1000);
        }
    }
}