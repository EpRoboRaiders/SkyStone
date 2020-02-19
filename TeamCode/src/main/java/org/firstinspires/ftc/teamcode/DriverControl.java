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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

import static com.qualcomm.robotcore.util.Range.clip;

@TeleOp(name="Robot Driving Controls", group="Iterative Opmode")

// DriverControl is the main program used during the Driver Controlled and Endgame periods.
public class DriverControl extends OpMode {

    // Creates a motor based on the RobotTemplate class.
    RobotTemplate robot = new RobotTemplate();

    // The array driveMode stores all of the possible modes for driving our robot. At the start of
    // the program, the mode is set to 0, or "tank."
    enum Mode {
        TANK("Pure Tank Drive"),
        OMNI("Hybrid Tank/Mecanum Drive"),
        MECANUM("Pure Mecanum Drive");

        private String description;

        // getNext taken from (with modifications)
            // https://digitaljoel.nerd-herders.com/2011/04/05/get-the-next-value-in-a-java-enum/.
            public Mode getNext() {
            return this.ordinal() < Mode.values().length - 1
                    ? Mode.values()[this.ordinal() + 1]
                    : Mode.values()[0];
        }

        // Code taken from (with modifications)
        // https://stackoverflow.com/questions/15989316/
        // how-to-add-a-description-for-each-entry-of-enum/15989359#15989359
        private Mode(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }
    }

    Mode mode = Mode.OMNI;

    // final String[] driveMode = {"tank", "pov", "debug", "omni", "mecanum"};
    int currentMode = 0;

    // "[button]Pressed" booleans store the "current" state of a controller's button to compare it
    // to the actual current value as stated by gamepad[controller number].[button].
    boolean bPressed = false;
    boolean leftPressed = false;
    boolean rightPressed = false;
    boolean leftBumperPressed = false;

    // These constants ("final" variables) act to pin the terms "LOW," "MEDIUM," and "HIGH" speed
    // to concrete numbers for extending and retracting the stone lift. The speed is set to
    // "MEDIUM" at the beginning of the program.
    static final double LOW = .33;
    static final double MEDIUM = .67;
    static final double HIGH = 1;
    double liftSpeed = MEDIUM;

    // clampPressed simply exists to make sure the robot does not forcibly move into a position
    // in which the Capstone would be dropped onto the playing field as soon as the Driver
    // Controlled period begins. It locks the clamp into its initial position once the code starts,
    // and does not move until instructed, at which point it begins to behave according to the code.
    boolean clampPressed = false;

    // These doubles store the position of the "clamp" servos on the robot to do calculations on.
    double chassisPosition = 0;
    double mountedPosition = 1;

    @Override
    public void init() {

        // Initialize the robot and its motors.
        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Robot Mode:", "Initialized");
        telemetry.update();
    }

    // Runs repeatedly after the "init" button is pressed on the phone. Currently, it serves no
    // function.
    @Override
    public void init_loop() {}

    // Runs once after the "start" is pressed on the phone. Currently, it simply moves the T-bar
    // to an upright position.
    @Override
    public void start() {

        // Move the t-bar to an upright position.
        robot.stoneGrabber.setPosition(1);
    }

    // Runs repeatedly after the "start" button is pressed on the phone.

    public void loop() {

        // As seen below, run the code controlling the robot (separated into three distinct parts
        // for the sake of modularity).

        // chassisDrive: Contains the code used for running the "drive" motors on the robot.
        chassisDrive();

        // liftMovement: Contains the code used for controlling our stone lift's angle and
        // "position," whether in or out.
        liftMovement();

        // clampControl: Contains the code used for the rest of the servos and motors on the robot,
        // including the "stone clamps."
        clampControl();
    }

    // Runs once after the "stop" button is pressed on the phone. Currently, it serves no function.
    @Override
    public void stop() {}

    private void chassisDrive() {

        // Cycle through the driving modes when the "b" button on the first controller is pressed.
        if (gamepad1.b != bPressed) {

            if(!bPressed) {
                mode = mode.getNext();
            }
            bPressed = !bPressed;
        }

        // Run code depending on which drive mode is currently active:
        switch(mode) {
            case TANK: {

                // In "tank" drive mode,
                // the left joystick controls the speed of the left set of motors,
                // and the right joystick controls the right set.
                robot.leftFront.setPower(-gamepad1.left_stick_y * .75);
                robot.leftBack.setPower(-gamepad1.left_stick_y * .75);
                robot.rightFront.setPower(-gamepad1.right_stick_y * .75);
                robot.rightBack.setPower(-gamepad1.right_stick_y * .75);
                break;

            }
            case OMNI: {

                // Really funky. Strafes left or right using the left joystick
                // if the left joystick's "x" value is greater than "y;" runs like tank drive
                // otherwise.
                // This code was developed as a simple test by request of a coach, but the driver
                // responsible for moving the chassis actually liked the way that it worked!
                if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {
                    robot.leftFront.setPower(gamepad1.left_stick_x * .75);
                    robot.rightFront.setPower(-gamepad1.left_stick_x * .75);
                    robot.leftBack.setPower(-gamepad1.left_stick_x * .75);
                    robot.rightBack.setPower(gamepad1.left_stick_x * .75);
                }
                else {
                    robot.leftFront.setPower(-gamepad1.left_stick_y * .75);
                    robot.leftBack.setPower(-gamepad1.left_stick_y * .75);
                    robot.rightFront.setPower(-gamepad1.right_stick_y * .75);
                    robot.rightBack.setPower(-gamepad1.right_stick_y * .75);
                }
                break;

            }
            case MECANUM: {

                // Code taken from http://ftckey.com/programming/advanced-programming/. Also
                // funky; turns with the right joystick and moves/strafes with the left one.
                robot.leftFront.setPower(clip((-gamepad1.left_stick_y + gamepad1.left_stick_x
                        + gamepad1.right_stick_x), -1., 1) * .75);
                robot.leftBack.setPower(clip((-gamepad1.left_stick_y - gamepad1.left_stick_x
                        - gamepad1.right_stick_x), -1., 1) * .75);
                robot.rightFront.setPower(clip((-gamepad1.left_stick_y - gamepad1.left_stick_x
                        + gamepad1.right_stick_x), -1., 1) * .75);
                robot.rightBack.setPower(clip((-gamepad1.left_stick_y + gamepad1.left_stick_x
                        - gamepad1.right_stick_x), -1., 1) * .75);
                break;
            }
            default: {

                mode = Mode.TANK;

                robot.leftFront.setPower(-gamepad1.left_stick_y * .75);
                robot.leftBack.setPower(-gamepad1.left_stick_y * .75);
                robot.rightFront.setPower(-gamepad1.right_stick_y * .75);
                robot.rightBack.setPower(-gamepad1.right_stick_y * .75);
            }
        }

        // Display the current mode of the robot in Telemetry for reasons deemed obvious.
        for (int i=0; i<(Mode.values().length); i++){
            telemetry.addData("Mode ", Mode.values()[i].ordinal());
            telemetry.addData("Name", Mode.values()[i]);
        }

        // Display other information, including the position, speed, and mode of motors.
        telemetry.addData("Robot Mode:", mode.getDescription());
        telemetry.addData("Motor Position:", robot.leftFront.getCurrentPosition());

        telemetry.addData("left front motor: ", robot.leftFront.getPower());
        telemetry.addData("right front motor: ", robot.rightFront.getPower());
        telemetry.addData("left back motor: ", robot.leftBack.getPower());
        telemetry.addData("right back motor: ", robot.rightBack.getPower());
        telemetry.update();
    }

    private void liftMovement(){

        // Set the power of stoneLift (raising and lowering said lift) to 0 if both up and down
        // are pressed on the second controller.
        if (gamepad2.dpad_up && gamepad2.dpad_down) {
            robot.stoneLift.setPower(0);
        }
        // If the "up" arrow is pressed, raise the stoneLift at half power.
        else if(gamepad2.dpad_up) {
            robot.stoneLift.setPower(1);
        }
        // If the "down" arrow is pressed, lower the stoneLift at half power.
        else if(gamepad2.dpad_down){
            robot.stoneLift.setPower(-1);
        }
        // Set the power of stoneLift to 0 if no directional buttons are pressed.
        else{
            robot.stoneLift.setPower(0);
        }

        // Set the power of liftRotator to the "y" value of the right stick of the second
        // controller.
        robot.liftRotator.setPower(gamepad2.right_stick_y);


        // Set the foundation clamps to a "down" position if "x" is pressed; otherwise, set the
        // clamps to "up."

        if(gamepad2.x){
            robot.rightClamp.setPosition(0);
            robot.leftClamp.setPosition(1);
        }
        else if(gamepad2.b){
            robot.rightClamp.setPosition(.275);
            robot.leftClamp.setPosition(.775);
        }


    }

    private void clampControl() {

        // Because chassisGrabber is a servo, controlling it using speed does not apply. Instead,
        // slowly increase or decrease the position of said servo when the "left" or "right"
        // directional buttons are pressed on the second controller.
        if(gamepad2.dpad_right){
            chassisPosition += -.005;
        }
        else if(gamepad2.dpad_left) {
            chassisPosition += .005;
        }

        // Clip chassisPosition between 0 and 1, the minimum and maximum positions of a servo.
        clip(chassisPosition, 0, 1);

        // Toggle mountedPosition between .5 and 1 when the "left bumper" on the second controller
        // is pressed.
        if (gamepad2.left_bumper != leftBumperPressed) {

            if (!leftBumperPressed) {

                if (mountedPosition == 1) {
                    mountedPosition = .5;
                }
                else if (mountedPosition == .5) {
                    mountedPosition = 1;
                }
            }

            leftBumperPressed = !leftBumperPressed;
        }

        // Set the chassisGrabber and mountedGrabber servos to their respective positions.
        robot.chassisGrabber.setPosition(chassisPosition);
        robot.mountedGrabber.setPosition(mountedPosition);
    }
}