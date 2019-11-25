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

@TeleOp(name="Robot Driving Controls", group="Iterative Opmode")

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
        // https://stackoverflow.com/questions/15989316/how-to-add-a-description-for-each-entry-of-enum/15989359#15989359

        private Mode(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }
    }

    Mode mode = Mode.TANK;

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
    double mountedPosition = .3;

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

    // Runs once after the "start" is pressed on the phone. Currently, it serves no function.

    @Override
    public void start() {}

    // Runs repeatedly after the "start" button is pressed on the phone.

    public void loop() {

        // As seen below, run the code controlling the robot (separated into three distinct parts
        // for the sake of modularity).

        // chassisDrive: Contains the code used for running the "drive" motors on the robot.

        chassisDrive();

        // liftMovement: Contains the code used for controlling our stone lift's angle and
        // "position," whether in or out.

        liftMovement();

        //

        clampControl();

    }

    // Runs once after the "stop" button is pressed on the phone. Currently, it serves no function.

    @Override
    public void stop() {}



    private void chassisDrive() {

        if (gamepad1.b != bPressed) {

            if(!bPressed) {
                mode = mode.getNext();
            }
            bPressed = !bPressed;
        }

        switch(mode) {
            case TANK: {

                // In "tank" drive mode,
                // the left joystick controls the speed of the left set of motors,
                // and the right joystick controls the right set.

                robot.leftFront.setPower(-gamepad1.left_stick_y);
                robot.leftBack.setPower(-gamepad1.left_stick_y);
                robot.rightFront.setPower(-gamepad1.right_stick_y);
                robot.rightBack.setPower(-gamepad1.right_stick_y);
                break;

            }
            case OMNI: {

                // Really funky. Strafes left or right using the left joystick
                // if the left joystick's "x" value is greater than "y;" runs like tank drive
                // otherwise.

                // This code was developed as a simple test by request of a coach, but the driver
                // responsible for moving the chassis actually liked the way that it worked!

                if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {
                    robot.leftFront.setPower(gamepad1.left_stick_x);
                    robot.rightFront.setPower(-gamepad1.left_stick_x);
                    robot.leftBack.setPower(-gamepad1.left_stick_x);
                    robot.rightBack.setPower(gamepad1.left_stick_x);
                }
                else {
                    robot.leftFront.setPower(-gamepad1.left_stick_y);
                    robot.leftBack.setPower(-gamepad1.left_stick_y);
                    robot.rightFront.setPower(-gamepad1.right_stick_y);
                    robot.rightBack.setPower(-gamepad1.right_stick_y);
                }
                break;

            }
            case MECANUM: {

                // Code taken from http://ftckey.com/programming/advanced-programming/. Also
                // funky; turns with the right joystick and moves/strafes with the left one.

                robot.leftFront.setPower(Range.clip((-gamepad1.left_stick_y + gamepad1.left_stick_x
                        + gamepad1.right_stick_x), -1., 1));
                robot.leftBack.setPower(Range.clip((-gamepad1.left_stick_y - gamepad1.left_stick_x
                        - gamepad1.right_stick_x), -1., 1));
                robot.rightFront.setPower(Range.clip((-gamepad1.left_stick_y - gamepad1.left_stick_x
                        + gamepad1.right_stick_x), -1., 1));
                robot.rightBack.setPower(Range.clip((-gamepad1.left_stick_y + gamepad1.left_stick_x
                        - gamepad1.right_stick_x), -1., 1));
                break;

            }
            default: {

                mode = Mode.TANK;

                robot.leftFront.setPower(-gamepad1.left_stick_y);
                robot.leftBack.setPower(-gamepad1.left_stick_y);
                robot.rightFront.setPower(-gamepad1.right_stick_y);
                robot.rightBack.setPower(-gamepad1.right_stick_y);
            }
        }


        // Defunct drive modes that no member of the team preferred driving in. Descriptions of
        // them can be found inside of their respective code.
        /*
        else if(driveMode[currentMode] == "pov") {

            // In "pov' drive mode, the left joystick controls the speed of the robot, and the
            // right joystick controls the heading. Refer to PushbotTeleopPOV_Linear for specifics
            // on how it functions.

            robot.leftFront.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            robot.leftBack.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            robot.rightFront.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            robot.rightBack.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
        }
        else if(driveMode[currentMode] == "debug") {

            // In the "debug" drive mode, each stick on each controller controls its own motor with
            // its "x" value. It is HIGHLY recommended to remove this before competition use.

            robot.leftFront.setPower(gamepad1.right_stick_x);
            robot.rightFront.setPower(gamepad2.left_stick_x);
            robot.leftBack.setPower(gamepad1.left_stick_x);
            robot.rightBack.setPower(gamepad2.right_stick_x);

        }
         */
        // Display the current mode of the robot in Telemetry for reasons deemed obvious.


        telemetry.addData("Robot Mode:", mode.getDescription());
        telemetry.addData("Motor Position:", robot.leftFront.getCurrentPosition());

        for (int i=0; i<(Mode.values().length); i++){
            telemetry.addData("Mode ", Mode.values()[i].ordinal());
            telemetry.addData("Name", Mode.values()[i]);
        }
        telemetry.update();

    }

    private void liftMovement(){

        /*

        if (gamepad2.dpad_left != leftPressed) {

            if(!leftPressed) {
                if (liftSpeed == MEDIUM) {
                    liftSpeed = LOW;
                }
                else if (liftSpeed == HIGH) {
                    liftSpeed = MEDIUM;
                }
            }

            leftPressed = !leftPressed;
        }


        if (gamepad2.dpad_right != rightPressed) {

            if(!rightPressed) {
                if (liftSpeed == LOW) {
                    liftSpeed = MEDIUM;
                }
                else if (liftSpeed == MEDIUM) {
                    liftSpeed = HIGH;
                }
            }

            rightPressed = !rightPressed;
        }
        */
        if (gamepad2.dpad_up && gamepad2.dpad_down) {
            robot.stoneLift.setPower(0);
        }
        else if(gamepad2.dpad_up){
            robot.stoneLift.setPower(1);
        }
        else if(gamepad2.dpad_down){
            robot.stoneLift.setPower(-1);
        }
        else{
            robot.stoneLift.setPower(0);
        }

        robot.liftRotator.setPower(gamepad2.right_stick_y);
    }

    private void clampControl() {
        // Changes the position of the "chassis" servo from .5 to .25, the range that it would
        // realistically need to be for competition.

        if (gamepad2.left_bumper != leftBumperPressed) {

            if (!leftBumperPressed) {

                if (chassisPosition == 0) {
                    chassisPosition = .25;
                } else if (chassisPosition == .25) {
                    chassisPosition = 0;
                }
            }

            leftBumperPressed = !leftBumperPressed;
        }



        // *Used* to set the position of the "mounted" servo 0 or 1. Defunct in that more precision
        // is needed to control the robot properly.

        /*
        if (gamepad2.right_bumper != rightBumperPressed) {

            if(!rightBumperPressed) {

                if(mountedPosition == 1) {
                    mountedPosition = 0;
                }
                else if(mountedPosition == 0) {
                    mountedPosition = 1;
                }
            }

            rightBumperPressed = !rightBumperPressed;
        }
         */

        // robot.leftClamp.setPower(gamepad2.left_stick_y);
        // robot.rightClamp.setPower(gamepad2.left_stick_y);

        if (!gamepad2.dpad_left || !gamepad2.dpad_right){
            clampPressed = true;
        }

        // "Default" the position of the mounted servo to .6, as opposed to .5, for the clamp to
        // rest correctly when placed on top of a stone.
        if(gamepad2.dpad_right){
            mountedPosition += .005;
        }
        else if(gamepad2.dpad_left) {
            mountedPosition += -.005;
        }

        if (gamepad2.right_bumper) {
            mountedPosition = .3;
        }

        /*

        mountedPosition = ((gamepad2.left_stick_y) / 2) + .5;
        if (mountedPosition == .5) {
            mountedPosition = .35;
        }
         */

        // Extension of the clampPressed code above.

        robot.chassisGrabber.setPosition(chassisPosition); // THIS WORKS FINE DO NOT CHANGE!

        if(clampPressed) {
            robot.mountedGrabber.setPosition(mountedPosition);
        }


    }
}
