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

@TeleOp(name="Robot Driving Controls", group="Iterative Opmode")

public class DriverControl extends OpMode {

    // Creates a motor based on the RobotTemplate class.

    RobotTemplate robot = new RobotTemplate();

    // The array driveMode stores all of the possible modes for driving our robot. At the start of
    // the program, the mode is set to 0, or "tank."

    final String[] driveMode = {"tank", "pov", "debug"};

    int currentMode = 0;

    // "[button]Pressed" booleans store the "current" state of a controller's button to compare it
    // to the actual current value as stated by gamepad[controller number].[button].

    boolean bPressed = false;
    boolean leftPressed = false;
    boolean rightPressed = false;
    boolean leftBumperPressed = false;
    boolean rightBumperPressed = true;

    // These constants ("final" variables) act to pin the terms "LOW," "MEDIUM," and "HIGH" speed
    // to concrete numbers for extending and retracting the stone lift. The speed is set to
    // "MEDIUM" at the beginning of the program.

    static final double LOW = .33;
    static final double MEDIUM = .67;
    static final double HIGH = 1;

    double liftSpeed = MEDIUM;


    double chassisPosition = 1;
    double mountedPosition = 0;

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

        // Cycle through the robot drive modes when the B button is released and subsequently
        // pressed.

        chassisDrive();

        liftMovement();

        clampControl();

        // Display the current mode of the robot in Telemetry for reasons deemed obvious.

        telemetry.update();
        telemetry.addData("Robot Mode:", driveMode[currentMode]);

    }

    // Runs once after the "stop" button is pressed on the phone. Currently, it serves no function.

    @Override
    public void stop() {}

    private void chassisDrive() {

        if (gamepad1.b != bPressed) {

            if(!bPressed) {
                currentMode = (currentMode + 1) % driveMode.length;
            }

            bPressed = !bPressed;
        }

        if(driveMode[currentMode] == "tank") {

            // In "tank" drive mode, the left joystick controls the speed of the left set of motors,
            // and the right joystick controls the right set.

            robot.leftFront.setPower(-gamepad1.left_stick_y);
            robot.leftBack.setPower(-gamepad1.left_stick_y);
            robot.rightFront.setPower(-gamepad1.right_stick_y);
            robot.rightBack.setPower(-gamepad1.right_stick_y);
        }
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
    }

    private void liftMovement(){
        
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

        if(gamepad2.dpad_up){
            robot.stoneLift.setPower(liftSpeed);
        }
        else if(gamepad2.dpad_down){
            robot.stoneLift.setPower(-liftSpeed);
        }
        else{
            robot.stoneLift.setPower(0);
        }

        robot.liftRotator.setPower(-gamepad2.right_stick_y);
    }

    private void clampControl() {

        if (gamepad2.left_bumper != leftBumperPressed) {

            if(!leftBumperPressed) {

                if(chassisPosition == 1) {
                    chassisPosition = .5;
                }
                else if(chassisPosition == .5) {
                    chassisPosition = 1;
                }
            }

            leftBumperPressed = !leftBumperPressed;
        }

        if (gamepad2.right_bumper != rightBumperPressed) {

            if(!rightBumperPressed) {

                if(mountedPosition == 0) {
                    mountedPosition = 1;
                }
                else if(mountedPosition == 1) {
                    mountedPosition = 0;
                }
            }

            rightBumperPressed = !rightBumperPressed;
        }

        robot.leftClamp.setPower(gamepad2.left_stick_y);
        robot.rightClamp.setPower(gamepad2.left_stick_y);

        robot.chassisGrabber.setPosition(chassisPosition);
        robot.mountedGrabber.setPosition(mountedPosition);

    }
}
