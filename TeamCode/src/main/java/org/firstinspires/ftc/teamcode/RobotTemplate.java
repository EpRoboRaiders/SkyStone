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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

// The descriptions of the above bits of code would be redundant if listed in every program; as
// a result, they will only be listed in RobotTemplate.

// This is not an OpMode, and simply acts as an external "constructor" for the motors and servos
// on the robot. This is a direct copy of HardwarePushbot, and should be treated as such.

public class RobotTemplate {

    // Listed below are the declarations of all of the motors, servos, and sensors used on our
    // robot, along with a description of their function.

    // Motors that "drive" the robot; that is, directly control the motion of the chassis and its
    // attachments. The concepts of direction are relative to the "front" of the robot being the
    // side at which the "claw" portion of our Stone lift (as explained below) lays when at rest.
    public DcMotor      leftFront      = null;
    public DcMotor      rightFront     = null;
    public DcMotor      leftBack       = null;
    public DcMotor      rightBack      = null;

    // Motor that raises and lowers the "lift" responsible for grabbing and delivering Stones
    // during play.
    public DcMotor      stoneLift      = null;

    // Motor that changes the angle of said lift from the ground. Used to "lift" Stones when the
    // robot is latched onto them.
    public DcMotor      liftRotator   = null;

    // Servos located on the front of the robot that "latch onto" the Foundation when we are trying
    // to move it, whether in Autonomous or Endgame.
    public Servo      leftClamp      = null;
    public Servo      rightClamp     = null;

    // Servo used to "rotate" the Stone clamp itself (NOT the entire lift; the liftRotator serves
    // this purpose) during the Driver Controlled period.
    public Servo        chassisGrabber = null;

    // Servo used to "clamp" and "unclamp" from Stones during the Driver Controlled period; toggles
    // between two positions.
    public Servo        mountedGrabber = null;

    // Color sensor used to detect the position of Skystones during Autonomous. Located on the
    // front of the robot.
    public ColorSensor  colorSensor = null;

    // A "t bar" connected to a servo, used to grab onto Skystones easily during Autonomous.
    // Located on the front of the robot.
    public Servo        stoneGrabber = null;

    // External constructor used inside of this constructor. Meta!
    HardwareMap hwMap          =  null;

    // Called to construct the robot template in another OpMode.
    public RobotTemplate(){}

    // Initializes the defined motors and servos.
    public void init(HardwareMap ahwMap) {

        // "Saves" the hardware map used to find motors and servos.
        hwMap = ahwMap;

        // Maps the motors found at certain ports on the robot "hub" on our robot to names used
        // to define them in the code. Going through their purposes is redundant, as said
        // explanation is done above; however, they are still separated by "type" for the sake
        // of cleanliness.

        leftFront       = hwMap.get(DcMotor.class, "left_front");
        rightFront      = hwMap.get(DcMotor.class, "right_front");
        leftBack        = hwMap.get(DcMotor.class, "left_back");
        rightBack       = hwMap.get(DcMotor.class, "right_back");

        stoneLift       = hwMap.get(DcMotor.class, "stone_lift");
        liftRotator     = hwMap.get(DcMotor.class, "lift_rotator");

        leftClamp       = hwMap.get(Servo.class, "left_clamp");
        rightClamp      = hwMap.get(Servo.class, "right_clamp");

        chassisGrabber  = hwMap.get(Servo.class, "chassis_grabber");
        mountedGrabber  = hwMap.get(Servo.class, "mounted_grabber");

        chassisGrabber  = hwMap.get(Servo.class, "chassis_grabber");
        stoneGrabber    = hwMap.get(Servo.class, "stone_grabber");

        colorSensor     = hwMap.get(ColorSensor.class, "color_sensor");

        // Set all motors to zero power so they don't run by mistake when initialized.
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        stoneLift.setPower(0);
        liftRotator.setPower(0);

        // Because of the orientation of the motors on our robot, the ones on the right side
        // have to be "flipped" in terms of direction. Sure, every instruction written with
        // these motors in mind could be multiplied by negative one, but that would be tedious
        // and annoying, wouldn't it?
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Make sure that the stoneLift "brakes" to combat gravity.
        stoneLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Function for moving all motors the same speed.
    public void motorsSpeed(double speed) {
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
    }
 }