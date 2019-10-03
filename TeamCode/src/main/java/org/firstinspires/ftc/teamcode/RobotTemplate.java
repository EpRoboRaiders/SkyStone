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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/* This is not an OpMode, and simply acts as an external "constructor" for the motors and servos
on the robot. This is a direct copy of HardwarePushbot, and should be treated as such.
 */

public class RobotTemplate {
    
    // Defines the motors, servos, and speeds used in the robot.
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftBack   = null;
    public DcMotor  rightBack  = null;

    /*
    These motors have names that are "unhelpful" to describing
    the functions of the motors on our personal robot. As a result, we use our own custom motor
    names; these are left for future reference.
     */
    
    /*
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftArm     = null;
     */
    
    /*
    Because our robot currently does not use servos, there is no need to define ones that do
    not exist. These initializations are left for future reference.
     */

    /*
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
     */
    
    /*
    These variables store values related to the servos that, again, do not exist. Also again,
    they are left for reference.
     */
    
    /*
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
     */

    // External constructors used inside of this constructor. Meta!
    HardwareMap hardwareMap          =  null;
    
    /* Commented out because the initialization of a timer has little use, in that the timer is 
    never referred to in this class. It is kept for reference, or if it ends up being necessary
    for other timers (or code) to function. */
    // private ElapsedTime period = new ElapsedTime();
    
    // Called to constsruct the robot template in another OpMode.
    public RobotTemplate(){}
    
    // Initializes the defined motors and servos.
    public void init(HardwareMap ahwMap) {
        
        // "Saves" the hardware map used to find motors and servos.
        hardwareMap = ahwMap;
        
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        // Unused motor mappings. Left for reference.

        /*
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftArm    = hwMap.get(DcMotor.class, "left_arm");
         */

        // Unused motor configuration. Left for reference.

        /*
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
         */

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Unused motor configuration. Left for reference.

        /*
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftArm.setPower(0);
         */

        // Unused servo mappings. Left for reference.

        /*
        leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
         */

        // Unused servo initialization. Left for reference.

        /*
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
         */
    }
 }

