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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Blue Team- Start By Skystones", group="Linear Blue")

// SkystonesBlue is run during Autonomous when our robot is on the Blue team and is attempting
// to identify and move Skystones. Note that it is based on the AutonomousBase class,
// and uses its methods.

// Because the robot is situated "backwards" to detect Skystones, all comments should be considered
// to be written from the perspective of the robot (that is, moving away from the wall is going
// "backwards," and vice versa).
public class SkystonesBlue extends AutonomousBase {

    @Override
    public void runOpMode() {

        // Initialize the robot.
        initRobot();

        // Set the position of the stoneGrabber ("t-bar") to being vertical
        // to prepare to grab a Skystone.
        grabStone("up");

        // Drive backwards to prepare to align with a Stone.
        preciseDrive(DRIVE_SPEED, -20, -20,
                -20, -20, 10);

        // Strafe left to align the robot with the third Stone from the left.
        preciseDrive(DRIVE_SPEED, -1.5, 1.5,
                 1.5, -1.5, .5);

        // Drive backward to approach the first Stone.
        preciseDrive(DRIVE_SPEED, -7, -7,
                -7, -7, 10);

        // Scan the first Stone. If it is a Skystone:
        if(isSkystone()) {

            // Set the position of the stoneGrabber to horizontal to "grab onto" the Stone.
            grabStone("down");

            // Drive forwards to prepare to cross the Skybridge.
            preciseDrive(DRIVE_SPEED, 24, 24,
                    24, 24, 10);

            // Strafe right, across the Skybridge to deposit the Skystone.
            preciseDrive(.25, 60, -60,
                    -60, 60, 9.5);
        }
        // If the first Stone is not a Skystone:
        else {

            // Drive forward to back away from the first Stone.
            preciseDrive(DRIVE_SPEED, 3, 3,
                    3, 3, 10);

            // Strafe right, towards the second Stone.
            preciseDrive(DRIVE_SPEED, 8.25, -8.25,
                    -8.25, 8.25, 9.5);

            // Drive forward to approach the second Stone.
            preciseDrive(DRIVE_SPEED, -3.25, -3.25,
                    -3.25, -3.25, 10);

            // Scan the second Stone. If it is a Skystone:
            if(isSkystone()) {

                // Set the position of the stoneGrabber to horizontal to "grab onto" the Stone.
                grabStone("down");

                // Drive forwards to prepare to cross the Skybridge.
                preciseDrive(DRIVE_SPEED, 24, 24,
                        24, 24, 10);

                // Strafe right, across the Skybridge to deposit the Skystone.
                preciseDrive(.25, 52, -52,
                        -52, 52, 9.5);
            }
            // If the second Stone is not a Skystone (making the third automatically one):
            else {

                // Drive forward to back away from the second Stone.
                preciseDrive(DRIVE_SPEED, 3, 3,
                        3, 3, 10);

                // Strafe right, towards the third Stone.
                preciseDrive(DRIVE_SPEED, 7.5, -7.5,
                        -7.5, 7.5, 9.5);

                // Drive forward to approach the third Stone.
                preciseDrive(DRIVE_SPEED, -3.25, -3.25,
                        -3.25, -3.25, 10);

                // Set the position of the stoneGrabber to horizontal to "grab onto" the Stone.
                grabStone("down");

                // Drive forwards to prepare to cross the Skybridge.
                preciseDrive(DRIVE_SPEED, 24, 24,
                        24, 24, 10);

                // Strafe right, across the Skybridge to deposit the Skystone.
                preciseDrive(.25, 44, -44,
                        -44, 44, 9.5);
            }
        }

        // Let go of the Skystone.
        grabStone("up");

        // Strafe left, back under the Skybridge to park.
        preciseDrive(DRIVE_SPEED, -12, 12,
                12, -12, 9.5);

        // If parking close to the Neutral Skybridge, move backward; otherwise, drive forwards
        // into the wall.
        if(parking_location == "Bridge") {

            preciseDrive(DRIVE_SPEED, -15, -15,
                    -15, -15, 10);
        }
        else {

            preciseDrive(DRIVE_SPEED, 21, 21,
                    21, 21, 10);

            // Strafe left again to account for the angle of the robot from the friction of the
            // Skystone against the playing field.
            preciseDrive(DRIVE_SPEED, -8, 8,
                    8, -8, 9.5);
        }
    }
}