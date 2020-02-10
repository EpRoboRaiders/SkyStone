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

@Disabled
@Autonomous(name="Red Team- Start By Skystones", group="Linear Opmode")

// SkystonesRed is run during Autonomous when our robot is on the Red team and is attempting
// to identify and move Skystones. Note that it is based on the AutonmousBase class,
// and uses its methods.
public class SkystonesRed extends AutonomousBase {

    @Override
    public void runOpMode() {

        initRobot();

        // Set the position of chassisGrabber (which the stoneGrabber is mounted on) to 1 for
        // consistency.
        robot.chassisGrabber.setPosition(1);

        // Set the position of the stoneGrabber ("t-bar") to being vertical
        // to prepare to grab a Skystone.
        robot.stoneGrabber.setPosition(.5);


        // Drive forward to prepare to align with a Stone.
        preciseDrive(SLOW_DRIVE_SPEED, 25, 25,
                25, 25, 3);

        // Drive left to align the robot with the third Stone from the right.
        preciseDrive(SLOW_DRIVE_SPEED, -8.5, 8.5,
                8.5, -8.5, 3);

        // Drive forward to approach the first Stone.
        preciseDrive(SLOW_DRIVE_SPEED, 5, 5,
                5, 5, 5);

        // At this point, the robot's color sensor should be directly in front of the third
        // stone from the right. Scan it, and run the following code if it is a Skystone:
        if(isSkystone()) {

            // Make sure the chassisGrabber is in the same position.
            robot.chassisGrabber.setPosition(1);

            // Latch onto the Skystone using the stoneGrabber, by setting it to a horizontal
            // position.
            robot.stoneGrabber.setPosition(1);

            // Wait for the servo to complete its motion.
            sleep(1000);

            // Backup VERY SLOWLY to avoid tipping the Skystone over.
            preciseDrive(STONE_BACKUP_SPEED, -7, -7,
                    -7, -7, 15);

            // Turn clockwise to account for "drift" caused by the friction of the Skystone
            // against the field.
            preciseDrive(.2, 6, -6,
                    6, -6, 5);

            // Drive right, across the Red Alliance Bridge.
            preciseDrive(1, 80, -80,
                    -80, 80, 10);

            // Set the position of the stoneGrabber ("t-bar") to being vertical
            // to "let go" of the Skystone.
            robot.stoneGrabber.setPosition(.5);

            // Drive left, back under the Skybridge to park.
            preciseDrive(1, -24, 24,
                    24, -24, 10);

            // Drive forward to hopefully allow our Alliance Partner to park as well.
            preciseDrive(1, 10, 10,
                    10, 10, 3);
        }
        // Prepare to scan the second stone from the right if the third is not a Skystone:
        else {

            // Drive backwards slightly to avoid dislodging any stones.
            preciseDrive(SLOW_DRIVE_SPEED, -3, -3,
                    -3, -3, 3);

            // Drive right to approach the next Stone.
            preciseDrive(SLOW_DRIVE_SPEED, 8, -8,
                    -8, 8, 3);

            // Drive forwards slightly to approach the next Stone.
            preciseDrive(SLOW_DRIVE_SPEED, 3, 3,
                    3, 3, 3);

            // At this point, the robot's color sensor should be directly in front of the second
            // stone from the right. Scan it, and run the following code if it is a Skystone:
            if(isSkystone()){

                // Make sure the chassisGrabber is in the same position.
                robot.chassisGrabber.setPosition(1);

                // Latch onto the Skystone using the stoneGrabber, by setting it to a horizontal
                // position.
                robot.stoneGrabber.setPosition(1);

                // Wait for the servo to complete its motion.
                sleep(1000);

                // Backup VERY SLOWLY to avoid tipping the Skystone over.
                preciseDrive(STONE_BACKUP_SPEED, -7, -7,
                        -7, -7, 15);

                // Turn clockwise to account for "drift" caused by the friction of the Skystone
                // against the field.
                preciseDrive(.2, 6, -6,
                        6, -6, 5);

                // Drive right, across the Red Alliance Bridge.
                preciseDrive(1, 72, -72,
                        -72, 72, 10);

                // Set the position of the stoneGrabber ("t-bar") to being vertical
                // to "let go" of the Skystone.
                robot.stoneGrabber.setPosition(.5);

                // Drive left, back under the Skybridge to park.
                preciseDrive(1, -24, 24,
                        24, -24, 10);

                // Drive forward to hopefully allow our Alliance Partner to park as well.
                preciseDrive(1, 10, 10,
                        10, 10, 3);
            }
            // Prepare to grab the rightmost stone, which should be a Skystone if the other two
            // are not:
            else {

                // Drive backwards slightly to avoid dislodging any stones.
                preciseDrive(SLOW_DRIVE_SPEED, -3, -3,
                        -3, -3, 3);

                // Drive right to approach the last Stone.
                preciseDrive(SLOW_DRIVE_SPEED, 10, -10,
                        -10, 10, 3);

                // Drive forwards slightly to approach the last Stone.
                preciseDrive(SLOW_DRIVE_SPEED, 3, 3,
                        3, 3, 3);

                // Make sure the chassisGrabber is in the same position.
                robot.chassisGrabber.setPosition(1);

                // Latch onto the Skystone using the stoneGrabber, by setting it to a horizontal
                // position.
                robot.stoneGrabber.setPosition(1);

                // Wait for the servo to complete its motion.
                sleep(2000);

                // Backup VERY SLOWLY to avoid tipping the Skystone over.
                preciseDrive(STONE_BACKUP_SPEED, -7, -7,
                        -7, -7, 15);

                // Turn clockwise to account for "drift" caused by the friction of the Skystone
                // against the field.
                preciseDrive(.2, 6, -6,
                        6, -6, 5);

                // Drive right, across the Red Alliance Bridge.
                preciseDrive(1, 64, -64,
                        -64, 64, 10);

                // Set the position of the stoneGrabber ("t-bar") to being vertical
                // to "let go" of the Skystone.
                robot.stoneGrabber.setPosition(.5);

                // Drive left, back under the Skybridge to park.
                preciseDrive(1, -24, 24,
                        24, -24, 10);

                // Drive forward to hopefully allow our Alliance Partner to park as well.
                preciseDrive(1, 10, 10,
                        10, 10, 3);

            }
        }
    }
}