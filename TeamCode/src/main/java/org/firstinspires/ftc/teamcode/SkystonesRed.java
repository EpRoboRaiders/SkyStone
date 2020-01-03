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

@Autonomous(name="Red Team- Start By Skystones", group="Linear Opmode")

public class SkystonesRed extends AutonomousBase {

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.leftClamp.setPower(-.375);

        // Strafe right to move under the bridge.

        timeDrive(1, 1, -1, -1, 1,1.75);

        // Move forward to avoid the other team's robot
        timeDrive(1, 1, 1, 1, 1, 1);



        // Get to a stone by using the distance sensor to approach it. Because of the sensitive
        // nature of the colour sensor, the robot will have to almost be "touching" the stone.

        // Scan the stone using the color sensor. If the "redness" of the Stone scanned is
        // sufficiently low, then the stone can safely be labeled as a "Skystone" and the code
        // can continue; otherwise, repeat the first step.

        // "Pick up" the stone by a method currently unknown, given the odd angle the stones would
        // need to be approached from in order to use the "Stone Lift."

        // Move to the Building Zone by backing up and entering it.

        // Release the stone from the robot; this can be done trivially given a "method" by which
        // the stone is grabbed.

        // Park under the bridge, or go for another Skystone if time allows (which it most likely
        // will not).



        // Code to approach the first stone for scanning.

        // Code to scan the first stone to check if it is a Skystone.\

        /*

        if(isSkystone()) {

            // Code to pick up and deliver the first stone if it can be reasonably identified as
            // a Skystone.

        }
        else {

            // Code to approach the second stone for scanning if if it is of the "Sky" variety and
            // as such earns the robot picking it up bonus points.

            if(isSkystone()){

                // Code to pick up and deliver the second stone if it is of a nature which allows it
                // to be identified by the prefix "Sky," referring to the black "Star Wars" related
                // image located on its side.

            }
            else{

                // Code to approach the third and final stone and pick it up if it, according to
                // the FTC 2019-2020 Season rules, unlike the other two stones scanned before it,
                // falls under the classification of being a "Skystone" (which, notably, is the
                // title of the current FTC Game), and as such earns a robot that delivers it across
                // its alliances Skybridge 10 bonus points. Note that if the Skystone is not one of
                // the first two stones delivered across the previously mentioned bridge, it earns
                // the deliverer only 2 points, the same as a regular Stone.
            }
        }

         */


    }

}