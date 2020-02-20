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
// to identify and move Skystones. Note that it is based on the AutonmousBase class,
// and uses its methods.
public class SkystonesBlue extends AutonomousBase {

    @Override
    public void runOpMode() {

        initRobot();


        grabStone("up");

        preciseDrive(.1, -20, -20,
                -20, -20, 10);

        preciseDrive(.1, -1.5, 1.5,
                 1.5, -1.5, .5);

        // Drive "forward" (actually backward) to approach the first Stone.
        preciseDrive(.1, -7, -7,
                -7, -7, 10);

        if(isSkystone()) {

            grabStone("down");

            preciseDrive(.1, 24, 24,
                    24, 24, 10);

            preciseDrive(.25, 60, -60,
                    -60, 60, 9.5);
        }
        else {

            preciseDrive(.1, 3, 3,
                    3, 3, 10);

            preciseDrive(.1, 8.25, -8.25,
                    -8.25, 8.25, 9.5);

            preciseDrive(.1, -3.25, -3.25,
                    -3.25, -3.25, 10);

            if(isSkystone()) {

                grabStone("down");

                preciseDrive(.1, 24, 24,
                        24, 24, 10);

                preciseDrive(.25, 52, -52,
                        -52, 52, 9.5);
            }
            else {

                preciseDrive(.1, 3, 3,
                        3, 3, 10);

                preciseDrive(.1, 7.5, -7.5,
                        -7.5, 7.5, 9.5);

                preciseDrive(.1, -3.25, -3.25,
                        -3.25, -3.25, 10);

                grabStone("down");

                preciseDrive(.1, 24, 24,
                        24, 24, 10);

                preciseDrive(.25, 44, -44,
                        -44, 44, 9.5);
            }
        }

        grabStone("up");

        preciseDrive(.1, -12, 12,
                12, -12, 9.5);

        if(parking_location == "Bridge") {

            preciseDrive(.1, -15, -15,
                    -15, -15, 10);
        }
        else {

            preciseDrive(.1, 21, 21,
                    21, 21, 10);

            preciseDrive(.1, -8, 8,
                    8, -8, 9.5);
        }
    }
}