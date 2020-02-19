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

@Autonomous(name="Blue Team- Start By Foundation", group="Linear Opmode")

// FoundationBlue is run during Autonomous when our robot is on the Blue team and is attempting
// to move the foundation. Note that it is based on the AutonomousBase class, and uses its methods.

public class FoundationBlue extends AutonomousBase {

    @Override
    public void runOpMode() {

        // Initialize the robot.
        initRobot();

        clampSet("up");

        // Drive forward to get closer to the foundation.
        preciseDrive(.1, 24, 24,
                24, 24, 10);

        // Strafe left to get lined up with center of the foundation.
        preciseDrive(.1, -12, 12,
                12, -12, 10);

        // Drive forward to "push" the robot up against the foundation. Pushing the foundation
        // slightly is acceptable.
        preciseDrive(.1, 6, 6,
                6, 6, 10);

        clampSet("down");

        // Drive backwards to move the foundation back into the base.
        preciseDrive(.1, -38, -38,
                -38, -38, 10);

        // Strafe left to "force" the foundation against the wall.
        preciseDrive(.1, -14, 14,
                14, -14, 10);

        // Drive backwards to make sure at least part of the foundation is in the base, and that
        // the robot is touching the corner of the field.
        preciseDrive(.1, -2, -2,
                -2, -2, 10);

        clampSet("up");

        // Strafe right to move under the bridge.

        preciseDrive(.1, 68, -68,
                -68, 68, 10);

        // If parking by the far side of the Skybridge, drive forward to do so.
        if(parking_location == "Bridge"){

            preciseDrive(.1, 36, 36,
                    36, 36, 10);
        }



    }
}