package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Right Side of Skybridge- Park Under Skybridge", group="Parking")

public class ParkingLeft extends AutonomousBase {

    @Override
    public void runOpMode() {

        // Initialize the robot.
        initRobot();

        // Strafe right to move under the Skybridge.
        preciseDrive(.1, -42, 42,
                42, -42, 10);

        // If predetermined to do so, drive forward to the "far" side of the Skybridge (nearest
        // to the Neutral bridge) to avoid the path of the other robot.

        if(parking_location == "Bridge") {

            preciseDrive(.1, 36, 36,
                    36, 36, 10);
        }
    }
}
