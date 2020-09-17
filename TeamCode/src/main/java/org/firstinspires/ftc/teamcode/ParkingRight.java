package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Left Side of Skybridge- Park Under Skybridge", group="Parking")

// ParkingRight is a useful, albeit niche program used to simply park under our Alliance-specific
// Skybridge during Autonomous. It doesn't seem like much, but 5 points can mean the difference
// in a match.

public class ParkingRight extends AutonomousBase {

    @Override
    public void runOpMode() {

        // Initialize the robot.
        initRobot();

        sleep(1900);

        // Strafe left to move under the Skybridge.
        preciseDrive(DRIVE_SPEED, 42, -42,
                -42, 42, 10);

        // If predetermined to do so, drive forward to the "far" side of the Skybridge (nearest
        // to the Neutral bridge) to avoid the path of the other robot.

        if(parking_location == "Bridge") {

            preciseDrive(DRIVE_SPEED, 36, 36,
                    36, 36, 10);
        }
    }
}
