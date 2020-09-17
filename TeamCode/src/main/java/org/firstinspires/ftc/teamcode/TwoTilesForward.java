package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Drive Two Tiles Forward", group="Parking")

public class TwoTilesForward extends AutonomousBase {

    @Override
    public void runOpMode() {

        initRobot();

        // Go go go
        preciseDrive(.75, 36, 36,
                36, 36, 10);
    }
    
}
