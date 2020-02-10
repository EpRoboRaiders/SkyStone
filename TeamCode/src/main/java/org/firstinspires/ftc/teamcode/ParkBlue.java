package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Disabled
@Autonomous(name="Park under Bridge", group="Linear Opmode")

public class ParkBlue extends AutonomousBase {

    @Override
    public void runOpMode() {

        initRobot();

        preciseDrive(.5, 10, -10, -10, 10, 5);



        preciseDrive(.5, 28, 28,
                28, 28,5);

    }
}
