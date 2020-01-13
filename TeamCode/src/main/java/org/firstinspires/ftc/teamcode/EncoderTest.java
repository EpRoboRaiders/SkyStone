package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Encoder Test", group="Linear Opmode")


public class EncoderTest extends AutonomousBase {

    private ElapsedTime     runtime = new ElapsedTime();

    static final int DISTANCE = 17;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.leftFront.setTargetPosition(DISTANCE * 120);
        robot.rightFront.setTargetPosition(DISTANCE * 120);
        robot.leftBack.setTargetPosition(DISTANCE * 120);
        robot.rightBack.setTargetPosition(DISTANCE * 120);



        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Turn On RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        // reset the timeout time and start motion.
        runtime.reset();


        robot.leftFront.setPower(.5);
        robot.rightFront.setPower(.5);
        robot.leftBack.setPower(.5);
        robot.rightBack.setPower(.5);


        while (opModeIsActive() &&
                (runtime.seconds() < 5)
                && robot.leftFront.isBusy() && robot.rightFront.isBusy()
                && robot.leftBack.isBusy() && robot.rightBack.isBusy()) {

        }

        // Stop all of the motors after all of the moves have completed.
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        //robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         */


    }
}
