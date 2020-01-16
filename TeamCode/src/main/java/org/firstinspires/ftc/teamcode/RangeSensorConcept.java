package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Range Sensor Concept", group="Iterative Opmode")

public class RangeSensorConcept extends OpMode {

    // Creates a motor based on the RobotTemplate class.

    RobotTemplate robot = new RobotTemplate();



    @Override
    public void init() {

        // Initialize the robot and its motors.

        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.

        telemetry.addData("Robot Mode:", "Initialized");
        telemetry.update();
    }

    // Runs repeatedly after the "init" button is pressed on the phone. Currently, it serves no
    // function.

    @Override
    public void init_loop() {}

    // Runs once after the "start" is pressed on the phone. Currently, it serves no function.

    @Override
    public void start() {}

    // Runs repeatedly after the "start" button is pressed on the phone.

    public void loop() {

        robot.stoneGrabber.setPosition(.5);

    }

    // Runs once after the "stop" button is pressed on the phone. Currently, it serves no function.

    @Override
    public void stop() {}
}
