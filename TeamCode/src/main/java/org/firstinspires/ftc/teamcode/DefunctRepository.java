package org.firstinspires.ftc.teamcode;

public class DefunctRepository {

    // This OpMode serves as a place do "dump" all of the commented-out code no longer in use
    // to be forgotten about forever. Yay programming!

     /*
    These motors have names that are "unhelpful" to describing
    the functions of the motors on our personal robot. As a result, we use our own custom motor
    names; these are left for future reference.
     */

    /*
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftArm     = null;
     */

    /*
    Because our robot currently does not use servos, there is no need to define ones that do
    not exist. These initializations are left for future reference.
     */

    /*
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
     */

    /*
    These variables store values related to the servos that, again, do not exist. Also again,
    they are left for reference.
     */

    /*
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
     */



    /* Commented out because the initialization of a timer has little use, in that the timer is
    never referred to in this class. It is kept for reference, or if it ends up being necessary
    for other timers (or code) to function. */
    // private ElapsedTime period = new ElapsedTime();

    // Unused motor mappings. Left for reference.

        /*
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftArm    = hwMap.get(DcMotor.class, "left_arm");
         */

    // Unused motor configuration. Left for reference.

        /*
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
         */


    // Unused motor configuration. Left for reference.

        /*
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftArm.setPower(0);
         */

    // Unused servo mappings. Left for reference.

        /*
        leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
         */

    // Unused servo initialization. Left for reference.

        /*
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
         */

         /*

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int leftTarget;
        int rightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftTarget = robot.leftFront.getCurrentPosition() + (int)(leftInches * 1);
            rightTarget = robot.rightFront.getCurrentPosition() + (int)(rightInches * 1);
            robot.leftFront.setTargetPosition(leftTarget);
            robot.rightFront.setTargetPosition(rightTarget);
            robot.leftBack.setTargetPosition(leftTarget);
            robot.rightBack.setTargetPosition(rightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    ((runtime.seconds() < timeoutS)
                            && robot.leftFront.isBusy() && robot.rightFront.isBusy()
                            && robot.leftBack.isBusy() && robot.rightBack.isBusy())) {
                /*
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();


            }

            // Stop all of the motors after all of the moves have completed.
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);
        }
    }
    */

         /* public void reachStone() {
        // Will probably incorporate a distance sensor trained on the stone. Pending
    }
    public void grabStone() {
        // Will probably involve doing a 180 turn at some point. Pending
    }

          */
       /*
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();
                 */

        /*

        if (gamepad2.dpad_left != leftPressed) {

            if(!leftPressed) {
                if (liftSpeed == MEDIUM) {
                    liftSpeed = LOW;
                }
                else if (liftSpeed == HIGH) {
                    liftSpeed = MEDIUM;
                }
            }

            leftPressed = !leftPressed;
        }


        if (gamepad2.dpad_right != rightPressed) {

            if(!rightPressed) {
                if (liftSpeed == LOW) {
                    liftSpeed = MEDIUM;
                }
                else if (liftSpeed == MEDIUM) {
                    liftSpeed = HIGH;
                }
            }

            rightPressed = !rightPressed;
        }
        */


    // Defunct drive modes that no member of the team preferred driving in. Descriptions of
    // them can be found inside of their respective code.
        /*
        else if(driveMode[currentMode] == "pov") {

            // In "pov' drive mode, the left joystick controls the speed of the robot, and the
            // right joystick controls the heading. Refer to PushbotTeleopPOV_Linear for specifics
            // on how it functions.

            robot.leftFront.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            robot.leftBack.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            robot.rightFront.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            robot.rightBack.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
        }
        else if(driveMode[currentMode] == "debug") {

            // In the "debug" drive mode, each stick on each controller controls its own motor with
            // its "x" value. It is HIGHLY recommended to remove this before competition use.

            robot.leftFront.setPower(gamepad1.right_stick_x);
            robot.rightFront.setPower(gamepad2.left_stick_x);
            robot.leftBack.setPower(gamepad1.left_stick_x);
            robot.rightBack.setPower(gamepad2.right_stick_x);

        }

          for (int i=0; i<(Mode.values().length); i++){
            telemetry.addData("Mode ", Mode.values()[i].ordinal());
            telemetry.addData("Name", Mode.values()[i]);
        }
         */
    // Changes the position of the "chassis" servo from .5 to .25, the range that it would
    // realistically need to be for competition.

        /*

        if (gamepad2.left_bumper != leftBumperPressed) {

            if (!leftBumperPressed) {

                if (mountedPosition == 0) {
                    mountedPosition = 1;
                } else if (mountedPosition == 1) {
                    mountedPosition = 0;
                }
            }

            leftBumperPressed = !leftBumperPressed;
        }

         */


    /*

        // Strafe left to move under the bridge.

        timeDrive(1, -1, 1, 1, -1,1.75);

        // Move forward to avoid the other team's robot.

        timeDrive(1, 1, 1, 1, 1, 1);

         */



    // Strafe right to move under the bridge.

        /*

        timeDrive(1, 1, -1, -1, 1,1.75);

        // Move forward to avoid the other team's robot
        timeDrive(1, 1, 1, 1, 1, 1);

         */



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



    // robot.leftClamp.setPower(gamepad2.left_stick_y);
    // robot.rightClamp.setPower(gamepad2.left_stick_y);

}
