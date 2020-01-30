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


}
