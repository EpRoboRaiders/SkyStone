package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Robot Autonomous", group="Linear Opmode")
public class FullAutonomous extends AutonomousBase{
    @Override

    public void runOpMode() {

        initRobot();

        if (team == "red") {

            if (autonomous) {

                // Translation: Run the code to identify and move Skystones.
                if (starting_location == "Loading Zone") {

                    // Set the position of chassisGrabber (which the stoneGrabber is mounted on) to 1 for
                    // consistency.
                    robot.chassisGrabber.setPosition(1);

                    // Set the position of the stoneGrabber ("t-bar") to being vertical
                    // to prepare to grab a Skystone.
                    robot.stoneGrabber.setPosition(.5);


                    // Drive forward to prepare to align with a Stone.
                    preciseDrive(SLOW_DRIVE_SPEED, 25, 25,
                            25, 25, 3);

                    // Drive left to align the robot with the third Stone from the right.
                    preciseDrive(SLOW_DRIVE_SPEED, -8.5, 8.5,
                            8.5, -8.5, 3);

                    // Drive forward to approach the first Stone.
                    preciseDrive(SLOW_DRIVE_SPEED, 5, 5,
                            5, 5, 5);

                    // At this point, the robot's color sensor should be directly in front of the third
                    // stone from the right. Scan it, and run the following code if it is a Skystone:
                    if(isSkystone()) {

                        // Make sure the chassisGrabber is in the same position.
                        robot.chassisGrabber.setPosition(1);

                        // Latch onto the Skystone using the stoneGrabber, by setting it to a horizontal
                        // position.
                        robot.stoneGrabber.setPosition(1);

                        // Wait for the servo to complete its motion.
                        sleep(1000);

                        // Backup VERY SLOWLY to avoid tipping the Skystone over.
                        preciseDrive(STONE_BACKUP_SPEED, -7, -7,
                                -7, -7, 15);

                        // Turn clockwise to account for "drift" caused by the friction of the Skystone
                        // against the field.
                        preciseDrive(.2, 6, -6,
                                6, -6, 5);

                        // Drive right, across the Red Alliance Bridge.
                        preciseDrive(1, 80, -80,
                                -80, 80, 10);

                        // Set the position of the stoneGrabber ("t-bar") to being vertical
                        // to "let go" of the Skystone.
                        robot.stoneGrabber.setPosition(.5);

                        // Drive left, back under the Skybridge to park.
                        preciseDrive(1, -24, 24,
                                24, -24, 10);
                    }
                    // Prepare to scan the second stone from the right if the third is not a Skystone:
                    else {

                        // Drive backwards slightly to avoid dislodging any stones.
                        preciseDrive(SLOW_DRIVE_SPEED, -3, -3,
                                -3, -3, 3);

                        // Drive right to approach the next Stone.
                        preciseDrive(SLOW_DRIVE_SPEED, 8, -8,
                                -8, 8, 3);

                        // Drive forwards slightly to approach the next Stone.
                        preciseDrive(SLOW_DRIVE_SPEED, 3, 3,
                                3, 3, 3);

                        // At this point, the robot's color sensor should be directly in front of the second
                        // stone from the right. Scan it, and run the following code if it is a Skystone:
                        if(isSkystone()){

                            // Make sure the chassisGrabber is in the same position.
                            robot.chassisGrabber.setPosition(1);

                            // Latch onto the Skystone using the stoneGrabber, by setting it to a horizontal
                            // position.
                            robot.stoneGrabber.setPosition(1);

                            // Wait for the servo to complete its motion.
                            sleep(1000);

                            // Backup VERY SLOWLY to avoid tipping the Skystone over.
                            preciseDrive(STONE_BACKUP_SPEED, -7, -7,
                                    -7, -7, 15);

                            // Turn clockwise to account for "drift" caused by the friction of the Skystone
                            // against the field.
                            preciseDrive(.2, 6, -6,
                                    6, -6, 5);

                            // Drive right, across the Red Alliance Bridge.
                            preciseDrive(1, 72, -72,
                                    -72, 72, 10);

                            // Set the position of the stoneGrabber ("t-bar") to being vertical
                            // to "let go" of the Skystone.
                            robot.stoneGrabber.setPosition(.5);

                            // Drive left, back under the Skybridge to park.
                            preciseDrive(1, -24, 24,
                                    24, -24, 10);
                        }
                        // Prepare to grab the rightmost stone, which should be a Skystone if the other two
                        // are not:
                        else {

                            // Drive backwards slightly to avoid dislodging any stones.
                            preciseDrive(SLOW_DRIVE_SPEED, -3, -3,
                                    -3, -3, 3);

                            // Drive right to approach the last Stone.
                            preciseDrive(SLOW_DRIVE_SPEED, 10, -10,
                                    -10, 10, 3);

                            // Drive forwards slightly to approach the last Stone.
                            preciseDrive(SLOW_DRIVE_SPEED, 3, 3,
                                    3, 3, 3);

                            // Make sure the chassisGrabber is in the same position.
                            robot.chassisGrabber.setPosition(1);

                            // Latch onto the Skystone using the stoneGrabber, by setting it to a horizontal
                            // position.
                            robot.stoneGrabber.setPosition(1);

                            // Wait for the servo to complete its motion.
                            sleep(2000);

                            // Backup VERY SLOWLY to avoid tipping the Skystone over.
                            preciseDrive(STONE_BACKUP_SPEED, -7, -7,
                                    -7, -7, 15);

                            // Turn clockwise to account for "drift" caused by the friction of the Skystone
                            // against the field.
                            preciseDrive(.2, 6, -6,
                                    6, -6, 5);

                            // Drive right, across the Red Alliance Bridge.
                            preciseDrive(1, 64, -64,
                                    -64, 64, 10);

                            // Set the position of the stoneGrabber ("t-bar") to being vertical
                            // to "let go" of the Skystone.
                            robot.stoneGrabber.setPosition(.5);

                            // Drive left, back under the Skybridge to park.
                            preciseDrive(1, -24, 24,
                                    24, -24, 10);

                        }
                    }
                }
                // Translation: Run the code to reposition the Foundation.
                else if (starting_location == "Building Zone") {

                    // Drive forward to get closer to the foundation.
                    timeDrive(1, 1, 1,
                            1, 1, .25);

                    // Strafe right to become "centered" on the platform, and so both clamps can attach.
                    timeDrive(1, 1, -1,
                            -1, 1, .5);

                    // Drive forward to "push" the robot up against the foundation. Pushing the foundation
                    // slightly is acceptable.
                    timeDrive(1, 1, 1,
                            1, 1, .8);

                    timeDrive(.5, 1, 1,
                            1, 1, .2);

                    // Lower the clamps to latch onto the foundation.
                    clampSet("down");

                    // Drive backwards to move the foundation back into the base.
                    timeDrive(1, -1, -1,
                            -1, -1, 2);

                    // Strafe right to "force" the foundation against the wall.
                    timeDrive(1, 1, -1,
                            -1, 1, .75);

                    // Drive backwards to make sure at least part of the foundation is in the base, and that
                    // the robot is touching the corner of the field.
                    timeDrive(1, -1, -1,
                            -1, -1, .25);

                    // Raise the clamps to let go from the foundation.
                    clampSet("up");

                    // Strafe left to move under the bridge.
                    timeDrive(1, -1, 1,
                            1, -1, 3);
                }
            } else {
                if (starting_location == "Loading Zone") {

                    // Strafe left under the bridge.
                    preciseDrive(.5, -24, 24,
                            24, -24, 5);

                } else if (starting_location == "Building Zone") {

                    // Strafe right under the bridge.
                    preciseDrive(.5, 24, -24,
                            -24, 24, 5);

                }
            }
        }
        else if(team == "blue") {

            if(autonomous){

                // Translation: Run the code to identify and move Skystones.
                if (starting_location == "Loading Zone") {

                    // Set the position of chassisGrabber (which the stoneGrabber is mounted on) to 1 for
                    // consistency.
                    robot.chassisGrabber.setPosition(1);

                    // Set the position of the stoneGrabber ("t-bar") to being vertical
                    // to prepare to grab a Skystone.
                    robot.stoneGrabber.setPosition(.5);

                    // Drive forward to prepare to align with a Stone.
                    preciseDrive(SLOW_DRIVE_SPEED, 25, 25,
                            25, 25, 3);

                    // Drive right slightly to align the robot with the third Stone from the left.
                    preciseDrive(SLOW_DRIVE_SPEED, 2, -2,
                            -2, 2, 3);

                    // Drive forward to approach the first Stone.
                    preciseDrive(SLOW_DRIVE_SPEED, 5.5, 5.5,
                            5.5, 5.5, 5);

                    // At this point, the robot's color sensor should be directly in front of the third
                    // stone from the left. Scan it, and run the following code if it is a Skystone:
                    if(isSkystone()) {

                        // Make sure the chassisGrabber is in the same position.
                        robot.chassisGrabber.setPosition(1);

                        // Latch onto the Skystone using the stoneGrabber, by setting it to a horizontal
                        // position.
                        robot.stoneGrabber.setPosition(1);

                        // Wait for the servo to complete its motion.
                        sleep(1000);

                        // Backup VERY SLOWLY to avoid tipping the Skystone over.
                        preciseDrive(STONE_BACKUP_SPEED, -7, -7,
                                -7, -7, 15);

                        // Turn counterclockwise to account for "drift" caused by the friction of the Skystone
                        // against the field.
                        preciseDrive(.2, -6, 6,
                                -6, 6, 5);

                        // Drive left, across the Blue Alliance Bridge.
                        preciseDrive(1, -80, 80,
                                80, -80, 10);

                        // Set the position of the stoneGrabber ("t-bar") to being vertical
                        // to "let go" of the Skystone.
                        robot.stoneGrabber.setPosition(.5);

                        // Drive right, back under the Skybridge to park.
                        preciseDrive(1, 30, -30,
                                -30, 30, 10);
                    }
                    // Prepare to scan the second stone from the left if the third is not a Skystone:
                    else {

                        // Drive backwards slightly to avoid dislodging any stones.
                        preciseDrive(SLOW_DRIVE_SPEED, -3, -3,
                                -3, -3, 3);

                        // Drive left to approach the next Stone.
                        preciseDrive(SLOW_DRIVE_SPEED, -8, 8,
                                8, -8, 3);

                        // Drive forwards slightly to approach the next Stone.
                        preciseDrive(SLOW_DRIVE_SPEED, 3, 3,
                                3, 3, 3);

                        // At this point, the robot's color sensor should be directly in front of the second
                        // stone from the left. Scan it, and run the following code if it is a Skystone:
                        if(isSkystone()){

                            // Make sure the chassisGrabber is in the same position.
                            robot.chassisGrabber.setPosition(1);

                            // Latch onto the Skystone using the stoneGrabber, by setting it to a horizontal
                            // position.
                            robot.stoneGrabber.setPosition(1);

                            // Wait for the servo to complete its motion.
                            sleep(1000);

                            // Backup VERY SLOWLY to avoid tipping the Skystone over.
                            preciseDrive(STONE_BACKUP_SPEED, -7, -7,
                                    -7, -7, 15);

                            // Turn counterclockwise to account for "drift" caused by the friction of the Skystone
                            // against the field.
                            preciseDrive(.2, -6, 6,
                                    -6, 6, 5);

                            // Drive left, across the Blue Alliance Bridge.
                            preciseDrive(1, -72, 72,
                                    72, -72, 10);

                            // Set the position of the stoneGrabber ("t-bar") to being vertical
                            // to "let go" of the Skystone.
                            robot.stoneGrabber.setPosition(.5);

                            // Drive right, back under the Skybridge to park.
                            preciseDrive(1, 30, -30,
                                    -30, 30, 10);

                        }
                        // Prepare to grab the leftmost stone, which should be a Skystone if the other two
                        // are not:
                        else {

                            // Drive backwards slightly to avoid dislodging any stones.
                            preciseDrive(SLOW_DRIVE_SPEED, -3, -3,
                                    -3, -3, 3);

                            // Drive left to approach the last Stone.
                            preciseDrive(SLOW_DRIVE_SPEED, -10, 10,
                                    10, -10, 3);

                            // Drive forwards slightly to approach the last Stone.
                            preciseDrive(SLOW_DRIVE_SPEED, 3.5, 3.5,
                                    3.5, 3.5, 3);

                            // Make sure the chassisGrabber is in the same position.
                            robot.chassisGrabber.setPosition(1);

                            // Latch onto the Skystone using the stoneGrabber, by setting it to a horizontal
                            // position.
                            robot.stoneGrabber.setPosition(1);

                            // Wait for the servo to complete its motion.
                            sleep(2000);

                            // Backup VERY SLOWLY to avoid tipping the Skystone over.
                            preciseDrive(STONE_BACKUP_SPEED, -7.5, -7.5,
                                    -7.5, -7.5, 15);

                            // Turn counterclockwise to account for "drift" caused by the friction of the Skystone
                            // against the field.
                            preciseDrive(.2, -6, 6,
                                    -6, 6, 5);

                            // Drive left, across the Blue Alliance Bridge.
                            preciseDrive(1, -64, 64,
                                    64, -64, 10);

                            // Set the position of the stoneGrabber ("t-bar") to being vertical
                            // to "let go" of the Skystone.
                            robot.stoneGrabber.setPosition(.5);

                            // Drive right, back under the Skybridge to park.
                            preciseDrive(1, 30, -30,
                                    -30, 30, 10);

                        }
                    }
                }
                // Translation: Run the code to reposition the Foundation.
                else if (starting_location == "Building Zone") {

                    // Drive forward to get closer to the foundation.
                    timeDrive(1, 1, 1,
                            1, 1, .25);

                    // Strafe left to become "centered" on the platform, and so both clamps can attach.
                    timeDrive(1, -1, 1,
                            1, -1,.5);

                    // Drive forward to "push" the robot up against the foundation. Pushing the foundation
                    // slightly is acceptable.
                    timeDrive(1, 1, 1,
                            1, 1, .8);

                    timeDrive(.5, 1, 1,
                            1, 1, .2);

                    // Lower the clamps to latch onto the foundation.
                    clampSet("down");

                    // Drive backwards to move the foundation back into the base.
                    timeDrive(1, -1, -1,
                            -1, -1, 2);

                    // Strafe left to "force" the foundation against the wall.
                    timeDrive(1, -1, 1,
                            1, -1,.75);

                    // Drive backwards to make sure at least part of the foundation is in the base, and that
                    // the robot is touching the corner of the field.
                    timeDrive(1, -1, -1,
                            -1, -1, .25);

                    // Raise the clamps to let go from the foundation.
                    clampSet("up");

                    // Strafe right to move under the bridge.
                    timeDrive(1, 1, -1,
                            -1, 1, 3);
                }
            }
            else {

                if (starting_location == "Loading Zone") {

                    // Strafe right under the bridge.
                    preciseDrive(.5, 24, -24,
                            -24, 24, 5);

                }
                else if (starting_location == "Building Zone") {

                    // Strafe left under the bridge.
                    preciseDrive(.5, -24, 24,
                            24, -24, 5);

                }
            }
        }

        if(parking_location == "bridge") {
            // Drive forwards into the "far side" of the Alliance bridge.
            preciseDrive(.5, 28, 28,
                    28, 28,5);
        }
        else if(parking_location == "wall") {
            // Drive backwards into the wall.
            preciseDrive(.5, -28, -28,
                    -28, -28,5);
        }
    }
}
