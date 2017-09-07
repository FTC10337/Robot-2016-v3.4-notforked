/*
Copyright (c) 2017 Dark Matter FTC 10337

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * 100 point autonomous main routine.  It processes both blue and red -- based on the amIBlue method return.
 *
 * Strategy:
 * -- Move forward
 * -- Fire 2 particles into center vortex
 * -- Turn and drive towards wall w/ beacons
 * -- Turn parallel to wall
 * -- Drive to find white stripe on first beacon
 * -- Read color sensor, and decide whether to move forward or back to align beacon presser
 * -- Move as needed and press beacon
 * -- Move to 2nd white line and repeat beacon color sense and press
 * -- Drive to center vortex, knock cap ball, and park.
 */

@Autonomous(name="2. Auto Blue 100 Shoot Last & Park", group="1.BEACONS")
// @Disabled
public class Auto100BlueShootLast extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDM         robot   = new HardwareDM ();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.8;     // Nominal speed for auto moves.
    static final double     DRIVE_SPEED_SLOW        = 0.65;     // Slower speed where required
    static final double     TURN_SPEED              = 0.8;     // Turn speed

    static final double     HEADING_THRESHOLD       = 2 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.011;   // Larger is more responsive, but also less accurate
    static final double     P_TURN_COEFF2           = 0.025;
    static final double     P_TURN_COEFF_RED        = 0.0085;
    static final double     P_DRIVE_COEFF_1         = 0.03;  // Larger is more responsive, but also less accurate
    static final double     P_DRIVE_COEFF_2         = 0.02;

    // Coeff for doing range sensor driving -- 1.25 degrees per CM off
    static final double     P_DRIVE_COEFF_3         = 1.25;
    static final double     RANGE_THRESHOLD         = 1.0;  // OK at +/- 1cm
    static final double     WALL_DISTANCE_1           = 12.0; // 12 cm from wall for beacons
    static final double     WALL_DISTANCE_2           = 11.0;

    // White line finder thresholds
    static final double     WHITE_THRESHOLD         = 2.0;      // Line finder

    // Beacon color sensor thresholds
    static final double     BEACON_ALPHA_MIN        = 100.0;
    static final double     BLUE_MIN                = -180;
    static final double     BLUE_MAX                = -100;
    static final double     RED_MIN                 = -40;
    static final double     RED_MAX                 = 40;

    // Variables used for reading Gyro
    Orientation             angles;
    double                  headingBias = 0.0;            // Gyro heading adjustment

    // Keep track of how far we moved to line up to press beacons
    double distCorrection = 0.0;
    double distCorrection_2 = 0.0;


    // Storage for reading adaFruit color sensor for beacon sensing
    // adaHSV is an array that will hold the hue, saturation, and value information.
    float[] adaHSV = {0F, 0F, 0F};
    // adaValues is a reference to the adaHSV array.
    final float adaValues[] = adaHSV;

    /**
     * The main routine of the OpMode.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {

        int beacon = 0;         // What color beacon do we see

        RobotLog.i("DM10337- Starting Auto 100 init.  We are:" + (amIBlue()?"Blue":"Red"));

        // Init the robot hardware -- including gyro and range finder
        robot.init(hardwareMap, true);

        // And turn on the LED on stripe finder
        robot.stripeColor.enableLed(true);

        // Force reset the drive train encoders.  Do it twice as sometimes this gets missed due to USB congestion
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RobotLog.i("DM10337 -- Drive train encoders reset");

        RobotLog.i("DM10337- Finished Init");

        // Show telemetry for gyro status
        telemetry.addData("IMU calibrated: ", robot.adaGyro.isSystemCalibrated());
        telemetry.addData("IMU Gyro calibrated:  ", robot.adaGyro.isGyroCalibrated());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        // Set a timer of how often to update gyro status telemetry
        ElapsedTime updateGyroStatTimer = new ElapsedTime();
        updateGyroStatTimer.reset();
        while (!isStarted()) {
            if (updateGyroStatTimer.milliseconds() >= 500) {
                // Update telemetry every 0.5 seconds
                telemetry.addData("IMU calibrated: ", robot.adaGyro.isSystemCalibrated());
                telemetry.addData("IMU Gyro calibrated:  ", robot.adaGyro.isGyroCalibrated());

                // Do a gyro read to keep it "fresh"
                telemetry.addData("Gyro heading: ", readGyro());
                telemetry.update();

                // And reset the timer
                updateGyroStatTimer.reset();
            }
            idle();
        }

        RobotLog.i("DM10337- Auto Pressed Start");
        // Step through each leg of the path,

        // Make sure the gyro is zeroed
        zeroGyro();




        RobotLog.i("DM10337 - Gyro bias set to " + headingBias);



        // Drive towards the beacon wall
        // Use gyro to hold heading
        // Distance is the "inside" of the turn distance
        encoderDrive(DRIVE_SPEED, amIBlue()?75.5:75.0, 5.0, true, amIBlue()?-25.0:25.0, false);




         //waitForSwitch();

        // Turn parallel to beacon wall using gyro
        gyroTurn(TURN_SPEED, amIBlue()?0.0:180.0, amIBlue()?P_TURN_COEFF: P_TURN_COEFF_RED);



        // Move slowly to approach 1st beacon -- Slow allows us to be more accurate w/ alignment
        // Autocorrects any heading errors while driving
        encoderDrive(DRIVE_SPEED_SLOW, amIBlue()?-10.0:30.0, 5.0, true,
                amIBlue()?0.0:180.0, false, true, amIBlue()?WALL_DISTANCE_1:WALL_DISTANCE_2);


        //waitForSwitch();

        // If heading is more than 5 degrees from target heading after wall follow, robot turns to correct heading
        double headingThreshold = getError(amIBlue()?0:180);
        if (headingThreshold > 2){
            gyroTurn(TURN_SPEED, amIBlue()?0:180, P_TURN_COEFF2);
            RobotLog.i ("DM10337 - adjusted heading before find line1 by " + headingThreshold);
        }

        //waitForSwitch();

        // Use line finder to align to white line
        findLine(amIBlue()?-0.10:0.10, 5.0);

        if (headingThreshold > 2){
            gyroTurn(TURN_SPEED, amIBlue()?0:180, P_TURN_COEFF2);
            RobotLog.i ("DM10337 - adjusted heading after find line1 by " + headingThreshold);
        }
        //waitForSwitch();
        // Wait for beacon color sensor
        sleep(1000);

        beacon = beaconColor();
        if (beacon == 1) {
            // We see blue
            distCorrection = amIBlue()?1.2:-2.15;
        } else if (beacon == -1) {
            // We see red
            distCorrection = amIBlue()?-1.85:1.1;
        } else {
            // We see neither
            distCorrection = 0;
        }

        encoderDrive(0.2, distCorrection, 2.5, true,
                amIBlue()?(0):(180), true);

        if (beacon == 1) {
            // We see blue
            distCorrection_2 = amIBlue()?4.5:-4.5;
        } else if (beacon == -1) {
            // We see red
            distCorrection_2 = amIBlue()?-4.5:4.5;
        } else {
            // We see neither
            distCorrection_2 = 0;
        }

        if (beacon != 0) {
            // We saw the beacon color so press the center of the beacon
            robot.beacon.setPosition(robot.BEACON_MAX_RANGE);
            //waitForSwitch();
            encoderDrive(0.15, distCorrection_2, 2.5, true,
                    amIBlue()?(0):(180), true);

            // Return beacon arm back to home position
            //waitForSwitch();
            robot.beacon.setPosition((robot.BEACON_HOME));
            sleep(100);

        }

        // Drive to the 2nd beacon.  Tweaked Red heading to correct alignment errors.
        // Use rangefinder correction to get us to 10cm from all
        encoderDrive(DRIVE_SPEED_SLOW, amIBlue()?42.0 - distCorrection - distCorrection_2:-44.0 - distCorrection - distCorrection_2, 4.0,
                true, amIBlue()?0.0:180.0, false, true, WALL_DISTANCE_1);

        //waitForSwitch();

        // If heading is more than 5 degrees from target heading after wall follow, robot turns to correct heading
        headingThreshold = getError(amIBlue()?0:180);
        if (headingThreshold > 2){
            gyroTurn(TURN_SPEED, amIBlue()?0:180, P_TURN_COEFF2);
            RobotLog.i ("DM10337 - adjusted heading before find line2 by " + headingThreshold);

        }

        //waitForSwitch();

        // Find the 2nd white line
        findLine(amIBlue()?0.10:-0.10, 5.0);

        if (headingThreshold > 2){
            gyroTurn(TURN_SPEED, amIBlue()?0:180, P_TURN_COEFF2);
            RobotLog.i ("DM10337 - adjusted heading after find line2 by " + headingThreshold);
        }
        // wait for color sensor
        sleep(1000);

        //waitForSwitch();

        // Check the beacon color
        beacon = beaconColor();
        if (beacon == 1) {
            // I see blue
            distCorrection = amIBlue()?1.2:-2.25;
        } else if (beacon == -1) {
            // I see red
            distCorrection = amIBlue()?-1.85:1.1;
        } else {
            // I see neither
            distCorrection = 0;
        }

        encoderDrive(0.2, distCorrection, 2.5, true,
                amIBlue()?(0):(180), true);

        if (beacon == 1) {
            // I see blue
            distCorrection_2 = amIBlue()?4.5:-4.5;
        } else if (beacon == -1) {
            // I see red
            distCorrection_2 = amIBlue()?-4.5:4.5;
        } else {
            // I see neither
            distCorrection_2 = 0;
        }

        if (beacon != 0) {
            // We saw the beacon color so press the center of the beacon
            robot.beacon.setPosition(robot.BEACON_MAX_RANGE);
            //waitForSwitch();
            encoderDrive(0.15, distCorrection_2, 2.5, true,
                    amIBlue()?0:180, true);

            // Return beacon arm back to home position
            //waitForSwitch();
            robot.beacon.setPosition((robot.BEACON_HOME));
            sleep(100);
        }


        double LINE_FOLLOW_DIRECTION = 0.0;
        if (distCorrection_2 > 0) {
            LINE_FOLLOW_DIRECTION = -1.0;
        } else if (distCorrection_2 < 0) {
            LINE_FOLLOW_DIRECTION = 1.0;
        }

        findLine(LINE_FOLLOW_DIRECTION * 0.20, 5.0);


        // Spin up the shooter
        robot.lShoot.setPower(robot.SHOOT_DEFAULT);
        robot.rShoot.setPower(robot.SHOOT_DEFAULT);

        encoderDrive(DRIVE_SPEED, amIBlue()?-43.0:34.0, 7.0, true, amIBlue()?-48:243, false);

        //Turn toward center vortex to shoot
        gyroTurn(TURN_SPEED, amIBlue()?140:243, amIBlue()?P_TURN_COEFF_RED : P_TURN_COEFF2);

        // Fire the balls
        camDrive(1.0, 3, 50, 1500);

        robot.lShoot.setPower(0.0);
        robot.rShoot.setPower(0.0);

        if (capBallPush()) {
            if(!amIBlue()) robot.intake.setPower(-1.0);
            encoderDrive(DRIVE_SPEED, 24.0, 5.0, true, amIBlue() ? 145:243, false);
            robot.intake.setPower(0.0);
        } else
            {
            encoderDrive(DRIVE_SPEED, -24, 5.0, true, amIBlue() ? 145:243, false);
            }

        RobotLog.i("DM10337- Finished last move of auto");

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *
     */


    /**
     * Abbreviated call to encoderDrive w/o range aggressive turning or finding adjustments
     *
     * @param speed
     * @param distance
     * @param timeout
     * @param useGyro
     * @param heading
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading) throws InterruptedException {
        encoderDrive(speed, distance, timeout, useGyro, heading, false, false, 0.0);
    }


    /**
     * Abbreviated call to encoderDrive w/o range finding adjustments
     *
     * @param speed
     * @param distance
     * @param timeout
     * @param useGyro
     * @param heading
     * @param aggressive
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading,
                             boolean aggressive) throws InterruptedException {
        encoderDrive(speed, distance, timeout, useGyro, heading, aggressive, false, 0.0);
    }

    /**
     *
     * Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     * @param speed                 Motor power (0 to 1.0)
     * @param distance              Inches
     * @param timeout               Seconds
     * @param useGyro               Use gyro to keep/curve to an absolute heading
     * @param heading               Heading to use
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading,
                             boolean aggressive,
                             boolean userange,
                             double maintainRange) throws InterruptedException {

        // Calculated encoder targets
        int newLFTarget;
        int newRFTarget;
        int newLRTarget;
        int newRRTarget;

        // The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            RobotLog.i("DM10337- Starting encoderDrive speed:" + speed +
                    "  distance:" + distance + "  timeout:" + timeout +
                    "  useGyro:" + useGyro + " heading:" + heading + "  maintainRange: " + maintainRange);

            // Calculate "adjusted" distance  for each side to account for requested turn during run
            // Purpose of code is to have PIDs closer to finishing even on curved moves
            // This prevents jerk to one side at stop
            double leftDistance = distance;
            double rightDistance = distance;
            if (useGyro) {
                // We are gyro steering -- are we requesting a turn while driving?
                double headingChange = getError(curHeading) * Math.signum(distance);
                if (Math.abs(headingChange) > 5.0) {
                    //Heading change is significant enough to account for
                    if (headingChange > 0.0) {
                        // Assume 16 inch wheelbase
                        // Add extra distance to the wheel on outside of turn
                        rightDistance += Math.signum(distance) * 2 * 3.1415 * 16 * headingChange / 360.0;
                        RobotLog.i("DM10337 -- Turn adjusted R distance:" + rightDistance);
                    } else {
                        // Assume 16 inch wheelbase
                        // Add extra distance from the wheel on inside of turn
                        // headingChange is - so this is increasing the left distance
                        leftDistance -= Math.signum(distance) * 2 * 3.1415 * 16 * headingChange / 360.0;
                        RobotLog.i("DM10337 -- Turn adjusted L distance:" + leftDistance);
                    }
                }
            }

            // Determine new target encoder positions, and pass to motor controller
            newLFTarget = robot.lfDrive.getCurrentPosition() + (int)(leftDistance * robot.COUNTS_PER_INCH);
            newLRTarget = robot.lrDrive.getCurrentPosition() + (int)(leftDistance * robot.COUNTS_PER_INCH);
            newRFTarget = robot.rfDrive.getCurrentPosition() + (int)(rightDistance * robot.COUNTS_PER_INCH);
            newRRTarget = robot.rrDrive.getCurrentPosition() + (int)(rightDistance * robot.COUNTS_PER_INCH);

            while(robot.lfDrive.getTargetPosition() != newLFTarget){
                robot.lfDrive.setTargetPosition(newLFTarget);
                sleep(1);
            }
            while(robot.rfDrive.getTargetPosition() != newRFTarget){
                robot.rfDrive.setTargetPosition(newRFTarget);
                sleep(1);
            }
            while(robot.lrDrive.getTargetPosition() != newLRTarget){
                robot.lrDrive.setTargetPosition(newLRTarget);
                sleep(1);
            }
            while(robot.rrDrive.getTargetPosition() != newRRTarget){
                robot.rrDrive.setTargetPosition(newRRTarget);
                sleep(1);
            }

            // Turn On motors to RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED,speed);

            // Set the motors to the starting power
            robot.lfDrive.setPower(Math.abs(curSpeed));
            robot.rfDrive.setPower(Math.abs(curSpeed));
            robot.lrDrive.setPower(Math.abs(curSpeed));
            robot.rrDrive.setPower(Math.abs(curSpeed));

            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opModeIsActive() &&
                   (runtime.seconds() < timeout) &&
                    robot.lfDrive.isBusy() &&
                    robot.lrDrive.isBusy() &&
                    robot.rfDrive.isBusy() &&
                    robot.rrDrive.isBusy()) {

                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;

                // Doing gyro heading correction?
                if (useGyro){

                    // Get the difference in distance from wall to desired distance
                    double errorRange = robot.rangeSensor.getDistance(DistanceUnit.CM) - maintainRange;

                    if (userange) {
                        if (Math.abs(errorRange) >= RANGE_THRESHOLD) {
                            // We need to course correct to right distance from wall
                            // Have to adjust sign based on heading forward or backward
                            curHeading = heading - Math.signum(distance) * errorRange * P_DRIVE_COEFF_3;
                            RobotLog.i("DM10337 - Range adjust -- range:" + errorRange + "  heading: " + curHeading + "  actual heading: " + readGyro());
                        } else {
                            // We are in the right range zone so just use the desired heading w/ no adjustment
                            curHeading = heading;
                        }
                    }

                    // adjust relative speed based on heading
                    double error = getError(curHeading);
                    double steer = getSteer(error,
                            (aggressive?P_DRIVE_COEFF_1:P_DRIVE_COEFF_2));

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0)
                    {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                }

                // And rewrite the motor speeds
                robot.lfDrive.setPower(Math.abs(leftSpeed));
                robot.rfDrive.setPower(Math.abs(rightSpeed));
                robot.lrDrive.setPower(Math.abs(leftSpeed));
                robot.rrDrive.setPower(Math.abs(rightSpeed));

                // Allow time for other processes to run.
                idle();
            }


            RobotLog.i("DM10337- encoderDrive done" +
                    "  lftarget: " +newLFTarget + "  lfactual:" + robot.lfDrive.getCurrentPosition() +
                    "  lrtarget: " +newLRTarget + "  lractual:" + robot.lrDrive.getCurrentPosition() +
                    "  rftarget: " +newRFTarget + "  rfactual:" + robot.rfDrive.getCurrentPosition() +
                    "  rrtarget: " +newRRTarget + "  rractual:" + robot.rrDrive.getCurrentPosition() +
                    "  heading:" + readGyro());

            // Stop all motion;
            robot.lfDrive.setPower(0);
            robot.rfDrive.setPower(0);
            robot.lrDrive.setPower(0);
            robot.rrDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to find a white line
     *
     * @param speed             Speed to move.  Can be negative to find moving backwards
     * @param timeout
     * @return
     */
    public boolean findLine(double speed, double timeout) {

        // Try to find white line
        // loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.

        runtime.reset();
        while (opModeIsActive() &&
                robot.stripeColor.alpha() < WHITE_THRESHOLD &&
                runtime.seconds() < timeout) {

            // Drive til we see the stripe
            robot.lfDrive.setPower(speed);
            robot.rfDrive.setPower(speed);
            robot.lrDrive.setPower(speed);
            robot.rrDrive.setPower(speed);
            idle();
        }

        // Did we find the line?
        boolean finished = (runtime.seconds() < timeout);

        // Use brake mode so we stop quicker at line
        robot.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop moving
        robot.lfDrive.setPower(0.0);
        robot.rfDrive.setPower(0.0);
        robot.lrDrive.setPower(0.0);
        robot.rrDrive.setPower(0.0);

        // And reset to float mode
        robot.setDriveZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);

        return finished;
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle, double coefficient) {

        RobotLog.i("DM10337- gyroTurn start  speed:" + speed +
            "  heading:" + angle);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, coefficient)) {
            // Allow time for other processes to run.
            // onHeading() does the work of turning us
            idle();
        }

        RobotLog.i("DM10337- gyroTurn done   heading actual:" + readGyro());
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            // Close enough so no need to move
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            // Calculate motor powers
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.lfDrive.setPower(leftSpeed);
        robot.rfDrive.setPower(rightSpeed);
        robot.lrDrive.setPower(leftSpeed);
        robot.rrDrive.setPower(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;


        // calculate error in -179 to +180 range  (
        robotError = targetAngle - readGyro();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public int beaconColor () {

        // Return 1 for Blue and -1 for Red
        // convert the RGB adaValues to HSV adaValues.
        Color.RGBToHSV((robot.beaconColor.red() * 255) / 800, (robot.beaconColor.green() * 255) / 800,
                (robot.beaconColor.blue() * 255) / 800, adaHSV);

        // Normalize hue to -180 to 180 degrees
        if (adaHSV[0] > 180.0) {
            adaHSV[0] -= 360.0;
        }

        // Only continue if we see beacon i.e. enough light
        if (robot.beaconColor.alpha() < BEACON_ALPHA_MIN) {
            RobotLog.i("DM10337- Beacon color read & nothing found   alpha:" +
                robot.beaconColor.alpha());
            return 0;
        }


        // Check for blue
        if (adaHSV[0] > BLUE_MIN && adaHSV[0] < BLUE_MAX) {
            // we see blue so return 1.0
            RobotLog.i("DM10337- Beacon color found blue  alpha:" +
                robot.beaconColor.alpha() +
                "  hue:" + adaHSV[0] );
            return 1;
        }

        // Check for red
        if (adaHSV[0] > RED_MIN && adaHSV[0] < RED_MAX) {
            //telemetry.addData("beacon", -1);
            //telemetry.update();
            RobotLog.i("DM10337- Beacon color found red  alpha:" +
                    robot.beaconColor.alpha() +
                    "  hue:" + adaHSV[0] );
            return -1;
        }

        RobotLog.i("DM10337- Beacon color found neither  alpha:" +
                robot.beaconColor.alpha() +
                "  hue:" + adaHSV[0] );
        return 0;         // We didn't see either color so don't know
    }

    /**
     * Record the current heading and use that as the 0 heading point for gyro reads
     * @return
     */
    void zeroGyro() {
        angles = robot.adaGyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        headingBias = angles.firstAngle;
    }


    /**
     * Read the current heading direction.  Use a heading bias if we recorded one at start to account for drift during
     * the init phase of match
     *
     * @return      Current heading (Z axis)
     */
    double readGyro() {
        angles = robot.adaGyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angles.firstAngle - headingBias;
    }

    // Cam drive code
    public void camDrive (double speed, double shots, long pause, double timeout) throws InterruptedException {

        ElapsedTime     pauseTime = new ElapsedTime();

        runtime.reset();

        robot.fire.setPower(speed);
        double totalShots = 0;
        boolean camSwitchPressed = false;
        boolean paused = false;

        while (opModeIsActive() && totalShots < shots && runtime.milliseconds() < timeout) {
            // run cam until limit switch is pressed
            if (!paused && robot.camSwitch.isPressed() && !camSwitchPressed && runtime.milliseconds() > 250) {
                totalShots += 1.0;
                camSwitchPressed = true;
                robot.fire.setPower(0.0);
                pauseTime.reset();
                paused = true;

            } else if (paused && pauseTime.milliseconds() > pause) {
                paused = false;
                robot.fire.setPower(speed);
                pauseTime.reset();
            }
            if (!robot.camSwitch.isPressed() && pauseTime.milliseconds() > 150){
                camSwitchPressed = false;
            }
            idle();
        }
        RobotLog.i("DM10337 -- Auto shot: " + totalShots);
        robot.fire.setPower(0.0);
    }
    /**
     * Always returns true as we are blue.
     *
     * Red OpMode would extend this class and Override this single method.
     *
     * @return          Always true
     */
    public boolean amIBlue() {
        return true;
    }

    public boolean capBallPush() { return true;}

    public boolean waitForSwitch() {
        while (!gamepad1.a) {
            idle();
        }
        return true;
    }
}
