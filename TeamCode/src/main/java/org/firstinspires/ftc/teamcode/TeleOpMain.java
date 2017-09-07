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

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file provides  Telop driving for Dark Matter 2016-17 robot.
 *
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common HardwareDM hardware class to define the devices on the robot.
 * All device access is managed through the HardwareDM class.
 *
 */
@TeleOp(name="1. TeleOpMain", group="DM")
// @Disabled

public class TeleOpMain extends OpMode{

    /* Declare OpMode members. */
    HardwareDM robot       = new HardwareDM(); // use the class created to define a robot hardware

    // Drivetrain constants when in Cap Ball Mode
    final double         CAP_DRIVE_SPEED         = -1.0;        // Reverse the direction
    final double         CAP_TURN_SPEED          = 1.0;         // Slow down the turns a bit

    // Brake mode status on drive train
    boolean              braked                  = false;       // Not in brake mode initially
    int                  lfBrakedPosn;
    int                  lrBrakedPosn;
    int                  rfBrakedPosn;
    int                  rrBrakedPosn;

    /* Shooter status */
    double               shootSpeed              = robot.SHOOT_DEFAULT;
    static boolean       shootPressed            = false;
    boolean              shooterHot              = false;


    boolean camIsPressedTrue = false;

    // Keep track of the status of the intake
    boolean              intakeStop              = false;
    boolean              intakeIn                = false;    // intake running forward
    boolean              intakeOut               = false;    // intake running backward
    boolean              intakeInPressed         = false;    // Is intake button pressed
    boolean              intakeOutPressed        = false;    // Is intake button pressed
    boolean              intakeTimerOn           = false;
    boolean              intakeJammedTimerOn     = false;
    boolean              intakePausedTimerOn     = false;
    ElapsedTime          intakeTimer             = new ElapsedTime();
    ElapsedTime          intakeJammedTimer       = new ElapsedTime();
    ElapsedTime          intakePausedTimer       = new ElapsedTime();
    int                  previousIntakePos       = 0;
    int                  currentIntakePos        = 0;
    int                  difference              = 0;


    /* Servo current positions */
    double               beaconPos               = robot.BEACON_HOME;
    double               pivotPos                = robot.PIVOT_HOME;
    double               liftDeployPos           = robot.LIFT_DEPLOY_HOME;
    double               capholdPos              = robot.CAPHOLD_HOME;

    boolean              beaconDeployed          = false;
    boolean              pivotDeployed           = false;

    /*  Keep track of whether we have deployed the ball pickup.  Can't move the lift or pivot
        until this is deployed.
     */
    boolean              pickupDeployed          = false;
    boolean              liftMotorUp             = false;
    boolean              liftMotorDown           = false;
    ElapsedTime          pickupDeployTimer       = new ElapsedTime();

    /* Conditions for setting drive train back to normal after cap ball is dropped in vortex

     */

    boolean              liftCap                 = false;
    boolean              capBallDropped          = false;
    boolean              endGameDrive            = false;

    double               shotsMade               = 0;

    // New shooter status variables
    boolean camPaused = false;
    boolean camSwitchPressed = false;
    boolean camStopped = false;
    boolean camReverse = false;
    boolean camReverseStop = false;
    public double REVERSE_TIME = 100;
    ElapsedTime pausedTime = new ElapsedTime();
    ElapsedTime camReverseTimer = new ElapsedTime();




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        RobotLog.i("DM10337 -- Starting TeleOpMain Init.");

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         * Specifically don't need gyro and range finder so skip it to save time.
         */
        robot.init(hardwareMap, false);

        RobotLog.i("DM10337 -- Finished robot.init");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        float g1_right_trigger = gamepad1.right_trigger;
        float g1_left_trigger = gamepad1.left_trigger;
        float g2_right_trigger = gamepad2.right_trigger;
        float g2_left_trigger = gamepad2.left_trigger;
        telemetry.addData("G1 Left Trigger: ", g1_left_trigger);
        telemetry.addData("G1 Right Trigger: ", g1_right_trigger);
        telemetry.addData("G2 Left Trigger: ", g2_left_trigger);
        telemetry.addData("G2 Right Trigger: ", g2_right_trigger);
        telemetry.update();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        RobotLog.i("DM10337 -- Start pressed.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Temporary read of cam switch
        //telemetry.addData("Cam Switch :", robot.camSwitch.isPressed());
        //telemetry.addData("Pos: ", liftDeployPos );

        //if (gamepad1.dpad_up) liftDeployPos += 0.01;
        //if (gamepad1.dpad_down) liftDeployPos -= 0.01;
        //liftDeployPos = Range.clip(liftDeployPos, 0.0, 1.0);
        //robot.liftDeploy.setPosition(liftDeployPos);

        //telemetry.addData("intakePos: ", previousIntakePos);
        //telemetry.addData("currentPos: ", currentIntakePos);
        //telemetry.addData("difference: ", difference);
        //telemetry.addData("Shoot: ", shootSpeed);
        //telemetry.addData("Cam: ", fireCamHot);
        telemetry.addData("Shots: ", shotsMade);
        updateTelemetry(telemetry);


        if (robot.camSwitch.isPressed() && !camIsPressedTrue) {
                shotsMade = shotsMade + 1.0;
                RobotLog.i("DM10334switch -- limit switch pressed! Shots: " + shotsMade);
                camIsPressedTrue = true;
            }
        if (!robot.camSwitch.isPressed()) camIsPressedTrue = false;

        /*
           Driving code -- read joysticks and drive the motors
        */

        //Read thejoysticks -- Y axis is reversed so negate it
        double throttle = -gamepad1.left_stick_y;
        double direction = gamepad1.right_stick_x;

        // Smooth and deadzone the joytick values
        throttle = smoothPowerCurve(deadzone(throttle,0.10));
        direction = smoothPowerCurve(deadzone(direction,0.10));

        // If we deployed into Cap Ball mode the robot drives differently
        if (pickupDeployed && !endGameDrive) {
            // Slow down the turns since we have a cap ball -- and reverse which is front of robot
            throttle = CAP_DRIVE_SPEED * throttle;
            direction = CAP_TURN_SPEED * direction;
        }

        // Calculate the drive motors for left and right
        double right = throttle - direction;
        double left = throttle + direction;
        // clip the right/left values so that the values never exceed +/- 1
        //right = Range.clip(right, -1, 1);
        //left = Range.clip(left, -1, 1);
        // Normalize speeds if any one exceeds +/- 1.0;
        double max = Math.max(Math.abs(right), Math.abs(left));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }


        // If we aren't moving right now check for "brake mode" to hold position
        if ((Math.abs(right) < 0.1) && ((Math.abs(left) < 0.1) && gamepad1.x)) {
            // Requesting brake mode to hold position against defense (e.g. for shooting)
            if (!braked) {
                // First time we see this condition to setup brake mode
                RobotLog.i("DM10337 -- Setting braked mode.");
                braked = true;
                robot.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
                // Record where we are at and set it as motor target to hold
                lfBrakedPosn = robot.lfDrive.getCurrentPosition();
                lrBrakedPosn = robot.lrDrive.getCurrentPosition();
                rfBrakedPosn = robot.rfDrive.getCurrentPosition();
                rrBrakedPosn = robot.rrDrive.getCurrentPosition();
                robot.lfDrive.setTargetPosition(lfBrakedPosn);
                robot.lrDrive.setTargetPosition(lrBrakedPosn);
                robot.rfDrive.setTargetPosition(rfBrakedPosn);
                robot.rrDrive.setTargetPosition(rrBrakedPosn);
                robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
                // Allow up to max power to hold our position
                robot.lfDrive.setPower(1.0);
                robot.rfDrive.setPower(1.0);
                robot.rfDrive.setPower(1.0);
                robot.rrDrive.setPower(1.0);
            } else {
                // already in brake mode -- nothing to do but log if we are having to push
                if (robot.lfDrive.isBusy() || robot.lrDrive.isBusy() ||
                        robot.rfDrive.isBusy() || robot.rrDrive.isBusy()) {
                    RobotLog.i("DM10337 -- Being pushed and fighting back.");
                }
            }
        }


        if (braked && !gamepad1.x) {
            // We are leaving braked mode
            braked = false;
            RobotLog.i("DM10337 -- Leaving brake mode");
            robot.setDriveZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.lfDrive.setPower(0.0);
            robot.lrDrive.setPower(0.0);
            robot.rfDrive.setPower(0.0);
            robot.rrDrive.setPower(0.0);
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (!braked) {
            // Not braked so we can set the motors to power requested by joysticks
            // And lets drive
            robot.lfDrive.setPower(left);
            robot.lrDrive.setPower(left);
            robot.rfDrive.setPower(right);
            robot.rrDrive.setPower(right);
        }

        /*
            Code for the shooter firing cam
         */

        if (gamepad2.right_trigger > 0.25 && !camPaused && pausedTime.milliseconds() > 50) {
            robot.fire.setPower(1.0);
            camStopped = false;

        } else if (gamepad2.right_trigger > 0.25 && camPaused) {
            robot.fire.setPower(0.0);
            camPaused = false;
            pausedTime.reset();
        }


        if (robot.camSwitch.isPressed() && !camPaused && !camSwitchPressed) {
            camSwitchPressed = true;
            camPaused = true;
            pausedTime.reset();
        }

        if (!robot.camSwitch.isPressed()) {
            camSwitchPressed = false;
        }


        if (gamepad2.right_trigger < 0.25) {
            camPaused = false;
            camStopped = true;
            pausedTime.reset();
        }

        if (camStopped && robot.camSwitch.isPressed()) {
            robot.fire.setPower(0.0);
            camSwitchPressed = true;
            camPaused = false;
            camReverse = true;
        }

        if (gamepad2.right_trigger < 0.25 && camReverse) {
            robot.fire.setPower(-0.1);
            camReverseTimer.reset();
            camReverse = false;
            camReverseStop = true;
            REVERSE_TIME = 100;
        }

        if (gamepad2.right_trigger < 0.25 && camReverseStop && camReverseTimer.milliseconds() > REVERSE_TIME)
        {
            robot.fire.setPower(0.0);
            camReverseStop = false;
        }

        if (camStopped && pausedTime.milliseconds() > 1500) {
            robot.fire.setPower(0.0);
            camSwitchPressed = true;
            camPaused = false;
            REVERSE_TIME = 250;
        }



        /*
            Code to adjust the shooter flywheel speed
         */
        // Adjust shooter speed
        if (gamepad2.dpad_down && !shootPressed) {
            // Newly pressed  speed down button
            shootSpeed -= robot.SHOOT_SPEED_INCR;
            shootPressed = true;
            RobotLog.i("DM10337 -- Shooter speed adjusted to " + shootSpeed);
        } else if (gamepad2.dpad_up && !shootPressed) {
            // Newly pressed speed up button
            shootSpeed += robot.SHOOT_SPEED_INCR;
            shootPressed = true;
            RobotLog.i("DM10337 -- Shooter speed adjusted to " + shootSpeed);
        }
        if (shootPressed && !gamepad2.dpad_down && !gamepad2.dpad_up) {
            // Reset flag since no shoot speed adjustment pressed
            shootPressed = false;
        }
        shootSpeed = Range.clip(shootSpeed, 0.0, 1.0);

        /*
            Code for the shooter flywheels
         */
        if (gamepad2.left_trigger <= 0.2) {
            // Stopped when not pressed
            robot.lShoot.setPower(0.0);
            robot.rShoot.setPower(0.0);
            if (shooterHot) {
                //  already running so log stop event
                shooterHot = false;
                RobotLog.i("DM10337 -- Stopping shooter flywheels");
            }

        } else if (gamepad2.left_trigger > 0.2) {
            // Running when pressed
            robot.lShoot.setPower(shootSpeed);
            robot.rShoot.setPower(shootSpeed);
            if (!shooterHot) {
                // Was not running before so log stop event
                shooterHot = true;
                RobotLog.i("DM10337 -- Starting shooter flywheels");
            }
        }




        /*
            Beacon pusher code
         */
        if (gamepad1.right_bumper) {
            // Pressed so deploy the beacon pusher
            beaconPos = robot.BEACON_MAX_RANGE;
            if (!beaconDeployed) {
                // Newly pressed so log beacon deploy
                beaconDeployed = true;
                RobotLog.i("DM10337 -- Deploying beacon presser");
            }
        } else {
            // Not pressed so retract it
            beaconPos = robot.BEACON_MIN_RANGE;
            if (beaconDeployed) {
                // Was deployed so log beacon withdrawal
                beaconDeployed = false;
                RobotLog.i("DM10337 == Beacon presser off");

            }
        }

        // Set the beacon pusher
        beaconPos = Range.clip(beaconPos, robot.BEACON_MIN_RANGE, robot.BEACON_MAX_RANGE);
        robot.beacon.setPosition(beaconPos);

        /*
            Cap ball forks deployment code.  Keep track of whether the cap ball list if deployed.
            For safety, both drivers have to press a button simultaneously to deploy!
            We will use a timer to make sure we don't try and move it too quickly, to propect
            hardware from damage.  Cap ball lift and pivot are disabled until after this timer expires.
         */
        if (gamepad1.left_bumper && gamepad2.left_bumper) {
            if (pickupDeployed == false) {
                // First time we are trying to deploy
                pickupDeployed = true;
                pickupDeployTimer.reset();      // Set timer of how long to wait

                // Deploy the cap ball lift forks
                robot.liftDeploy.setPosition(robot.LIFT_DEPLOY_MIN_RANGE);

                RobotLog.i("DM10337 -- Deploying the cap ball lift forks");
            }
        }

        /*
            Code for the cap ball lift.  It is disabled until lift forks deployed and enough time
            has elapsed to safely move it.
         */
        if (pickupDeployed == true && pickupDeployTimer.milliseconds() > robot.DEPLOY_WAIT) {
            // The cap ball lift mechanism is ready to go!

            // Process the pivot servo
            if (gamepad1.left_bumper) {
                // Pressed stick so pivot the lift down
                pivotPos = robot.PIVOT_MAX_RANGE;

                // lift has been raised, so pivot forward must be capping cap ball.
                if (liftCap) capBallDropped = true;
                if (!pivotDeployed) {
                    pivotDeployed = true;
                    RobotLog.i("DM10337 -- Pivoting the Lift Down");
                }
            } else {
                // Not pressed so pivot it to the up (default) position
                pivotPos = robot.PIVOT_MIN_RANGE;
                if (pivotDeployed) {
                    pivotDeployed = false;
                    RobotLog.i("DM10337 -- Pivoting the Lift Up");
                }
            }

            // Process for moving cap ball holder. Y deploys cap holder. B releases cap holder.
            if (gamepad1.y || gamepad2.y) {
                capholdPos = robot.CAPHOLD_DEPLOY_MAX_RANGE;
                RobotLog.i("DM10337 -- Cap Hold Deployed");
            }
            if (gamepad1.b || gamepad2.b) {
                capholdPos = robot.CAPHOLD_DEPLOY_MIN_RANGE;
                RobotLog.i("DM10337 -- Cap Hold Released");
            }

            // For safety verify pivot and cap hold servo positions and then move them
            pivotPos = Range.clip(pivotPos, robot.PIVOT_MIN_RANGE, robot.PIVOT_MAX_RANGE);
            robot.pivot.setPosition(pivotPos);
            capholdPos = Range.clip(capholdPos, robot.CAPHOLD_DEPLOY_MIN_RANGE, robot.CAPHOLD_DEPLOY_MAX_RANGE);
            robot.caphold.setPosition(capholdPos);

            // And process the lift motor
            if ((gamepad2.right_stick_y < -0.2) && (!robot.liftLimit.isPressed())) {
                // Move cap ball holder out of the way when lifting
                capholdPos = robot.CAPHOLD_DEPLOY_MIN_RANGE;
                liftCap = true;
                RobotLog.i("DM10337 -- Cap Hold Released");
                // Lift it up
                robot.liftMotor.setPower(robot.LIFT_UP_SPEED);
                if (!liftMotorUp) {
                    // We weren't going up before so log event
                    liftMotorUp = true;
                    liftMotorDown = false;
                    RobotLog.i("DM10337 -- Cap Ball Lift moving up");
                }
            } else if (gamepad2.right_stick_y > 0.2 && !gamepad2.left_bumper) {
                // Or drop it down
                robot.liftMotor.setPower(robot.LIFT_DOWN_SPEED);
                if (!liftMotorDown) {
                    // We weren't moving down before so log event
                    liftMotorDown = true;
                    liftMotorUp = false;
                    RobotLog.i("DM10337 -- Cap Ball Lift moving down");
                }

            } else if (gamepad2.right_stick_y > 0.2 && gamepad2.left_bumper) {
                // Or drop it down
                robot.liftMotor.setPower(robot.LIFT_DOWN_SPEED_FAST);
                if (!liftMotorDown) {
                    // We weren't moving down before so log event
                    liftMotorDown = true;
                    liftMotorUp = false;
                    RobotLog.i("DM10337 -- Cap Ball Lift moving down FAST");
                }

            } else robot.liftMotor.setPower(0.0);

            if (gamepad1.dpad_down && capBallDropped) endGameDrive = true;

        }


        /*
            Code for the ball intake
         */
        if (gamepad1.left_trigger > 0.6) {
            // Pressing intake reverse button
            if (!intakeOutPressed) {
                // Haven't read this button press yet so process it
                intakeOutPressed = true;
                // Not already in reverse so set it so
                    robot.intake.setPower(robot.INTAKE_OUT_SPEED);
                    intakeStop = false;
                    intakeTimerOn = false;
                    intakeOut = true;
                    intakeIn = false;
                    intakeJammedTimerOn = false;
                    intakePausedTimerOn = false;
                    RobotLog.i("DM10337 -- Intake start reverse");
                }
            }
        else {
            // Intake reverse button is not pressed
            intakeOutPressed = false;
        }
        if (gamepad1.right_trigger > 0.6) {
            // Pressing intake forward button
            if (!intakeInPressed) {
                // Haven't read this button press yet so process it
                intakeInPressed = true;
                } else {
                    // Not already in forward so set it so
                    robot.intake.setPower(robot.INTAKE_IN_SPEED);
                    intakeStop = false;
                    intakeTimerOn = false;
                    intakeOut = false;
                    intakeIn = true;
                    intakeJammedTimerOn = false;
                    intakePausedTimerOn = false;
                    RobotLog.i("DM10337 -- Intake start forward");
                }
            }
        else {
            // Intake reverse button is not pressed
            intakeInPressed = false;
        }

        /*
        if (intakeStop) {

            curPos = robot.intake.getCurrentPosition();
            remainder = curPos % 420;
            newPos = curPos + (420 - remainder);
            robot.intake.setTargetPosition(newPos);
            robot.intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeStop = false;
        }
        */

        if (gamepad1.a) {
            // Stop intake
            intakeIn = false;
            intakeOut = false;
            intakeTimerOn = false;
            intakeJammedTimerOn = false;
            intakeStop = true;
            robot.intake.setPower(0.0);
            //RobotLog.i("DM10337 intake stopped");
        }

        // Turn intake timer on at the start of intaking
        if (intakeIn && !intakeTimerOn) {
            previousIntakePos = robot.intake.getCurrentPosition();
            intakeTimerOn = true;
            intakeTimer.reset();
        }

        // Check rotation of intake with encoders after 1750 ms
        if (intakeIn && intakeTimerOn && intakeTimer.milliseconds() > 1050) {
            intakeTimerOn = false;
            currentIntakePos = robot.intake.getCurrentPosition();
            difference = Math.abs(Math.abs(previousIntakePos) - Math.abs(currentIntakePos));
            previousIntakePos = robot.intake.getCurrentPosition();

            // If intake has slowed down to near stall or stalled due to jam, reverse intake and start timer for clearing jam
            if (difference < 1050) {
                robot.intake.setPower(robot.INTAKE_OUT_SPEED);
                intakeOut = true;
                intakeIn = false;
                intakePausedTimerOn = true;
                intakePausedTimer.reset();
                RobotLog.i("DM10337 -- Intake JAMMED! Reversing! Difference: " + difference);
            }

        }

        // Stop intake before reversing direction rotation.

        if (intakeOut && intakePausedTimerOn && intakePausedTimer.milliseconds() > 350) {
            robot.intake.setPower(0.0);
            intakeOut = false;
            intakeIn = false;
            intakeJammedTimerOn = true;
            intakeJammedTimer.reset();
            RobotLog.i("DM10337 -- Intake JAMMED! Done waiting");

        }

        // Clearing jam complete, so return to intaking
        if (!intakeOut && !intakeIn && intakeJammedTimerOn && intakeJammedTimer.milliseconds() > 250) {
            robot.intake.setPower(robot.INTAKE_IN_SPEED);
            intakeOut = false;
            intakeIn = true;
            intakeJammedTimerOn = false;
            RobotLog.i("DM10337 -- Intake done reversing. Returning to intake. Difference: " + difference);
        }




    }


    /*
     * Code to run ONCE after the driver hits STOP.  Make all motion stops
     */
    @Override
    public void stop() {
        robot.lfDrive.setPower(0.0);
        robot.lrDrive.setPower(0.0);
        robot.rfDrive.setPower(0.0);
        robot.rrDrive.setPower(0.0);
        robot.lShoot.setPower(0.0);
        robot.rShoot.setPower(0.0);
        robot.intake.setPower(0.0);
        robot.liftMotor.setPower(0.0);
        robot.fire.setPower(0.0);
        RobotLog.i("Teleop Stop Pressed");
    }


    /**
     * This does the cubic smoothing equation on joystick value.
     * Assumes you have already done any deadzone processing.
     *
     * @param x  joystick input
     * @return  smoothed value
     */
    protected double smoothPowerCurve (double x) {
        //double a = this.getThrottle();
        double a = 1.0;         // Hard code to max smoothing
        double b = 0.05;		// Min power to overcome motor stall

        if (x > 0.0)
            return (b + (1.0-b)*(a*x*x*x+(1.0-a)*x));

        else if (x<0.0)
            return (-b + (1.0-b)*(a*x*x*x+(1.0-a)*x));
        else return 0.0;
    }

    /**
     * Add deadzone to a stick value
     *
     * @param rawStick  Raw value from joystick read -1.0 to 1.0
     * @param dz	Deadzone value to use 0 to 0.999
     * @return		Value after deadzone processing
     */
    protected double deadzone(double rawStick, double dz) {
        double stick;

        // Force limit to -1.0 to 1.0
        if (rawStick > 1.0) {
            stick = 1.0;
        } else if (rawStick < -1.0) {
            stick = -1.0;
        } else {
            stick = rawStick;
        }

        // Check if value is inside the dead zone
        if (stick >= 0.0){
            if (Math.abs(stick) >= dz)
                return (stick - dz)/(1 -  dz);
            else
                return 0.0;

        }
        else {
            if (Math.abs(stick) >= dz)
                return (stick + dz)/(1 - dz);
            else
                return 0.0;

        }
    }

    /**
     * Start the firing cam
     */
    public void startFireCam() { robot.fire.setPower(1.0);
    }

    /**
     * Stop the firing cam
     * */
    public void stopFireCam() {
        robot.fire.setPower(0.0);
    }
}
