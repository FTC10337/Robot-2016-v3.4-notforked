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
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 *
 * This class defines all the hardware for Dark Matter 10337 FTC robot for 2016-17 season
 *
 *
 */
public class HardwareDM
{
    /* Public OpMode members. */

    /* Drive train motors */
    public DcMotor  lfDrive = null;
    public DcMotor  lrDrive = null;
    public DcMotor  rfDrive = null;
    public DcMotor  rrDrive = null;

    /* Shooter motors */
    public DcMotor lShoot = null;
    public DcMotor rShoot = null;

    /*  Intake motor */
    public DcMotor intake = null;

    /* Lift motor */
    public DcMotor liftMotor = null;

    /* Shooter feed cam */
    public CRServo fire = null;

    /* beacon and pivot servos */
    public Servo beacon = null;
    public Servo pivot = null;
    public Servo liftDeploy = null;
    public Servo caphold = null;

    /* lift limit switch - touch sensor */
    public TouchSensor liftLimit = null;

    /* firing cam sensor switch */
    public TouchSensor camSwitch = null;

    // Ultrasonic range sensor
    public ModernRoboticsI2cRangeSensor rangeSensor;

    // Adafruit IMU gyro & motion sensor object
    AdafruitBNO055IMU adaGyro;

    /* Adafruit RGB Sensor */
    public ColorSensor beaconColor;
    // Device interface module (for controlling Adafruit RGB sensor LED)
    DeviceInterfaceModule cdim;
    static final int LED_CHANNEL = 0;

    // Modern Robotics Color sensor on bottom
    public ColorSensor stripeColor;

    // Shooter default speed
    public final static double SHOOT_DEFAULT = 0.925;
    public final static double SHOOT_SPEED_INCR = 0.005;

    // Lift motor speeds
    public final static double LIFT_UP_SPEED = 1.0;
    public final static double LIFT_DOWN_SPEED = -0.2;
    public final static double LIFT_DOWN_SPEED_FAST = -0.50;

    // Intake motor speeds
    public final static double INTAKE_IN_SPEED = 1.0;
    public final static double INTAKE_OUT_SPEED = -1.0;

    /* Shooter constants */
    static final int     COUNTS_PER_MOTOR_REV    = 7 ;    // Neverrest w/ BaneBots 4:1
    static final int     NR_MAX_RPM              = 6600;  // NeverRest max RPM
    static final int     SHOOT_MAX_RPM           = NR_MAX_RPM * COUNTS_PER_MOTOR_REV;

    /* Drive train constants */
    static final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (4 * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // Servo max min ranges
    public final static double PIVOT_HOME = 0.025;
    public final static double BEACON_HOME = 0.22;
    public final static double CAPHOLD_HOME = 0.0;
    public final static double LIFT_DEPLOY_HOME = 0.27;
    public final static double PIVOT_MIN_RANGE  = 0.025;
    public final static double PIVOT_MAX_RANGE  = 0.55;
    public final static double BEACON_MIN_RANGE  = 0.22;
    public final static double BEACON_MAX_RANGE  = 0.82;
    public final static double LIFT_DEPLOY_MIN_RANGE = 0.0;
    public final static double LIFT_DEPLOY_MAX_RANGE = 0.27;
    public final static double CAPHOLD_DEPLOY_MIN_RANGE = 0.0;
    public final static double CAPHOLD_DEPLOY_MAX_RANGE = 0.80;

        // How long to wait for cap lift forks to deploy
    public final static double DEPLOY_WAIT = 150;  // Wait 1 second for ball pickup to deploy

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /**
     *
     *   Constructor -- no work to do here.  Use init method for hardware setup
     */
    public HardwareDM() {
    }


    /**
     * Abbreviated call to init w/o gyro functionality
     *
     * @param ahwMap
     */
    public void init(HardwareMap ahwMap) {
        init(ahwMap, true);
        // Assume using the gyro unless OpMode explicitly doesn't need it
        // TeleOp is the likely non-user
    }


    /**
     *
     * @param ahwMap    HardwareMap to use to find all of our hardware
     *
     * Does all initialization for all of our robot hardware to default configurations.
     *
     */
    public void init(HardwareMap ahwMap, boolean useGyroRange) {
        // save reference to HW Map

        RobotLog.i("DM10337 -- Starting HardwareDM Init");

        hwMap = ahwMap;

        // Only setup the IMU if we are going to need it to save time
        if (useGyroRange) {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            // Do this first as it takes the longest to finish
            AdafruitBNO055IMU.Parameters parameters = new AdafruitBNO055IMU.Parameters();
            parameters.angleUnit           = AdafruitBNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = AdafruitBNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "adaGyro".
            adaGyro = hwMap.get(AdafruitBNO055IMU.class, "gyro");
            adaGyro.initialize(parameters);

            RobotLog.i("DM10337 -- Gyro modes set and gyro initialized.");

        }


        // Define and Initialize Motors
        lfDrive   = hwMap.dcMotor.get("lf motor");
        lrDrive   = hwMap.dcMotor.get("lr motor");
        rfDrive   = hwMap.dcMotor.get("rf motor");
        rrDrive   = hwMap.dcMotor.get("rr motor");
        lShoot    = hwMap.dcMotor.get("l shoot");
        rShoot    = hwMap.dcMotor.get("r shoot");
        intake    = hwMap.dcMotor.get("intake");
        liftMotor = hwMap.dcMotor.get("lift");

        // Define and initialize servos
        fire = hwMap.crservo.get("fire");
        beacon = hwMap.servo.get("beacon");
        pivot = hwMap.servo.get("pivot");
        liftDeploy = hwMap.servo.get("lift deploy");
        caphold = hwMap.servo.get("cap hold");

        beacon.setPosition(BEACON_HOME);
        pivot.setPosition(PIVOT_HOME);
        liftDeploy.setPosition(LIFT_DEPLOY_HOME);
        caphold.setPosition(CAPHOLD_HOME);

        // Define touch sensors
        liftLimit = hwMap.touchSensor.get("ts");
        camSwitch = hwMap.touchSensor.get("cs");

        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");


        RobotLog.i("DM10337 -- Finished mapping all hardware.");

        // Set all motors to zero power
        lfDrive.setPower(0.0);
        lrDrive.setPower(0.0);
        rfDrive.setPower(0.0);
        rrDrive.setPower(0.0);
        lShoot.setPower(0.0);
        rShoot.setPower(0.0);
        fire.setPower(0.0);
        intake.setPower(0.0);

        // Set all motors to run with or without encoders
        //
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor directions
        setDriveDirection(DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE);
        lShoot.setDirection(DcMotor.Direction.FORWARD);
        rShoot.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to float mode to protect gearboxes from shock load
        setDriveZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);
        lShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake and lift can be brake mode to hold positon better
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RobotLog.i("DM10337 -- Motor modes set.");

        // Retrieve and initialize the Adafruit color sensor
        beaconColor = hwMap.colorSensor.get("color");
        cdim = hwMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_CHANNEL, false); // Turn RGB light off -- never needs to be on

        RobotLog.i("DM10337 -- Adafruit color sensor setup.");

        // And also for the MR color sensor
        stripeColor = hwMap.colorSensor.get("stripe");
        stripeColor.enableLed(false);

        RobotLog.i("DM10337 -- MR color sensor setup.");
    }

    /**
     *
     * @param mode  RunMode to set the drive train to (e.g. w/ or w/o encoders)
     *
     * Sets all drive train motors to the designated mode
     */
    public void setDriveMode(DcMotor.RunMode mode) {
        setDriveFrontMode(mode);
        setDriveRearMode(mode);
    }

    /**
     *
     * @param mode  RunMode to set the drive train to (e.g. w/ or w/o encoders)
     *
     * Sets front drive train motors to the designated mode
     */
    public void setDriveFrontMode(DcMotor.RunMode mode){
        lfDrive.setMode(mode);
        rfDrive.setMode(mode);
    }

    /**
     *
     * @param mode  RunMode to set the drive train to (e.g. w/ or w/o encoders)
     *
     * Sets rear drive train motors to the designated mode
     */
    public void setDriveRearMode(DcMotor.RunMode mode){
        lrDrive.setMode(mode);
        rrDrive.setMode(mode);
    }

    /**
     *
     * @param behavior  ZeroPower Behavior to set on motors -- brake or float
     *
     * Sets all drive train motors to float or brake mode as directed
     *
     */
    public void setDriveZeroPower(DcMotor.ZeroPowerBehavior behavior) {
        setDriveFrontZeroPower(behavior);
        setDriveRearZeroPower(behavior);
    }

    /**
     *
     * @param behavior  ZeroPower Behavior to set on motors -- brake or float
     *
     * Sets front drive train motors to float or brake mode as directed
     *
     */
    public void setDriveFrontZeroPower(DcMotor.ZeroPowerBehavior behavior) {
        lfDrive.setZeroPowerBehavior(behavior);
        rfDrive.setZeroPowerBehavior(behavior);
    }

    /**
     *
     * @param behavior  ZeroPower Behavior to set on motors -- brake or float
     *
     * Sets rear drive train motors to float or brake mode as directed
     *
     */
    public void setDriveRearZeroPower(DcMotor.ZeroPowerBehavior behavior) {
        lrDrive.setZeroPowerBehavior(behavior);
        rrDrive.setZeroPowerBehavior(behavior);
    }

    /**
     *
     * @param leftDir       DcMotor.Direction for left side of drive train
     * @param rightDir      DcMotor.Direction for right side of drive train
     *
     * Sets drive train motor directions.
     */
    public void setDriveDirection(DcMotor.Direction leftDir, DcMotor.Direction rightDir) {
        lrDrive.setDirection(leftDir);
        lfDrive.setDirection(leftDir);
        rrDrive.setDirection(rightDir);
        rfDrive.setDirection(rightDir);
    }



}
