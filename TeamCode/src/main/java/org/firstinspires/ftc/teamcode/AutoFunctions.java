/*
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

import android.app.Activity;
import android.graphics.Color;
import android.hardware.Sensor;
import android.util.Log;
import android.view.View;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.InstantRunDexHelper;

import java.text.DecimalFormat;


@Autonomous(name="AutoFunctions", group="Main")
@Disabled
public class AutoFunctions extends LinearOpMode {
    /* Declare OpMode members. */

    // Device declarations
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor leftFly = null;
    DcMotor rightFly = null;
    DcMotor elevator = null;
    Servo handFront = null;
    Servo handBack = null;

    TouchSensor touchSensor = null;
    TouchSensor elevatorTouch = null;
    OpticalDistanceSensor odsSensor;
    OpticalDistanceSensor sharpIR = null;
    ColorSensor colorSensor = null;
    UltrasonicSensor ultra = null;

    /* This is the port on the Core Device Interface Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
    private final int NAVX_DIM_I2C_PORT = 1;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 1.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private boolean calibration_complete = false;
    boolean navxConnected = true;

    navXPIDController.PIDResult yawPIDResult;
    DecimalFormat df;

    static double odsStart;

    double odsMax;
    double leftTurn;
    double rightTurn;
    boolean odsTurn;

    public void runOpMode() throws InterruptedException {
        // get motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftFly = hardwareMap.dcMotor.get("leftFly");
        rightFly = hardwareMap.dcMotor.get("rightFly");
        elevator = hardwareMap.dcMotor.get("elevator");
        handFront = hardwareMap.servo.get("handFront");
        handBack = hardwareMap.servo.get("handBack");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        leftFly.setDirection(DcMotor.Direction.REVERSE);

        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        ultra = hardwareMap.ultrasonicSensor.get("ultra");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        elevatorTouch = hardwareMap.touchSensor.get("elevatorTouch");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);


        odsStart = odsSensor.getLightDetected();

        colorSensor.enableLed(false);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        waitForStart();

        idle();
    }
    public void updateColor() throws InterruptedException {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
    }
    public void autoFire() throws InterruptedException {
        // drive to center to fire
        leftMotor.setPower(-1);
        rightMotor.setPower(-1);
        telemetry.addData("AutoStatus: ", "Moving to center");
        telemetry.update();
        leftFly.setPower(.5);
        rightFly.setPower(.5);
        sleep(500);

        leftFly.setPower(.95);
        rightFly.setPower(.95);
        elevator.setPower(1);
        sleep(500);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        telemetry.addData("Fire status: ", "Firing");
        telemetry.update();
        sleep(2500);

        elevator.setPower(0);
        telemetry.addData("Fire status: ", "Powered down");
        telemetry.update();
        sleep(50);

    }
    public void autoFireLong() throws InterruptedException {
        // drive to center to fire
        leftMotor.setPower(-1);
        rightMotor.setPower(-1);
        telemetry.addData("AutoStatus: ", "Moving to center");
        telemetry.update();
        leftFly.setPower(.5);
        rightFly.setPower(.5);
        sleep(500);

        leftFly.setPower(.95);
        rightFly.setPower(.95);
        elevator.setPower(1);
        sleep(1000);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        telemetry.addData("Fire status: ", "Firing");
        telemetry.update();
        sleep(2000);

        elevator.setPower(0);
        telemetry.addData("Fire status: ", "Powered down");
        telemetry.update();
        sleep(50);

    }
    public void autoFire2() throws InterruptedException {
        leftFly.setPower(.95);
        rightFly.setPower(.95);
        elevator.setPower(1);
        telemetry.addData("Fire status: ", "Firing #2");
        telemetry.update();
        sleep(3000);

        elevator.setPower(0);
        telemetry.addData("Fire status: ", "Powered down");
        telemetry.update();
        sleep(10);
    }
    public void elevatorDownDrive(double driveSpeed, double leftTurn, double rightTurn) throws InterruptedException {
        runtime.reset();
        while (opModeIsActive() && !elevatorTouch.isPressed()) {
            elevator.setPower(-.75);
            idle();
            while(opModeIsActive() && runtime.time() < .75) {
                leftMotor.setPower(driveSpeed);
                rightMotor.setPower(driveSpeed);
                telemetry.addData("AutoStatus: ", "Backing up");
                telemetry.update();
                idle();
            }
            while (opModeIsActive() && runtime.time() > .75 && runtime.time() < 1.25) {
                leftMotor.setPower(leftTurn);
                rightMotor.setPower(rightTurn);
                telemetry.addData("AutoStatus: ", "Turning towards beacon");
                telemetry.update();
                idle();
            }
            while (opModeIsActive() && runtime.time() > 1.25 && runtime.time() < 1.5) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                idle();
            }
        }
    }
    public void elevatorDown() throws InterruptedException {
        while (opModeIsActive() && !elevatorTouch.isPressed()) {
            elevator.setPower(-1);
            idle();
        }
    }
    public void findWhite() throws InterruptedException {
        runtime.reset();
        while (opModeIsActive() && (odsSensor.getLightDetected() < (odsStart * 3))) {
            leftMotor.setPower(-.5 + (runtime.time() / 100));
            rightMotor.setPower(-.5);
            telemetry.addData("Follow Status", "Finding white line");
            telemetry.addData("Light:", odsSensor.getLightDetected());
            telemetry.addData("White line", odsStart * 3);
            //updateColor();
            telemetry.update();
            idle();
        }

    }
    public void findWhiteStraight() throws InterruptedException {
        while (opModeIsActive() && (odsSensor.getLightDetected() < (odsStart * 3))) {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, 500)) {
                if (yawPIDResult.isOnTarget()) {
                    leftMotor.setPower(-.4);
                    rightMotor.setPower(-.4);
                    telemetry.addData("PIDOutput", df.format(-.4) + ", " +
                            df.format(-.4));
                } else {
                    double output = yawPIDResult.getOutput();
                    leftMotor.setPower(-.4 - 3 * output);
                    rightMotor.setPower(-.4 + 3 * output);
                    telemetry.addData("PIDOutput", df.format(limit(-.4 + output)) + ", " +
                            df.format(limit(-.4 - output)));
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
            } else {
                // A timeout occurred
                navxConnected = false;
                Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }
    }
    public void turnToWhite(double leftSpeed, double rightSpeed) throws InterruptedException {
        while (opModeIsActive() & (odsSensor.getLightDetected() < 3*odsStart)) {
            leftMotor.setPower(leftSpeed);
            rightMotor.setPower(rightSpeed);
            telemetry.addData("Follow Status", "Turning to White");
            telemetry.addData("Light:", odsSensor.getLightDetected());
            telemetry.addData("White line", odsStart * 3);
            telemetry.update();
            idle();
        }
    }
    public void stayWhiteBlueOld() throws InterruptedException {
        runtime.reset();
        while (opModeIsActive() && !touchSensor.isPressed() && (runtime.time() < 1.5)) {
            // follow white line
            while (opModeIsActive() && odsSensor.getLightDetected() > (3 * odsStart)) {
                leftMotor.setPower(-.25);
                rightMotor.setPower(-.25);
                telemetry.addData("Follow Status", "Driving on line");
                telemetry.addData("Light", odsSensor.getLightDetected());
                telemetry.addData("White line", odsStart * 3);
                telemetry.addData("Touch:", touchSensor.getValue());
                //updateColor();
                telemetry.update();
                idle();
            }
            while (opModeIsActive() && odsSensor.getLightDetected() < (3 * odsStart)){
                leftMotor.setPower(.15);
                rightMotor.setPower(-.15);
                telemetry.addData("Follow Status", "Turning to line");
                telemetry.addData("White line", odsStart * 3);
                telemetry.addData("Light", odsSensor.getLightDetected());
                telemetry.addData("Touch:", touchSensor.getValue());
                //updateColor();
                telemetry.update();
                idle();
            }
            idle();
        }
    }
    public void stayWhiteRedOld() throws InterruptedException {
        runtime.reset();
        while (opModeIsActive() && !touchSensor.isPressed() && (runtime.time() < 1.5)) {
            // follow white line
            while (opModeIsActive() && odsSensor.getLightDetected() > (3 * odsStart)) {
                leftMotor.setPower(-.25);
                rightMotor.setPower(-.25);
                telemetry.addData("Follow Status", "Driving on line");
                telemetry.addData("Light", odsSensor.getLightDetected());
                telemetry.addData("White line", odsStart * 3);
                telemetry.addData("Touch:", touchSensor.getValue());
                //updateColor();
                telemetry.update();
                idle();
            }
            while (opModeIsActive() && odsSensor.getLightDetected() < (3 * odsStart)){
                leftMotor.setPower(-.15);
                rightMotor.setPower(.15);
                telemetry.addData("Follow Status", "Turning to line");
                telemetry.addData("White line", odsStart * 3);
                telemetry.addData("Light", odsSensor.getLightDetected());
                telemetry.addData("Touch:", touchSensor.getValue());
                //updateColor();
                telemetry.update();
                idle();
            }
            idle();
        }
    }
    public void stayWhiteBlueAdvanced() throws InterruptedException {
        runtime.reset();
        odsMax = odsSensor.getLightDetected();
        odsTurn = false;
        sleep(50);
        while (opModeIsActive() && !touchSensor.isPressed() && (runtime.time() < 5)) {
            if (odsSensor.getLightDetected() >= .9 * odsMax) {
                odsMax = odsSensor.getLightDetected();
            } else if (odsSensor.getLightDetected() < .9 * odsMax) {
                odsTurn = !odsTurn;
                telemetry.addData("Follow Status", "Turning to line");
                telemetry.addData("OdsMax: ", odsMax);
                telemetry.addData("Light", odsSensor.getLightDetected());
                telemetry.addData("Touch:", touchSensor.getValue());
            }

            if (odsTurn) {
                leftMotor.setPower(-.10);
                rightMotor.setPower(.10);
            }
            else {
                leftMotor.setPower(.10);
                rightMotor.setPower(-.10);
            }
        }
    }
    public void stayWhiteRedAdvanced() throws InterruptedException {
        runtime.reset();
        odsMax = odsSensor.getLightDetected();
        odsTurn = true;
        sleep(50);
        while (opModeIsActive() && !touchSensor.isPressed() && (runtime.time() < 5)) {
            if (odsSensor.getLightDetected() >= .9 * odsMax) {
                odsMax = odsSensor.getLightDetected();
            } else if (odsSensor.getLightDetected() < .9 * odsMax) {
                odsTurn = !odsTurn;
            }

            if (odsTurn) {
                leftMotor.setPower(-.10);
                rightMotor.setPower(.10);
            }
            else {
                leftMotor.setPower(.10);
                rightMotor.setPower(.10);
            }
        }
    }
    public void moveElevator(double power, double timeToRun) throws InterruptedException {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeToRun)) {
            elevator.setPower(power);
            idle();
        }
    }
    public void runForTime(double leftSpeed, double rightSpeed, long timeToRun) throws InterruptedException {
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
        sleep(timeToRun);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        idle();
    }
    public void scanWhite() throws InterruptedException {
        // move left while scanning
        while (opModeIsActive() && odsSensor.getLightDetected() < (odsStart * 1.5)) {
            // scan while turning left
            runForTime(-.1, 0, 1000);
            runForTime(0, -.1, 1000);
            runForTime(-.1, -.1, 500);
            idle();
        }
    }
    public void driveStraightTimed(double drive_speed, double driveTime) throws InterruptedException {
        while (opModeIsActive() && (runtime.time() < driveTime)) {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, 500)) {
                if (yawPIDResult.isOnTarget()) {
                    leftMotor.setPower(drive_speed);
                    rightMotor.setPower(drive_speed);
                    telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
                            df.format(drive_speed));
                } else {
                    double output = yawPIDResult.getOutput();
                    leftMotor.setPower(drive_speed - 3 * output);
                    rightMotor.setPower(drive_speed + 3 * output);
                    telemetry.addData("PIDOutput", df.format(limit(drive_speed + output)) + ", " +
                            df.format(limit(drive_speed - output)));
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
            } else {
                // A timeout occurred
                navxConnected = false;
                Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }
    }
    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }
    public void declareDevices() throws InterruptedException {
        // Device declarations
        DcMotor leftMotor = null;
        DcMotor rightMotor = null;
        DcMotor elevator = null;
        DcMotor leftFly = null;
        DcMotor rightFly = null;
        Servo handFront = null;
        Servo handBack = null;

        TouchSensor touchSensor = null;
        TouchSensor elevatorTouch = null;
        OpticalDistanceSensor odsSensor;
        ColorSensor colorSensor = null;
        OpticalDistanceSensor sharpIR = null;
        UltrasonicSensor ultra = null;


        // This is the port on the Core Device Interface Module
        // in which the navX-Model Device is connected.  Modify this
        // depending upon which I2C port you are using.
        final int NAVX_DIM_I2C_PORT = 1;
        AHRS navx_device;
        navXPIDController yawPIDController;
        ElapsedTime runtime = new ElapsedTime();

        final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

        final double TARGET_ANGLE_DEGREES = 0.0;
        final double TOLERANCE_DEGREES = 1.0;
        final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
        final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
        final double YAW_PID_P = 0.005;
        final double YAW_PID_I = 0.0;
        final double YAW_PID_D = 0.0;

        boolean calibration_complete = false;
        boolean navxConnected = true;

        navXPIDController.PIDResult yawPIDResult;
        DecimalFormat df;


        double odsMax;
        double leftTurn;
        double rightTurn;
        boolean odsTurn;
    }
    public void declareMap() throws InterruptedException {
        // get motors
        //     Drive motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        //     Fly wheels
        leftFly = hardwareMap.dcMotor.get("leftFly");
        rightFly = hardwareMap.dcMotor.get("rightFly");
        //     Elevator motor
        elevator = hardwareMap.dcMotor.get("elevator");

        // Servos
        handFront = hardwareMap.servo.get("handFront");
        handBack = hardwareMap.servo.get("handBack");

        {
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
        // Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        // Configure the PID controller
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);
        }


        // Sensors
        //     Light sensor
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        //     Ultrasonic Sensor
        ultra = hardwareMap.ultrasonicSensor.get("ultra");
        //     Touch sensor
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        //     Elevator touch
        elevatorTouch = hardwareMap.touchSensor.get("elevatorTouch");
        //     Color sensor
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        //     Ir Sensor
        sharpIR = hardwareMap.opticalDistanceSensor.get("infrared");


        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFly.setDirection(DcMotor.Direction.REVERSE);

        odsStart = odsSensor.getLightDetected();
    }
}