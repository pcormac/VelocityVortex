package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

@Autonomous(name="Auto: Smash Blue Advanced", group="Main")
public class AutoSmashBlueAdvanced extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    VoltageSensor driveController;
    // This is the port on the Core Device Interface Module
    // in which the navX-Model Device is connected.  Modify this
    // depending upon which I2C port you are using.
    final int NAVX_DIM_I2C_PORT = 0;
    AHRS navx_device;
    navXPIDController yawPIDController;

    final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    double TARGET_ANGLE_DEGREES = 0.0;
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


    static double odsStart;

    public void runOpMode() throws InterruptedException {

        declareMap();

        driveController = hardwareMap.voltageSensor.get("Drive");
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
        leftFly.setDirection(DcMotor.Direction.REVERSE);
        colorSensor.enableLed(false);

        handFront.setPosition(.5);

        while (!isStopRequested() && !isStarted() && navx_device.isCalibrating()) {
            telemetry.addData("Navx: ", "Calibrating");
            telemetry.addData("Calibration Time: ", time);
            telemetry.update();
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.addData("White line", odsStart * 3);
        telemetry.update();

        waitForStart();

        runtime.reset();

        if (driveController.getVoltage() < 13) {
            runForTime(-1, -1, 650);
        } else {
            runForTime(-1, -1, 450);
        }

        telemetry.addData("AutoStatus: ", "Moving to center");
        telemetry.update();

        // fire first ball
        telemetry.addData("AutoStatus: ", "Firing first ball");
        telemetry.update();
        autoFire();
        elevatorDown();

        // load second ball
        telemetry.addData("AutoStatus: ", "Loading second ball");
        telemetry.update();
        handFront.setPosition(1);
        sleep(750);

        // fire second ball
        telemetry.addData("AutoStatus: ", "Firing second ball");
        telemetry.update();
        autoFire2();
        elevatorDown();


        if (driveController.getVoltage() < 13 && driveController.getVoltage() >12.75) {
            runForTime(.45, -.45, 550);
        } else if (driveController.getVoltage() > 13) {
            runForTime(.45, -.45, 450);
        }  else {
            runForTime(.45, - .45, 625);
        }


        /*
        if (!navx_device.isCalibrating()) {
            navx_device.zeroYaw();
            findWhiteStraight();
        } else {
            findWhite();
        }
        */
        findWhite();

        runForTime(-.25, -.25, 100);

        turnToWhite(.25, -.25);

        stayWhiteBlueAdvanced();
        //runForTime(-.2, -.17, 4000);

            runForTime(-.25, -.25, 500);

        sleep(500);

        runForTime(.25, .25, 250);

        if (colorSensor.blue() < colorSensor.red() && !(colorSensor.blue() < colorSensor.alpha())) {
            sleep(5000);
            runForTime(-.25, -.25, 1000);
        } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.alpha()){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }


        handFront.setPosition(1);
        runForTime(.4, .4, 2000);
        runForTime(-.8, .8, 2000);


    }
}