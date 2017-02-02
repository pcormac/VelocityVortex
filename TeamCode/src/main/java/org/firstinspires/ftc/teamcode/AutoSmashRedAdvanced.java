package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Auto: Smash Red Advanced", group="Main")
public class AutoSmashRedAdvanced extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    public AHRS navx_device;

    static double odsStart;

    public void runOpMode() throws InterruptedException {

        declareMap();

        leftFly.setDirection(DcMotor.Direction.REVERSE);
        colorSensor.enableLed(false);

        handFront.setPosition(.5);

        telemetry.addData(">", "Robot Ready.");
        telemetry.addData("White line", odsStart * 3);
        telemetry.update();

        waitForStart();

        runtime.reset();

        navx_device.zeroYaw();

        if (!navx_device.isCalibrating()) {
            driveStraightTimed(-.3, 1);
        } else {
            runForTime(-.3, -.3, 500);
        }

        // fire first ball
        telemetry.addData("AutoStatus: ", "Firing first ball");
        telemetry.update();
        autoFire();
        elevatorDown();

        // load second ball
        telemetry.addData("AutoStatus: ", "Loading second ball");
        telemetry.update();
        handFront.setPosition(1);
        sleep(500);

        // fire second ball
        telemetry.addData("AutoStatus: ", "Firing second ball");
        telemetry.update();
        autoFire2();
        elevatorDownDrive(.8, -.2, .2);

        if (!navx_device.isCalibrating()) {
            findWhiteStraight();
        } else {
            findWhite();
        }

        if (!navx_device.isCalibrating()) {
            driveStraightTimed(-.125, .3);
        } else {
            runForTime(-.125, -.125, 300);
        }

        turnToWhite(0, -.125);

        stayWhiteRedAdvanced();
        runForTime(.1, .1, 500);
        sleep(500);

        runtime.reset();
        while (opModeIsActive() && runtime.time() < 2) {
            if (colorSensor.red() < colorSensor.blue() && !(colorSensor.red() < colorSensor.alpha())) {
                runForTime(-.1, -.1, 1000);
            }
            else if (colorSensor.alpha() > colorSensor.red()) {
                runForTime(-.1, -.1, 500);
            }
            else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

        }


    }
}