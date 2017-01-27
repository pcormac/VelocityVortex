package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

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

/**
 * Created by Cormac on 12/6/2016.
 */
@Autonomous(name="Auto: Just throw long", group="Main")
public class AutoJustThrowLong extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    static double odsStart;

    public void runOpMode() throws InterruptedException {

        declareMap();

        odsStart = odsSensor.getLightDetected();

        colorSensor.enableLed(false);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        handFront.setPosition(.5);
        handBack.setPosition(1);

        waitForStart();

        runtime.reset();

        sleep(10000);
        // fire first ball
        telemetry.addData("AutoStatus: ", "Firing first ball");
        telemetry.update();
        autoFireLong();
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
        elevatorDown();
        leftMotor.setPower(-1);
        rightMotor.setPower(-1);
        sleep(1000);

        telemetry.addData("AutoStatus: ", "Autonomous done");
        //updateColor() ;
        telemetry.update();
        idle();

    }

    // new code


}