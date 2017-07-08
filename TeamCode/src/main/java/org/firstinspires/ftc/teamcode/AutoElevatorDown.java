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
@Autonomous(name="Auto: Elevator Down", group="Main")
public class AutoElevatorDown extends AutoFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void declareDevices() throws InterruptedException {
        super.declareDevices();
    }

    static double odsStart;

    public void runOpMode() throws InterruptedException {

        declareMap();

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Begining pos: ", elevator.getCurrentPosition());
        telemetry.update();

        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Targer pos: ", elevator.getTargetPosition());
        telemetry.update();

        telemetry.addData("" +
                ">", "Robot Ready.");    //
        telemetry.update();

        waitForStart();
        elevator.setTargetPosition(2000);
        elevator.setPower(.75);
        while (elevator.getCurrentPosition() < 2000) {
            telemetry.addData("Encoder Ticks: ", elevator.getCurrentPosition());
            telemetry.update();
            idle();
        }
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorDown();

        telemetry.addData("AutoStatus: ", "Elevator down");
        telemetry.update();
    }

    // new code


}
