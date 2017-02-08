package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto: Smash Red", group="Main")
public class AutoSmashRed extends AutoFunctions {
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


    }
}