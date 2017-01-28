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
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Main: Follow White (Blue)", group="Main")
@Disabled
public class MainFollowWhiteBlue extends AutoFunctions {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Device declarations
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor leftFly = null;
    DcMotor rightFly = null;
    DcMotor elevator = null;

    Servo colorServo = null;

    TouchSensor touchSensor = null;
    OpticalDistanceSensor odsSensor;
    ColorSensor colorSensor = null;

    static double odsStart;

    public void runOpMode() throws InterruptedException {
        // get motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftFly = hardwareMap.dcMotor.get("leftFly");
        rightFly = hardwareMap.dcMotor.get("rightFly");
        elevator = hardwareMap.dcMotor.get("elevator");
        colorServo = hardwareMap.servo.get("colorServo");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // set servo to starting position
        colorServo.setPosition(.5);

        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        odsStart = odsSensor.getLightDetected();

        colorSensor.enableLed(false);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        waitForStart();

        telemetry.addData("Light", odsSensor.getLightDetected());
        telemetry.addData("Touch:", touchSensor.getValue());
        //updateColor();
        telemetry.update();

        moveElevator(1, .5);
        moveElevator(-1, .45);

        sleep(5000);
        //position to throw balls
        runForTime(-.5, -.5, 1);
        // throw balls
        autoFire();
        while (opModeIsActive() & runtime.seconds() < 2.25) {
            elevator.setPower(-.25);
        }
        // moveElevator(1, 1);
        // moveElevator(-1, 1);
        // find white line with findWhite loop
        findWhite();
        // stay on white line until touch sensor is pressed
        stayWhiteBlueOld();
        // back up supposedly straight
        runForTime(.1, .1, 3/2);
        // re find white line
        scanWhite();
        // drive back to beacon
        stayWhiteBlueOld();

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        telemetry.addData("Light", odsSensor.getLightDetected());
        telemetry.addData("Touch:", touchSensor.getValue());
        //updateColor();
        telemetry.update();
        idle();
    }
}