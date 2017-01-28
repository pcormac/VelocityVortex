/*
Copyright (c) 2016 Robert Atkinson

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

import com.qualcomm.ftccommon.FtcWifiChannelSelectorActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTelemetry;

@TeleOp(name="SensorTest", group="Iterative Opmode")
public class SensorTest extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Device declarations
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor elevator = null;
    private DcMotor leftFly = null;
    private DcMotor rightFly = null;
    private Servo handFront = null;
    private Servo handBack = null;

    TouchSensor touchSensor = null;
    TouchSensor elevatorTouch = null;
    OpticalDistanceSensor odsSensor;
    ColorSensor colorSensor = null;
    UltrasonicSensor ultra = null;
    OpticalDistanceSensor sharpIR = null;

    String color;

   static double odsStart;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // get motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        elevator = hardwareMap.dcMotor.get("elevator");
        leftFly = hardwareMap.dcMotor.get("leftFly");
        rightFly = hardwareMap.dcMotor.get("rightFly");

        handFront = hardwareMap.servo.get("handFront");
        handBack = hardwareMap.servo.get("handBack");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftFly.setDirection(DcMotor.Direction.REVERSE);

        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        elevatorTouch = hardwareMap.touchSensor.get("elevatorTouch");
        ultra = hardwareMap.ultrasonicSensor.get("ultra");
        sharpIR = hardwareMap.opticalDistanceSensor.get("infrared");

        odsStart = odsSensor.getLightDetected();

        colorSensor.enableLed(false);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if ((colorSensor.red() > colorSensor.blue()) && (colorSensor.red() > colorSensor.alpha())) {
            color = "Red";
        }
        else if ((colorSensor.blue() > colorSensor.red()) && (colorSensor.blue() > colorSensor.alpha())) {
            color = "Blue";
        }
        else {
            color = "Alpha";
        }

        // end of code, update telemetry
        telemetry.addData("UltraSonic: ", ultra.getUltrasonicLevel());
        telemetry.addData("SharpIR: ", sharpIR.getLightDetected());
        telemetry.addData("Color Red: ", color);
        telemetry.addData("Elevator Touch: ", elevatorTouch.getValue());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
