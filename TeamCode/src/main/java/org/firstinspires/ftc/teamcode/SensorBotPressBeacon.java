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
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="SensorBot: Press Beacon (Blue)", group="SensorBot")
public class SensorBotPressBeacon extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Device declarations
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    Servo servoColor = null;

    TouchSensor touchSensor = null;
    OpticalDistanceSensor odsSensor;
    ColorSensor colorSensor = null;

    static double odsStart;

    public void runOpMode() throws InterruptedException {
        // get motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        servoColor = hardwareMap.servo.get("servoColor");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // set servo to starting position
        servoColor.setPosition(.5);

        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        odsStart = odsSensor.getLightDetected();

        colorSensor.enableLed(false);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        waitForStart();

        checkColor();

        runForTime(0, 0, 10);
        telemetry.addData("Light", odsSensor.getLightDetected());
        telemetry.addData("Touch:", touchSensor.getValue());
        updateColor();
        telemetry.update();
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

    public void findWhite() throws InterruptedException {
        while (opModeIsActive() && !touchSensor.isPressed() && odsSensor.getLightDetected() < (odsStart * 1.5)) {
            leftMotor.setPower(.2);
            rightMotor.setPower(.2);
            telemetry.addData("Follow Status", "Finding");
            telemetry.addData("Light", odsSensor.getLightDetected());
            telemetry.addData("Touch:", touchSensor.getValue());
            //updateColor();
            telemetry.update();
            idle();
        }

    }

    public void stayWhite() throws InterruptedException {
        while (opModeIsActive() && !touchSensor.isPressed()) {
            // follow white line
            if (odsSensor.getLightDetected() > (1.5 * odsStart)) {
                leftMotor.setPower(.15);
                rightMotor.setPower(.15);
                telemetry.addData("Follow Status", "Driving");
                telemetry.addData("Light", odsSensor.getLightDetected());
                telemetry.addData("Touch:", touchSensor.getValue());
                //updateColor();
                telemetry.update();
            } else {
                leftMotor.setPower(.1);
                rightMotor.setPower(-.1);
                telemetry.addData("Follow Status", "Turning");
                telemetry.addData("Light", odsSensor.getLightDetected());
                telemetry.addData("Touch:", touchSensor.getValue());
                //updateColor();
                telemetry.update();
            }
            idle();
        }
    }

    public void stayWhiteForTime(double timeToRun) throws InterruptedException {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeToRun)) {
            // follow white line
            if (odsSensor.getLightDetected() > (1.5 * odsStart)) {
                leftMotor.setPower(.1);
                rightMotor.setPower(.1);
                telemetry.addData("Follow Status", "Driving");
                telemetry.addData("Light", odsSensor.getLightDetected());
                telemetry.addData("Touch:", touchSensor.getValue());
                //updateColor();
                telemetry.update();
            } else {
                leftMotor.setPower(.1);
                rightMotor.setPower(-.1);
                telemetry.addData("Follow Status", "Turning");
                telemetry.addData("Light", odsSensor.getLightDetected());
                telemetry.addData("Touch:", touchSensor.getValue());
                //updateColor();
                telemetry.update();
            }
            idle();
        }
    }

    public void runForTime(double leftSpeed, double rightSpeed, double timeToRun) throws InterruptedException {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeToRun)) {
            leftMotor.setPower(leftSpeed);
            rightMotor.setPower(rightSpeed);
            telemetry.addData("Follow Status", "Run For Time", leftSpeed, rightSpeed);
            telemetry.update();
            idle();
        }
    }

    public void scanWhite() throws InterruptedException {
        // move left while scanning
        while (opModeIsActive() && odsSensor.getLightDetected() < (odsStart * 1.5)) {
            // scan while turning left
            runForTime(.1, 0, 1);
            runForTime(0, .1, 1);
            runForTime(.05, .05, .5);
            idle();
        }
    }
    public void checkColor() throws InterruptedException {
        if (colorSensor.blue() > colorSensor.red()) {
            telemetry.addData("Color: ", "Blue");
            servoColor.setPosition(0);
            runForTime(.05, .05, .5);
        }
        else {
            telemetry.addData("Color: ", "Red");
            servoColor.setPosition(1);
            runForTime(.05, .05, .5);
        }
    }
}