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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="SensorBot: Drive to white (Red)", group="SensorBot Red")
public class SensorBotDriveToWhite extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Device declarations
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    ColorSensor colorSensor;

    OpticalDistanceSensor odsSensor;

    public void runOpMode() throws InterruptedException {
        // get motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        boolean bLedOn = true;

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        /*
        colorSensor.enableLed(bLedOn);
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();
        */
        waitForStart();

        while (opModeIsActive()) {
            float hsvValues[] = {0, 0, 0};
        /*
        while (hsvValues[0] < 290) {
            drive(20,20);
        }

        if (red + blue + green > 40) {
            drive(25,25);
        }
        else {
            drive(50,0);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        */
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Raw", odsSensor.getRawLightDetected());
            telemetry.addData("Normal", odsSensor.getLightDetected());
        }
    }
public void drive (double leftSpeed, double rightSpeed) throws InterruptedException {
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
        }
        }
