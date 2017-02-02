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

import com.qualcomm.ftccommon.FtcWifiChannelSelectorActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTelemetry;

@TeleOp(name="TeleOp7646", group="Iterative Opmode")
public class TeleOp7646 extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Device declarations
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor elevator = null;
    private DcMotor leftFly = null;
    private DcMotor rightFly = null;
    private Servo handFront = null;
    private Servo handCap = null;

    TouchSensor touchSensor = null;
    TouchSensor elevatorTouch = null;
    OpticalDistanceSensor odsSensor;
    ColorSensor colorSensor = null;
    OpticalDistanceSensor sharpIR = null;

    boolean fly = false;
    boolean hand = true;
    double servoTimedPosition = 0;
    double elevatorPower = 0;
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
        handCap = hardwareMap.servo.get("handCap");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftFly.setDirection(DcMotor.Direction.REVERSE);

        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        elevatorTouch = hardwareMap.touchSensor.get("elevatorTouch");
        sharpIR = hardwareMap.opticalDistanceSensor.get("infrared");

        odsStart = odsSensor.getLightDetected();

        colorSensor.enableLed(false);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        handCap.setPosition(1);
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
        telemetry.addData("Status", "Running: " + runtime.toString());

        /* Controls
        Game pad 1
        Left Stick    = Left drive
        Right Stick   = Right Drive
        A             = Grab Ball
        B             =
        Y             =
        X             = Rotate front hand
        Left Bumper   = .25 speed
        Right Bumper  =

        Game pad 2
        Left Stick    = Elevator
        Right Stick   =
        A             = 50% fly
        B             = 100% fly
        Y             =
        X             = Let handCap go
        Left Bumper   =
        Right Bumper  =
        */


        // Tank drive
        if (gamepad1.left_stick_y < .25 && gamepad1.left_stick_y > 0) {
            leftMotor.setPower(0);
        } else if (gamepad1.left_stick_y > -.25 && gamepad1.left_stick_y < 0) {
            leftMotor.setPower(0);
        } else if (gamepad1.dpad_up) {
            leftMotor.setPower(1);
        }
        else if (gamepad1.dpad_down) {
            leftMotor.setPower(-1);
        }
        else {
            if (gamepad1.left_bumper) {
                leftMotor.setPower(.25*gamepad1.left_stick_y);
            }
            else {
                leftMotor.setPower(gamepad1.left_stick_y);
            }
        }

        if (gamepad1.right_stick_y < .25 && gamepad1.right_stick_y > 0) {
            rightMotor.setPower(0);
        } else if (gamepad1.right_stick_y > -.25&& gamepad1.right_stick_y < 0) {
            rightMotor.setPower(0);
        } else if (gamepad1.dpad_up) {
            rightMotor.setPower(1);
        }
        else if (gamepad1.dpad_down) {
            rightMotor.setPower(-1);
        }
        else {
            if (gamepad1.left_bumper) {
                rightMotor.setPower(.25*gamepad1.right_stick_y);
            }
            else {
                rightMotor.setPower(gamepad1.right_stick_y);
            }
        }

        // switching hand boolean
        if (gamepad1.a) {
            handFront.setPosition(1);
        } else if (gamepad1.b) {
            handFront.setPosition(.2);
        } else {
            if (sharpIR.getRawLightDetected() >  2.4 && handFront.getPosition() == .2) {
                runtime.reset();
                servoTimedPosition = (runtime.time()/100);
                handFront.setPosition(Range.clip(.2, .9, servoTimedPosition));
            } else if (sharpIR.getRawLightDetected() < 2.4 && handFront.getPosition() == .9){
                runtime.reset();
                servoTimedPosition = -1*(runtime.time()/100);
                handFront.setPosition(Range.clip(.2, .9, servoTimedPosition));
            }
        }

        if (handFront.getPosition() == 1) {
            hand = true;
        } else if (handFront.getPosition() == .2) {
            hand = false;
        }
        //
        //
        //        JOY 2
        //
        //
        if (elevatorTouch.isPressed()) {
            elevator.setPower(Range.clip(gamepad2.left_stick_y, 0, 1));
        }
        else {
            // elevator
            elevatorPower = -gamepad2.left_stick_y;
            elevator.setPower(elevatorPower);
        }

        // THROWING BALLS
        if (gamepad2.a) {
            if (gamepad2.a  && gamepad2.b) {
                leftFly.setPower(95);
                rightFly.setPower(.95);
            } else {
                leftFly.setPower(.5);
                rightFly.setPower(.5);
            }
        } else {
            leftFly.setPower(0);
            rightFly.setPower(0);
        }

        if (gamepad2.x) {
            handCap.setPosition(.5);
        } else {
            handCap.setPosition(1);
        }


        // end of code, update telemetry
        telemetry.addData("Right: ", gamepad1.right_stick_y);
        telemetry.addData("Left: ", gamepad1.left_stick_y);
        telemetry.addData("SharpIR: ", sharpIR.getRawLightDetected());
        telemetry.addData("Hand: ", hand);
        telemetry.addData("Elevator: ", elevator.getPower());
        telemetry.addData("Front Hand: ", handFront.getPosition());
        telemetry.addData("Fly Power: ", rightFly.getPower());
        telemetry.addData("Elevator Touch: ", elevatorTouch.getValue());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}
