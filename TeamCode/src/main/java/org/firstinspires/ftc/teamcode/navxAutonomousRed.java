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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;

import java.text.DecimalFormat;

@Autonomous(name="Navx Red Autonomous", group="3 Red")  // @Autonomous(...) is the other common choice
//@Disabled
public class navxAutonomousRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private final int NAVX_DIM_I2C_PORT = 5;

    DcMotor leftmotor;
    DcMotor rightmotor;
    DcMotor beacon;
    DcMotor lights;
    DcMotor leftCannon;
    DcMotor rightCannon;
    DcMotor shooter;
    DcMotor sweeper;

    OpticalDistanceSensor ods;

    private AHRS gyro;
    ColorSensor color;

    Servo leftServo;
    Servo   rightServo;
    Servo stopper;

    boolean gyroIsActive = true;
    boolean shooterIsActive = true;
    boolean firstPosition = false;
    boolean secondPosition = false;

    int currentGyro;
    int currentEncoder;
    int gyroError;
    int targetGyro;
    int encError;

    double lightPower;

    public void checkColor() throws InterruptedException {
        if (color.red() + 50 > color.blue()) {
            double currentTime = runtime.milliseconds();
            beacon.setPower(0.5);
            while (Math.abs( beacon.getCurrentPosition() ) < 800 && currentTime + 1000 > runtime.milliseconds() ) {

            }
            beacon.setPower(-0.25);
            while (beacon.getCurrentPosition() > 1) {

            }
            beacon.setPower(0);
            firstPosition = true;
        } else if ( color.blue() - 50 > color.red() ) {
            currentEncoder = rightmotor.getCurrentPosition();
            leftmotor.setPower(-0.25);
            rightmotor.setPower(-0.25);
            while (currentEncoder - 560 < rightmotor.getCurrentPosition()) {
//changed 500 to 560
            }
            leftmotor.setPower(0);
            rightmotor.setPower(0);
            beacon.setPower(0.5);
            double currentTime = runtime.milliseconds();
            while (Math.abs( beacon.getCurrentPosition() ) < 800 && currentTime + 1000 > runtime.milliseconds()) {

            }
            beacon.setPower(-0.25);
            while (beacon.getCurrentPosition() > 1) {

            }
            beacon.setPower(0);
            secondPosition = true;
        } else {
            checkColor();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftmotor = hardwareMap.dcMotor.get("leftmotor");
        rightmotor = hardwareMap.dcMotor.get("rightmotor");
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
        beacon = hardwareMap.dcMotor.get("beacon");
        beacon.setDirection(DcMotor.Direction.REVERSE);
        sweeper = hardwareMap.dcMotor.get("sweeper");
        shooter = hardwareMap.dcMotor.get("shooter");
        leftCannon = hardwareMap.dcMotor.get("leftCannon");
        rightCannon = hardwareMap.dcMotor.get("rightCannon");
        leftCannon.setDirection(DcMotor.Direction.REVERSE);
        gyro =  AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
        color = hardwareMap.colorSensor.get("color");
        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");
        stopper = hardwareMap.servo.get("stopper");
        lights = hardwareMap.dcMotor.get("lights");
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        leftServo.setPosition(0.0);
        rightServo.setPosition(1.0);

        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beacon.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        beacon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopper.setPosition(0.8);
        rightServo.setPosition(1.0);
        leftServo.setPosition(0);

        telemetry.addData("Version ", 1);
        telemetry.update();

        waitForStart();

        runtime.reset();
        gyro.zeroYaw();
        lights.setPower(.3);

        currentEncoder = rightmotor.getCurrentPosition();
        leftmotor.setPower(-0.45);
        rightmotor.setPower(-0.45);
        while (currentEncoder - 1300 < rightmotor.getCurrentPosition() && opModeIsActive()) {

        }
        currentEncoder = rightmotor.getCurrentPosition();
        leftmotor.setPower(-0.25);
        rightmotor.setPower(-0.25);
        while (currentEncoder - 1600 < rightmotor.getCurrentPosition() && opModeIsActive()) {

        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        leftCannon.setPower(0.9);
        rightCannon.setPower(0.9);
        sleep(1100);
        currentEncoder = shooter.getCurrentPosition();
        shooter.setPower(0.5);
        while (shooterIsActive) {
            if ( Math.abs( currentEncoder + 1050 ) > shooter.getCurrentPosition() ) {

            } else {
                shooter.setPower(0);
                shooterIsActive = false;
            }
        }
        sleep(100);

        leftCannon.setPower(0);
        rightCannon.setPower(0);
        shooterIsActive = true;

        sweeper.setPower(-0.8);
        sleep(1200);
        sweeper.setPower(0);

        leftCannon.setPower(0.9);
        rightCannon.setPower(0.9);
        sleep(1100);
        currentEncoder = shooter.getCurrentPosition();
        shooter.setPower(0.5);
        while (shooterIsActive) {
            if (Math.abs(currentEncoder + 1050 ) > shooter.getCurrentPosition()) {

            } else {
                shooter.setPower(0);
                shooterIsActive = false;
            }
        }
        sleep(0300);

        leftCannon.setPower(0);
        rightCannon.setPower(0);

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        leftmotor.setPower(-0.3);
        rightmotor.setPower(0.3);
        currentGyro = (int) gyro.getYaw();
        while (currentGyro - 50 <= (int) gyro.getYaw() && opModeIsActive()) {

        }
        leftmotor.setPower(-0.1);
        rightmotor.setPower(0.1);
        while (currentGyro - 78 <= (int) gyro.getYaw() && opModeIsActive()) {

        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        sleep(0100);

        currentGyro = (int) gyro.getYaw();
        gyroError = Math.abs((int) gyro.getYaw() + 77);
        leftmotor.setPower(0.1);
        rightmotor.setPower(-0.1);
        telemetry.addData("gyro error" + gyroError + " | current gyro", currentGyro);
        telemetry.update();

        while (currentGyro + gyroError >= (int) gyro.getYaw() && opModeIsActive()) {

        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        sleep(0100);

        currentEncoder = rightmotor.getCurrentPosition();
        leftmotor.setPower(-0.5);
        rightmotor.setPower(-0.5);
        while (currentEncoder - 5100 < rightmotor.getCurrentPosition() && opModeIsActive()) {

        }
        leftmotor.setPower(-0.35);
        rightmotor.setPower(-0.35);
        lightPower = lights.getPower();
        while (currentEncoder - 5900 < rightmotor.getCurrentPosition() && opModeIsActive()) {
            if (lightPower > 0) {
                lightPower = lightPower - 0.01;
                lights.setPower(lightPower);
            } else {
                lights.setPower(0);
            }
        }
        lights.setPower(0);

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        sleep(0200);

        currentGyro = (int) gyro.getYaw();
        targetGyro = -179;

        leftmotor.setPower(-0.3);
        rightmotor.setPower(0.3);

        while (-150 <= gyro.getYaw()) {

        }
        leftmotor.setPower(-0.1);
        rightmotor.setPower(0.1);
        while (targetGyro <= gyro.getYaw() && gyro.getYaw() <= 170) {

        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        sleep(0100);

        leftmotor.setPower(0.1);
        rightmotor.setPower(-0.1);

        while (gyro.getYaw() <= 177 && gyro.getYaw() > 0 ) {

        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        sleep(0100);

        currentEncoder = rightmotor.getCurrentPosition();
        leftmotor.setPower(0.35);
        rightmotor.setPower(0.35);
        while (currentEncoder + 250 > rightmotor.getCurrentPosition() && opModeIsActive()) {
            if (color.red() > 2000 | color.red() <= 0 | color.blue() < 0) {
                telemetry.addData("ABORT ",0);
                telemetry.addData("Color sensor giving strange values ", 1);
                telemetry.update();
                leftmotor.setPower(0);
                rightmotor.setPower(0);
                sleep(20000);
            }
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        sleep(1000);
        checkColor();

        currentEncoder = rightmotor.getCurrentPosition();
        if (firstPosition) {
            leftmotor.setPower(0.5);
            rightmotor.setPower(0.5);
            while (currentEncoder + 5500 > rightmotor.getCurrentPosition()) {
                if (color.red() > 2000 | color.red() <= 0 | color.blue() < 0) {
                    telemetry.addData("ABORT ",0);
                    telemetry.addData("Color sensor giving strange values ", 1);
                    telemetry.update();
                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(20000);
                }
            }
        } else {
            leftmotor.setPower(0.5);
            rightmotor.setPower(0.5);
            while (currentEncoder + 6150 > rightmotor.getCurrentPosition()) {
                if (color.red() > 2000 | color.red() <= 0 | color.blue() < 0) {
                    telemetry.addData("ABORT ",0);
                    telemetry.addData("Color sensor giving strange values ", 1);
                    telemetry.update();
                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(20000);
                }
            }
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        sleep(2000);
        checkColor();

    }
}