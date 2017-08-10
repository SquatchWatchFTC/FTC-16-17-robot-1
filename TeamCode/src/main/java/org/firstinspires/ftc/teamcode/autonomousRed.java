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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red Autonomous", group="3 Red")  // @Autonomous(...) is the other common choice
@Disabled
public class autonomousRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftmotor;
    DcMotor rightmotor;
    DcMotor beacon;
    DcMotor lights;
    DcMotor leftCannon;
    DcMotor rightCannon;
    DcMotor shooter;
    DcMotor sweeper;

    ModernRoboticsI2cGyro gyro;
    ColorSensor color;

    Servo leftServo;
    Servo   rightServo;

    boolean gyroIsActive = true;
    boolean shooterIsActive = true;
    boolean firstPosition = false;
    boolean secondPosition = false;

    int currentGyro;

    public void encReset(String resetMode) throws InterruptedException {
        if (resetMode.equalsIgnoreCase("straight")) {
            rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            beacon.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            beacon.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            beacon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sleep (0250);
        } else if (resetMode.equalsIgnoreCase("turn")) {
            leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            telemetry.addData("Nothing works! ",resetMode);
            telemetry.update();
        }
    }

    public void checkColor() throws InterruptedException {
        if (color.red() > color.blue()) {
            double currentTime = runtime.milliseconds();
            beacon.setPower(0.5);
            while (Math.abs( beacon.getCurrentPosition() ) < 850 && currentTime + 1500 > runtime.milliseconds() ) {
             //   telemetry.addData("Current Position", beacon.getCurrentPosition() );
             //   telemetry.update();
            }
            beacon.setPower(-0.25);
            while (beacon.getCurrentPosition() > 1) {
             //   telemetry.addData("Current Position", beacon.getCurrentPosition() );
             //   telemetry.update();
            }
            beacon.setPower(0);
            encReset("straight");
            firstPosition = true;
        } else if ( color.blue() > color.red() ) {
            leftmotor.setPower(-0.25);
            rightmotor.setPower(-0.25);
            while ( Math.abs( rightmotor.getCurrentPosition() ) < 250 ) {
            //    telemetry.addData("Encoder Count", Math.abs( rightmotor.getCurrentPosition() ) );
            //    telemetry.update();
            }
            leftmotor.setPower(0);
            rightmotor.setPower(0);
            beacon.setPower(0.5);
            double currentTime = runtime.milliseconds();
            while (Math.abs( beacon.getCurrentPosition() ) < 500 && currentTime + 1500 > runtime.milliseconds()) {
             //   telemetry.addData("Current Position", beacon.getCurrentPosition() );
             //   telemetry.update();
            }
            beacon.setPower(-0.25);
            while (beacon.getCurrentPosition() > 1) {
             //   telemetry.addData("Current Position", beacon.getCurrentPosition() );
             //   telemetry.update();
            }
            beacon.setPower(0);
            encReset("straight");
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
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");
        lights = hardwareMap.dcMotor.get("lights");
        leftServo.setPosition(0.0);
        rightServo.setPosition(1.0);

        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        beacon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encReset("straight");
        //telemetry.addData("Status", "Running: " + runtime.toString());
        gyro.calibrate();
        waitForStart();

        //telemetry.addData("Status", "Running: " + runtime.toString());
        runtime.reset();
        lights.setPower(.3);

        leftmotor.setPower(0.25);
        rightmotor.setPower(0.25);
        while (rightmotor.getCurrentPosition() < 900) {
            //    telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
            //    telemetry.update();
        }
        leftmotor.setPower(0.05);
        rightmotor.setPower(0.05);
        while (rightmotor.getCurrentPosition() < 1120) {
            //    telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
            //    telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        encReset("straight");

        leftmotor.setPower(-0.20);
        rightmotor.setPower(0.20);
        currentGyro = gyro.getIntegratedZValue();
        while (currentGyro + 10 >= gyro.getIntegratedZValue()) {
            //    telemetry.addData("Target: ~45 ... Current Heading: ", gyro.getHeading());
            //    telemetry.update();
        }
        leftmotor.setPower(-0.05);
        rightmotor.setPower(0.03);
        while (currentGyro + 35 >= gyro.getIntegratedZValue()) {
            //    telemetry.addData("Target: ~45 ... Current Heading: ", gyro.getHeading());
            //    telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        encReset("striaght");
        telemetry.addData("Drifted Heading: ", gyro.getIntegratedZValue() - 40);
        telemetry.addData("Original Value: ", currentGyro);
        telemetry.update();

        leftmotor.setPower(0.25);
        rightmotor.setPower(0.25);
        while (rightmotor.getCurrentPosition() < 5750) {
            //    telemetry.addData("Encoder Count: ", rightmotor.getCurrentPosition());
            //    telemetry.addData("Heading: ", gyro.getHeading());
            //    telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        encReset("straight");

        leftmotor.setPower(0.20);
        rightmotor.setPower(-0.20);
        currentGyro = gyro.getIntegratedZValue();
        while (currentGyro - 10 <= gyro.getIntegratedZValue()) {
        //telemetry?
        }
/**        while (gyroIsActive) {
            if (gyro.getHeading() >= 345) {
                gyroIsActive = false;
            } else {
            //    telemetry.addData("Target Heading: ~0 ... Current Heading: ", gyro.getHeading());
            //    telemetry.update();
            }
        }
        gyroIsActive = true;
*/
        leftmotor.setPower(0.03);
        rightmotor.setPower(-0.05);
        while (currentGyro - 30 <= gyro.getIntegratedZValue()) {
            //telemetry?
        }
/**        while (gyroI
/**        while (gyroIsActive) {
            if (gyro.getHeading() >= 348 || gyro.getHeading() <= 352) {
                gyroIsActive = false;
            } else {
            //    telemetry.addData("Target Heading: ~0 ... Current Heading: ", gyro.getHeading());
            //    telemetry.update();
            }
        }
 */
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        encReset("straight");

        leftmotor.setPower(-0.25);
        rightmotor.setPower(-0.25);
        while (Math.abs(rightmotor.getCurrentPosition()) < 1250) {
        //    telemetry.addData("Encoder Count: ", rightmotor.getCurrentPosition());
        //    telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        lights.setPower(0);
        encReset("straight");

        sleep(500);

        checkColor();

        if (firstPosition) {
            leftmotor.setPower(0.25);
            rightmotor.setPower(0.25);
            while (rightmotor.getCurrentPosition() < 1100) {
            //    telemetry.addData("Encoder Count: ", rightmotor.getCurrentPosition());
             //   telemetry.addData("Gyro Heading: ", gyro.getHeading());
             //   telemetry.update();
            }
            leftmotor.setPower(0);
            rightmotor.setPower(0);
        } else {
            leftmotor.setPower(0.25);
            rightmotor.setPower(0.25);
            while (rightmotor.getCurrentPosition() < 1425) {
             //   telemetry.addData("Encoder Count: ", rightmotor.getCurrentPosition());
                //telemetry.addData("Gyro Heading: ", gyro.getHeading());
              //  telemetry.update();
            }
            leftmotor.setPower(0);
            rightmotor.setPower(0);
        }

        encReset("straight");
        leftmotor.setPower(-0.30);
        rightmotor.setPower(0.27);
        currentGyro = gyro.getIntegratedZValue();
        while (currentGyro + 20 >= gyro.getIntegratedZValue()) {
            //telemetry?
        }
//        while (gyro.getHeading() <= 300 || gyro.getHeading() >= 316) {
       //     telemetry.addData("Target: ~340 ... Current Heading: ", gyro.getHeading());
       //     telemetry.update();
//        }
        leftmotor.setPower(-0.05);
        rightmotor.setPower(0.03);
        while (currentGyro + 65 >= gyro.getIntegratedZValue()) {
            //telemetry?
        }
//        while (gyro.getHeading() <= 302 || gyro.getHeading() >= 309) {
        //    telemetry.addData("Target: ~320 ... Current Heading: ", gyro.getHeading());
        //   telemetry.update();
//        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        encReset("straight");

        leftmotor.setPower(-0.24);
        rightmotor.setPower(-0.25);
        while (Math.abs(rightmotor.getCurrentPosition()) < 1220) {
         //   telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
         //   telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        encReset("straight");

        leftCannon.setPower(1);
        rightCannon.setPower(1);
        sleep(1500);
        shooter.setPower(0.5);
        while (shooterIsActive) {
            if (Math.abs(shooter.getCurrentPosition()) < 1050) {
            //    telemetry.addData("Encoder Position: ", shooter.getCurrentPosition());
            } else {
                shooter.setPower(0);
                try {
                    encReset("straight");
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooterIsActive = false;
            }
        }
        sleep(100);

        leftCannon.setPower(0);
        rightCannon.setPower(0);
        shooterIsActive = true;

        sweeper.setPower(-0.8);
        sleep(1500);
        sweeper.setPower(0);

        leftCannon.setPower(1);
        rightCannon.setPower(1);
        sleep(1500);
        shooter.setPower(0.5);
        while (shooterIsActive) {
            if (Math.abs(shooter.getCurrentPosition()) < 1050) {
            //    telemetry.addData("Encoder Position: ", shooter.getCurrentPosition());
            } else {
                shooter.setPower(0);
                try {
                    encReset("straight");
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooterIsActive = false;
            }
        }
        sleep(0200);

        leftCannon.setPower(0);
        rightCannon.setPower(0);

        leftmotor.setPower(-0.24);
        rightmotor.setPower(-0.25);
        while (Math.abs(rightmotor.getCurrentPosition()) < 1120) {
         //   telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
         //   telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        encReset("straight");
    }
}