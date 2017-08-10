package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.kauailabs.navx.ftc.AHRS;


@Autonomous(name="Navx Blue Autonomous", group="2 Blue")
//@Disabled
public class navxAutonomousBlue extends LinearOpMode{

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

    private AHRS gyro;
    ColorSensor color;

    Servo leftServo;
    Servo rightServo;
    Servo stopper;

    boolean shooterIsActive = true;
    boolean firstPosition = false;
    boolean secondPosition = false;

    int currentGyro;
    int currentEncoder;
    int gyroError;
    int targetGyro;

    double lightPower;

    public void checkColor() throws InterruptedException {
        if (color.blue() - 50 > color.red()) {
            double currentTime = runtime.milliseconds();
            beacon.setPower(0.5);
            while (Math.abs( beacon.getCurrentPosition() ) < 800 && currentTime + 1500 > runtime.milliseconds() ) {

            }
            beacon.setPower(-0.25);
            while (beacon.getCurrentPosition() > 1) {

            }
            beacon.setPower(0);
            firstPosition = true;
        } else if ( color.red() + 50 > color.blue() ) {
            currentEncoder = rightmotor.getCurrentPosition();
            leftmotor.setPower(-0.25);
            rightmotor.setPower(-0.25);
            while (currentEncoder - 560 < rightmotor.getCurrentPosition()) {

            }
            leftmotor.setPower(0);
            rightmotor.setPower(0);
            beacon.setPower(0.5);
            double currentTime = runtime.milliseconds();
            while (Math.abs( beacon.getCurrentPosition() ) < 800 && currentTime + 1500 > runtime.milliseconds()) {

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

        leftCannon.setPower(0.90);
        rightCannon.setPower(0.90);
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
        sleep(1300);
        sweeper.setPower(0);

        leftCannon.setPower(0.90);
        rightCannon.setPower(0.90);
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

        leftmotor.setPower(0.3);
        rightmotor.setPower(-0.3);
        currentGyro = (int) gyro.getYaw();
        targetGyro = currentGyro + 75;
        while (targetGyro >= (int) gyro.getYaw() && opModeIsActive()) {

        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        sleep(0100);

        currentGyro = (int) gyro.getYaw();
        gyroError = Math.abs((int) gyro.getYaw() - targetGyro);
        leftmotor.setPower(-0.15);
        rightmotor.setPower(0.15);
        telemetry.addData("gyro error" + gyroError + "current gyro", currentGyro);
        telemetry.update();

        while (currentGyro - gyroError <= (int) gyro.getYaw() && opModeIsActive()) {

        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        sleep(0100);

        currentEncoder = rightmotor.getCurrentPosition();
        leftmotor.setPower(-0.5);
        rightmotor.setPower(-0.5);
        while (currentEncoder - 4900 < rightmotor.getCurrentPosition() && opModeIsActive()) {

        }
        leftmotor.setPower(-0.35);
        rightmotor.setPower(-0.35);
        lightPower = lights.getPower();
        while (currentEncoder - 5600 < rightmotor.getCurrentPosition() && opModeIsActive()) {
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
        targetGyro = currentGyro - 76;

        leftmotor.setPower(-0.3);
        rightmotor.setPower(0.3);

        while (targetGyro <= gyro.getYaw()) {

        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        sleep(0100);

        leftmotor.setPower(0.15);
        rightmotor.setPower(-0.15);
        while (gyro.getYaw() <= -5) {

        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        gyro.close();

        currentEncoder = rightmotor.getCurrentPosition();
        leftmotor.setPower(-0.35);
        rightmotor.setPower(-0.35);
        while (currentEncoder - 850 < rightmotor.getCurrentPosition() && opModeIsActive()) {
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
            leftmotor.setPower(-0.5);
            rightmotor.setPower(-0.5);
            while (currentEncoder - 6000 < rightmotor.getCurrentPosition()) {
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
            leftmotor.setPower(-0.5);
            rightmotor.setPower(-0.5);
            while (currentEncoder - 5300 < rightmotor.getCurrentPosition()) {
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
