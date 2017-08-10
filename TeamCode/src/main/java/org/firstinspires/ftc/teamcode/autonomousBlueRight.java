package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;


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

@Autonomous(name="Blue Autonomous", group="2 Blue")  // @Autonomous(...) is the other common choice
@Disabled
public class autonomousBlueRight extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftmotor;
    DcMotor rightmotor;
    DcMotor lights;
    ColorSensor color;
    DcMotor leftCannon;
    DcMotor rightCannon;
    DcMotor shooter;
    DcMotor beacon;
    DcMotor sweeper;


    ModernRoboticsI2cGyro gyro;

    boolean gyroIsActive = true;
    boolean shooterIsActive = true;

    boolean firstPosition;
    boolean secondPosition;

    int currentGyro;

    int currentEnc;


    public void encReset() throws InterruptedException {
        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beacon.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        beacon.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(0010);
    }

    public void checkColor() throws InterruptedException {
        if (color.blue() - 80 > color.red()) {
            telemetry.addData("Blue: ", color.blue());
            telemetry.addData("Red: ", color.red());
            telemetry.update();
            double currentTime = runtime.milliseconds();
            beacon.setPower(0.5);
            while (Math.abs( beacon.getCurrentPosition() ) < 850 && currentTime + 700 > runtime.milliseconds() && opModeIsActive() ) {
            }
            beacon.setPower(-0.25);
            while (beacon.getCurrentPosition() > 1 && opModeIsActive()) {
            }
            beacon.setPower(0);
            //encReset();

            firstPosition = true;
        } else {
            telemetry.addData("Blue: ", color.blue());
            telemetry.addData("Red: ", color.red());
            telemetry.update();
            currentEnc = rightmotor.getCurrentPosition();
            leftmotor.setPower(-0.25);
            rightmotor.setPower(-0.25);

            while (currentEnc - 300 <  rightmotor.getCurrentPosition() && opModeIsActive() ) {
            }
            leftmotor.setPower(0);
            rightmotor.setPower(0);
            beacon.setPower(0.5);
            double currentTime = runtime.milliseconds();
            while (Math.abs( beacon.getCurrentPosition() ) < 850 && currentTime + 700 > runtime.milliseconds() && opModeIsActive()) {

            }
            beacon.setPower(-0.25);
            while (beacon.getCurrentPosition() > 1 && opModeIsActive()) {

            }
            beacon.setPower(0);
            //encReset();
            secondPosition = true;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        sweeper = hardwareMap.dcMotor.get("sweeper");
        leftmotor = hardwareMap.dcMotor.get("leftmotor");
        rightmotor = hardwareMap.dcMotor.get("rightmotor");
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
        beacon = hardwareMap.dcMotor.get("beacon");
        color = hardwareMap.colorSensor.get("color");
        lights = hardwareMap.dcMotor.get("lights");
        shooter = hardwareMap.dcMotor.get("shooter");
        leftCannon = hardwareMap.dcMotor.get("leftCannon");
        rightCannon = hardwareMap.dcMotor.get("rightCannon");
        leftCannon.setDirection(DcMotor.Direction.REVERSE);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beacon.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        beacon.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encReset();
        gyro.calibrate();
        sleep(0300);
        while (gyro.isCalibrating() ) {
            telemetry.addData("Please wait..", 0);
            telemetry.update();
        }
        telemetry.addData("Ready!", 0);
        telemetry.update();
        waitForStart();

        runtime.reset();
        lights.setPower(.3);
        currentEnc = rightmotor.getCurrentPosition();

        leftmotor.setPower(-0.28);
        rightmotor.setPower(-0.3);
        while (currentEnc - 3200 < rightmotor.getCurrentPosition() && opModeIsActive()) {
        //    telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
        //    telemetry.update();
        }

        //telemetry.addData("Got first", 0);
        //telemetry.update();

        leftmotor.setPower(-0.13);
        rightmotor.setPower(-0.15);
        while (currentEnc - 3750 < rightmotor.getCurrentPosition() && opModeIsActive()) {

        }
        //telemetry.addData("Got second", 0);
        //telemetry.update();
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        //encReset();

        currentGyro = gyro.getIntegratedZValue();

        leftmotor.setPower(-0.2);
        rightmotor.setPower(0.2);
        while (currentGyro + 15 > gyro.getIntegratedZValue() && opModeIsActive()) {

        }
        leftmotor.setPower(-0.05);
        rightmotor.setPower(0.05);
        while (currentGyro + 40 > gyro.getIntegratedZValue() && opModeIsActive()) {

        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        //encReset();

        currentEnc = rightmotor.getCurrentPosition();
        leftmotor.setPower(-0.18);
        rightmotor.setPower(-0.20);
        while ( currentEnc - 800 < rightmotor.getCurrentPosition() && opModeIsActive()) {

        }
/**        int leftPosition = leftmotor.getCurrentPosition();
        leftmotor.setPower(-0.15);
        rightmotor.setPower(0);1
        while (leftmotor.getCurrentPosition() > leftPosition - 100) {

        }
*/
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        //encReset();
        lights.setPower(0);
        sleep(1000);
        checkColor();



/**        currentGyro = gyro.getIntegratedZValue();

/**        leftmotor.setPower(-0.12);
        rightmotor.setPower(0.12);
        while (currentGyro + 4 > gyro.getIntegratedZValue()) {

        }
/**
        leftmotor.setPower(-0.08);
        rightmotor.setPower(0.08);
        while (currentGyro + 6 > gyro.getIntegratedZValue()) {

        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);
        //encReset();

        //original distance over speed 2750 at 23
        //then 3000 over 13

        currentEnc = rightmotor.getCurrentPosition();

        leftmotor.setPower(-0.4);
        rightmotor.setPower(-0.4);
        while (currentEnc - 2600 < rightmotor.getCurrentPosition()) {
            //    telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
            //    telemetry.update();
        }
        leftmotor.setPower(-0.13);
        rightmotor.setPower(-0.13);
        while (currentEnc - 3450 < rightmotor.getCurrentPosition()) {

        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        //encReset();
        lights.setPower(0);
        sleep(500);


        checkColor();


*/
        currentGyro = gyro.getIntegratedZValue();

        leftmotor.setPower(-0.20);
        rightmotor.setPower(0.2);
        while (currentGyro + 5 > gyro.getIntegratedZValue() && opModeIsActive()) {
        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        currentEnc = rightmotor.getCurrentPosition();
        leftmotor.setPower(-0.35);
        rightmotor.setPower(-0.35);
        while (currentEnc - 1500 < rightmotor.getCurrentPosition() && opModeIsActive()) {
            //   telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
            //   telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        //encReset();

        leftmotor.setPower(0);
        rightmotor.setPower(0);

        currentGyro = gyro.getIntegratedZValue();
        leftmotor.setPower(-0.23);
        rightmotor.setPower(0.23);
        while (currentGyro + 85 > gyro.getIntegratedZValue() && opModeIsActive()) {

        }

        //encReset();

        leftmotor.setPower(0);
        rightmotor.setPower(0);
        sleep(1000);
        currentEnc = rightmotor.getCurrentPosition();
        leftmotor.setPower(-0.37);
        rightmotor.setPower(-0.4);
        while (currentEnc - 1200 < rightmotor.getCurrentPosition() && opModeIsActive()) {
            //   telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
            //   telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        //encReset();

        leftCannon.setPower(1);
        rightCannon.setPower(1);
        shooterIsActive = true;

        sleep(1250);

        shooter.setPower(0.5);
        while (shooterIsActive && opModeIsActive()) {
            if (Math.abs(shooter.getCurrentPosition()) < 1050) {
                //    telemetry.addData("Encoder Position: ", shooter.getCurrentPosition());
            } else {
                shooter.setPower(0);
//                try {
//                    encReset();
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
                shooterIsActive = false;
            }
        }
        sleep(050);

        leftCannon.setPower(0);
        rightCannon.setPower(0);
        sweeper.setPower(-0.7);

        sleep(2500);

        leftCannon.setPower(1);
        rightCannon.setPower(1);
        shooterIsActive = true;

        sweeper.setPower(0);
        sleep(1000);
        shooter.setPower(0.5);
        int currentShot = shooter.getCurrentPosition();
        while (shooterIsActive && opModeIsActive()) {
            if (currentShot + 1050 > shooter.getCurrentPosition()) {
                //    telemetry.addData("Encoder Position: ", shooter.getCurrentPosition());
            } else {
                shooter.setPower(0);
//                try {
//                    encReset();
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
                shooterIsActive = false;
            }
        }
        sleep(050);

        leftCannon.setPower(0);
        rightCannon.setPower(0);
        shooterIsActive = true;
        currentEnc = rightmotor.getCurrentPosition();
        leftmotor.setPower(-0.2);
        rightmotor.setPower(-0.2);
        while (currentEnc - 2500 < rightmotor.getCurrentPosition() && opModeIsActive()){
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);


}
}

