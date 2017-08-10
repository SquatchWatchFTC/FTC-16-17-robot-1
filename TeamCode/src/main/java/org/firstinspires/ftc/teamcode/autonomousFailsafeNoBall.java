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

@Autonomous(name="Autonomous Failsafe For Both Colors No Ball", group="1 Both")  // @Autonomous(...) is the other common choice
//@Disabled
public class autonomousFailsafeNoBall extends LinearOpMode {


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

    boolean gyroIsActive = true;
    boolean shooterIsActive = true;


    public void encReset() throws InterruptedException {
        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beacon.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        beacon.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        telemetry.addData("Version ", 1);
        telemetry.update();
        encReset();
        waitForStart();

        runtime.reset();
        lights.setPower(.3);

        sleep(10000);

        leftmotor.setPower(-0.24);
        rightmotor.setPower(-0.25);
        while (Math.abs(rightmotor.getCurrentPosition()) < 1550 && opModeIsActive()) {
            telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
            telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        encReset();

        leftCannon.setPower(1.0);
        rightCannon.setPower(1.0);
        sleep(2000);
        shooter.setPower(0.5);
        while (shooterIsActive && opModeIsActive()) {
            if (Math.abs(shooter.getCurrentPosition()) < 1050) {
                telemetry.addData("Encoder Position: ", shooter.getCurrentPosition());
            } else {
                shooter.setPower(0);
                try {
                    encReset();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooterIsActive = false;
            }
        }
        sleep(200);

        leftCannon.setPower(0);
        rightCannon.setPower(0);
        shooterIsActive = true;

        sweeper.setPower(-0.8);
        sleep(3000);
        sweeper.setPower(0);

        sleep(200);
        leftCannon.setPower(0.95);
        rightCannon.setPower(0.95);
        sleep(2000);
        shooter.setPower(0.5);
        while (shooterIsActive && opModeIsActive()) {
            if (Math.abs(shooter.getCurrentPosition()) < 1050) {
                telemetry.addData("Encoder Position: ", shooter.getCurrentPosition());
            } else {
                shooter.setPower(0);
                try {
                    encReset();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooterIsActive = false;
            }
        }
        sleep(0200);

        leftCannon.setPower(0);
        rightCannon.setPower(0);

        sleep(0200);

/**        leftmotor.setPower(-0.3);
        rightmotor.setPower(-0.3);
        while (Math.abs(rightmotor.getCurrentPosition()) < 1600 && opModeIsActive()) {
            telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
            telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
*/        encReset();
        leftmotor.setPower(0.24);
        rightmotor.setPower(0.25);
        while (Math.abs(rightmotor.getCurrentPosition()) < 1120 && opModeIsActive()) {
            telemetry.addData("Encoder Count", rightmotor.getCurrentPosition());
            telemetry.update();
        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
    }
}