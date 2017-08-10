package org.firstinspires.ftc.teamcode;

/**
 * Created by 8868 on 4/18/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
//Disabled
public class bigfoot extends LinearOpMode

{
    DcMotor leftfoot;
    DcMotor rightfoot;




    @Override
    public void runOpMode() throws InterruptedException {


        leftfoot = hardwareMap.dcMotor.get("leftfoot");
        rightfoot = hardwareMap.dcMotor.get("rightfoot");

        leftfoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        waitForStart();


        {

        leftfoot.setPower(90);
        rightfoot.setPower(90);


        sleep(2000);


    }
        {
            leftfoot.setPower(90);
            rightfoot.setPower(0);


            sleep(0700);


            leftfoot.setPower(90);
            rightfoot.setPower(90);


            sleep(1600);
        }

















}}

