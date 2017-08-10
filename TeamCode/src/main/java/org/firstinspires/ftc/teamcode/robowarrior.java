package org.firstinspires.ftc.teamcode;






/**
 * Created by 8868 on 6/2/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="robowarrior", group="1 both")






public class robowarrior extends LinearOpMode
{



    DcMotor left;
    //DcMotor right;
    //DcMotor leftcrush;
    //DcMotor rightcrush;




    @Override
    public void runOpMode() throws InterruptedException {


        left=hardwareMap.dcMotor.get("left");
        //right=hardwareMap.dcMotor.get("right");
       //leftcrush=hardwareMap.dcMotor.get("leftcrush");
        //rightcrush=hardwareMap.dcMotor.get("rightcrush");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftcrush.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightcrush.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();


        {

            left.setPower(-.90);


            sleep(2000);



        }











    }


}
