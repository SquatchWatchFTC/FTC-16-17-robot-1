package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 8868 on 6/2/2017.
 */

@TeleOp(name="teleoprobowarrior", group="teleop")
public class teloprobowarrior extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    int timesPassed;

    boolean isActive = false;
    boolean beaconIsActive = false;
    DcMotor left;
    DcMotor right;
    DcMotor leftcrusher;
    DcMotor rightcrusher;







    public void encReset() throws InterruptedException {

    }
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        leftcrusher = hardwareMap.dcMotor.get("leftcrusher");
        rightcrusher = hardwareMap.dcMotor.get("rightcrusher");


    }
    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
        runtime.startTime();
    }


    @Override
    public void loop() {




        if (gamepad1.x) {left.setPower(.90);}
        else {left.setPower(0);}
        if (gamepad1.b) {right.setPower(.90);}
        else {right.setPower(0);}


  /*      if (gamepad1.y) {
            left.setPower(-1.0);
            right.setPower(-1.0);
        } else if (gamepad1.a) {
            leftcrusher.setPower(1);
            rightcrusher.setPower(1);

        } else {
            leftcrusher.setPower(0.0);
            rightcrusher.setPower(0.0);
        }

*/






        if (gamepad2.b) {
            leftcrusher.setPower(-1.0);
            rightcrusher.setPower(-1.0);
        } else if (gamepad2.x) {
            leftcrusher.setPower(1);
            rightcrusher.setPower(1);
        } else {
            leftcrusher.setPower(0.0);
            rightcrusher.setPower(0.0);
        }




/*

        if (gamepad1.left_bumper) {
            leftY = leftY * -1;
            rightY = rightY * -1;




        if (gamepad1.x) {
            left.setPower(.75);
        }

        else { left.setPower(0);

            if (gamepad1.b) {
                right.setPower(.75);
            }
            else {right.setPower(0);}

            if (gamepad1.y) {
                right.setPower(.75);
                left.setPower(.75);
            }
            else {right.setPower(0);
                left.setPower(0);

                if (gamepad1.a) {
                    right.setPower(-.75);
                    left.setPower(-.75);
                }
                else{right.setPower(0);
                left.setPower(0);

                    if (rightT > 0) {
                        leftcrusher.setPower(-.75);
                        rightcrusher.setPower(-.75);
                    }
                    else {
                        rightcrusher.setPower(0);
                        leftcrusher.setPower(0);
                    }
                    if (leftT > 0) {
                        leftcrusher.setPower(.75);
                        rightcrusher.setPower(.75);
                    }
                    else {
                        rightcrusher.setPower(0);
                        leftcrusher.setPower(0);
                    }

                    }
                    }

                }
*/







        }}
