package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name= "TeleOp1", group= "Test")

public class TeleOp1 extends LinearOpMode
//declaring motor variables//
{
    private DcMotor motorfrontleft;
    private DcMotor motorbackleft;
    private DcMotor motorfrontright;
    private DcMotor motorbackright;


    @Override
    public void runOpMode () throws InterruptedException
    //hardware mapping//
    {
        motorfrontleft = hardwareMap.dcMotor.get("frontLeft");
        motorbackleft = hardwareMap.dcMotor.get("backLeft");  //"MC1M2" is the name of the motors declared in the phone//
        motorfrontright = hardwareMap.dcMotor.get("frontRight");  //"motorbackleft" is the name of the motors in the program//
        motorbackright = hardwareMap.dcMotor.get("backRight");

        motorfrontleft.setDirection(DcMotor.Direction.REVERSE);
        motorbackleft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())
        {
        //moving the robot forward//
            if(gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right){
                motorfrontleft.setPower(gamepad1.right_trigger);
                motorbackleft.setPower(gamepad1.right_trigger);
                motorfrontright.setPower(gamepad1.right_trigger);
                motorbackright.setPower(gamepad1.right_trigger);
            }
         //moving the robot backwards//

            if(!gamepad1.dpad_up && gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right){
                motorfrontleft.setPower(-gamepad1.right_trigger);
                motorbackleft.setPower(-gamepad1.right_trigger);
                motorfrontright.setPower(-gamepad1.right_trigger);
                motorbackright.setPower(-gamepad1.right_trigger);
            }
         //axis turn left//

            if(!gamepad1.dpad_up && !gamepad1.dpad_down && gamepad1.dpad_left && !gamepad1.dpad_right){
                motorfrontleft.setPower(-gamepad1.right_trigger);
                motorbackleft.setPower(-gamepad1.right_trigger);
                motorfrontright.setPower(gamepad1.right_trigger);
                motorbackright.setPower(gamepad1.right_trigger);
            }
         //axis turn right//

            if(!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && gamepad1.dpad_right){
                motorfrontleft.setPower(gamepad1.right_trigger);
                motorbackleft.setPower(gamepad1.right_trigger);
                motorfrontright.setPower(-gamepad1.right_trigger);
                motorbackright.setPower(-gamepad1.right_trigger);
            }


            idle();
        }

    }
}
