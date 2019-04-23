package org.firstinspires.ftc.teamcode.TEST.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Disabled
@TeleOp(name = "Basic DriveTest", group = "test")

public class DRIVETESTBASIC extends LinearOpMode {

    //Gamepad 1
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");

        motorFrontRight.setDirection(REVERSE);
        motorBackRight.setDirection(REVERSE);

//        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(STOP_AND_RESET_ENCODER);
//
//        motorFrontRight.setMode(RUN_USING_ENCODER);
//        motorBackRight.setMode(RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                motorFrontRight.setPower(gamepad1.right_trigger);
                motorFrontLeft.setPower(gamepad1.right_trigger);
                motorBackLeft.setPower(gamepad1.right_trigger);
                motorBackRight.setPower(gamepad1.right_trigger);
            }

            if (!gamepad1.dpad_up && gamepad1.dpad_down) {
                motorFrontRight.setPower(-gamepad1.right_trigger);
                motorFrontLeft.setPower(-gamepad1.right_trigger);
                motorBackLeft.setPower(-gamepad1.right_trigger);
                motorBackRight.setPower(-gamepad1.right_trigger);
            }

            if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }

        }
    }
}
//arm closed = 0.26