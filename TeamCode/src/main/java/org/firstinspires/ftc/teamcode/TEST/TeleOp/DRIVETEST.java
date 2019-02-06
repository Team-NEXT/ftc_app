package org.firstinspires.ftc.teamcode.TEST.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "DriveTest", group = "test")

public class DRIVETEST extends LinearOpMode {

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

//            gamepad1.setJoystickDeadzone(0.3f);

            //GamePad 1
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("r = ", r);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            telemetry.addData("robotAngle = ", robotAngle);
            double rightX = gamepad1.right_stick_x;
            telemetry.addData("rightX = ", rightX);
            final double v1 = r * Math.cos(robotAngle) + rightX;
            telemetry.addData("front left power = ", v1);
            final double v2 = r * Math.sin(robotAngle) - rightX;
            telemetry.addData("front right power = ", v2);
            final double v3 = r * Math.sin(robotAngle) + rightX;
            telemetry.addData("back left power = ", v3);
            final double v4 = r * Math.cos(robotAngle) - rightX;
            telemetry.addData("back right power = ", v4);

            motorFrontRight.setPower(v2);
            motorFrontLeft.setPower(v1);
            motorBackLeft.setPower(v3);
            motorBackRight.setPower(v4);

//            //Degrees travlled at this point
//            telemetry.addData("y = ", motorFrontRight.getCurrentPosition());
//            telemetry.addData("x = ", motorBackRight.getCurrentPosition());
//            telemetry.update();
//
//            if (gamepad1.a){
//
//                motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
//                motorBackRight.setMode(STOP_AND_RESET_ENCODER);
//
//                motorFrontRight.setMode(RUN_USING_ENCODER);
//                motorBackRight.setMode(RUN_USING_ENCODER);

//            }

        }
    }
}
//arm closed = 0.26