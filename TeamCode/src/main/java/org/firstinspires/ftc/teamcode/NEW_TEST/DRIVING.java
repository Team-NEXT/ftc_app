package org.firstinspires.ftc.teamcode.NEW_TEST;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

/**
 * Created by shiva on 25/5/18.
 */

@Disabled
@TeleOp(name = "Driving test", group = "prototype")

public class DRIVING extends LinearOpMode {

    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

//    private Servo relicArm;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {

            telemetry.addLine("MODE-1 <NATIONALS ALGORITHM>");

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("r = ", r);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            telemetry.addData("robotAngle = ", robotAngle);
            double rightX = gamepad1.right_stick_x;
            //telemetry.addData("rightX = ", rightX);
            final double v1 = r * Math.cos(robotAngle) + rightX;
            telemetry.addData("front left power = ", v1);
            final double v2 = r * Math.sin(robotAngle) - rightX;
            telemetry.addData("front right power = ", v2);
            final double v3 = r * Math.sin(robotAngle) + rightX;
            telemetry.addData("back left power = ", v3);
            final double v4 = r * Math.cos(robotAngle) - rightX;
            telemetry.addData("back right power = ", v4);

            motorFrontLeft.setPower(v1);
            motorFrontRight.setPower(v2);
            motorBackLeft.setPower(v3);
            motorBackRight.setPower(v4);

        }
    }

}
