package org.firstinspires.ftc.teamcode.TEST.TeleOp;

import android.view.Display;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "Dropper", group = "test")

public class dropper extends LinearOpMode {

    //DRIVE
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

    //DROPPING
    private DcMotor dropperDC;
    private Servo dropperServo;
    private ModernRoboticsTouchSensor dropperExtLimit;

    private double dLoad;
    private double dUnload;
    private double dPos;

    //DRIVE ENCODERS
    private Servo xServo;
    private Servo yServo;

    private double xUp;
    private double yUp;

    @Override
    public void runOpMode() throws InterruptedException {

        //DRIVING
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");

        motorFrontRight.setDirection(REVERSE);
        motorBackRight.setDirection(REVERSE);

        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(STOP_AND_RESET_ENCODER);

        motorFrontRight.setMode(RUN_USING_ENCODER);
        motorFrontLeft.setMode(RUN_USING_ENCODER);
        motorBackRight.setMode(RUN_USING_ENCODER);
        motorBackLeft.setMode(RUN_USING_ENCODER);

        //DROPPING
        dropperDC = hardwareMap.dcMotor.get("dropDC");
        dropperServo = hardwareMap.servo.get("dServo");
        dropperExtLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "dropper");

        dLoad = 0.66;
        dUnload = 0.29;

        //DRIVE ENCODERS
        xServo = hardwareMap.servo.get("xServo");
        yServo = hardwareMap.servo.get("yServo");

        xUp = 0.12;
        yUp = 0.38;

        //INITIALIZATION
        dropperServo.setPosition(dLoad);
        xServo.setPosition(xUp);
        yServo.setPosition(yUp);

        waitForStart();

        while (opModeIsActive()) {

//            gamepad1.setJoystickDeadzone(0.3f); //THIS WAS NEVER ON

            /**DRIVING*/
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

            motorFrontLeft.setPower(v1);
            motorFrontRight.setPower(v2);
            motorBackLeft.setPower(v3);
            motorBackRight.setPower(v4);

            /**DROPPER*/
            if (!gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.a && gamepad1.b) {
                while (dropperServo.getPosition() > dUnload) {
                    dPos = dropperServo.getPosition();
                    dPos -= 0.09;
                    dropperServo.setPosition(dPos);
                }
                if (dPos < dUnload) {
                    dPos = dUnload;
                    dropperServo.setPosition(dPos);
                }
            }
            if (!gamepad1.b && gamepad1.a) {
                while (dropperServo.getPosition() < dLoad) {
                    dPos = dropperServo.getPosition();
                    dPos += 0.09;
                    dropperServo.setPosition(dPos);
                }
                if (dPos > dLoad) {
                    dPos = dLoad;
                    dropperServo.setPosition(dPos);
                }
            }
            if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                dropperDC.setPower(1);
            }
            if (!gamepad1.right_bumper && gamepad1.left_bumper && !dropperExtLimit.isPressed()) {
                dropperDC.setPower(-0.7);
            }
            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                dropperDC.setPower(0);
            }

        }
    }
}