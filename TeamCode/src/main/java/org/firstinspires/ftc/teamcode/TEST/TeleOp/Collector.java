package org.firstinspires.ftc.teamcode.TEST.TeleOp;

import android.view.Display;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "Collector", group = "test")

public class Collector extends LinearOpMode {

    //DRIVE
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

    //COLLECTOR
    private DcMotor collectorDC;
    private DcMotor sweeperDC;
    private Servo collectorServo;
    private ModernRoboticsTouchSensor collectorServoLimit;
    private ModernRoboticsTouchSensor collectorExtLimit;

    private double cPos;
    private double cMid;
    private double cOpen;
    private double cClose;

    private double cSpeed;
//    private double cExtPos;

    //ENCODER SERVO
    private Servo xServo;
    private Servo yServo;

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

        //ENCODER SERVOS
        xServo = hardwareMap.servo.get("xServo");
        yServo = hardwareMap.servo.get("yServo");

        //COLLECTOR
        collectorDC = hardwareMap.dcMotor.get("collectDC");
        sweeperDC = hardwareMap.dcMotor.get("sweeperDC");
        collectorServo = hardwareMap.servo.get("cServo");
        collectorServoLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "collectorExt");
        collectorExtLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "collectorServo");

        sweeperDC.setDirection(REVERSE);
        collectorDC.setDirection(REVERSE);

        collectorDC.setMode(STOP_AND_RESET_ENCODER);
        collectorDC.setMode(RUN_USING_ENCODER);

        cSpeed = 0.07;

//        cExtPos = collectorDC.getCurrentPosition();
        cOpen = 0.79;
        cClose = 0;
        cMid = 0.45;

        collectorServo.setPosition(cClose);
        xServo.setPosition(0.12);
        yServo.setPosition(0.38);

        waitForStart();

        while (opModeIsActive()) {

//            gamepad1.setJoystickDeadzone(0.3f); //THIS WAS NEVER ON

            if (gamepad1.left_stick_button) {
                collectorDC.setMode(STOP_AND_RESET_ENCODER);
                collectorDC.setMode(RUN_USING_ENCODER);
            }

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


            /**SWEEPER*/
            if (gamepad2.right_bumper && !gamepad2.left_bumper && gamepad2.left_trigger < 0.1) {
                sweeperDC.setPower(0.7);
            }
            if (gamepad2.left_trigger > 0.1 && !gamepad2.right_bumper && !gamepad1.left_bumper  && !gamepad2.b) {
                sweeperDC.setPower(-gamepad2.left_trigger);
            }
            if (gamepad2.left_bumper && !gamepad2.right_bumper && gamepad2.left_trigger < 0.1  && !gamepad2.b) {
                sweeperDC.setPower(0);
            }

            /**COLLECTOR*/
            if (gamepad2.x && !gamepad2.y) {
                while (collectorServo.getPosition() < cOpen) {
                    cPos = collectorServo.getPosition();
                    cPos += 0.05;
                    collectorServo.setPosition(cPos);
                }
                if (cPos > cOpen) {
                    cPos = cOpen;
                    collectorServo.setPosition(cPos);
                }
            }
//            if (!collectorServoLimit.isPressed()) {
                if (!gamepad2.x && gamepad2.y ) {
                    while (collectorServo.getPosition() > cClose) {
                        cPos = collectorServo.getPosition();
                        cPos -= 0.07;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos < cClose) {
                        cPos = cClose;
                        collectorServo.setPosition(cPos);
                    }
                }

            telemetry.addData("collectorDC: ", collectorDC.getCurrentPosition());
            telemetry.update();

//            }
//            if (gamepad2.a && !gamepad2.b) {
//                collectorDC.setPower(0.8);
////                cExtPos = collectorDC.getCurrentPosition();
//            }
//            if (!gamepad2.a && gamepad2.b) {
//                collectorDC.setPower(-0.8);
////                COLLECTOREXTCLOSE(-0.8);
////                cExtPos = collectorDC.getCurrentPosition();
//            }
//            if (!gamepad2.a && !gamepad2.b) {
//                collectorDC.setPower(0);
//            }

            if (gamepad1.dpad_up && !gamepad1.dpad_down){
                collectorDC.setPower(0.8);
            }
            if (gamepad1.dpad_down && !gamepad1.dpad_up){
                collectorDC.setPower(-0.8);
            }
            if (!gamepad1.dpad_down && !gamepad1.dpad_up){
                collectorDC.setPower(0);
            }

            telemetry.addData("c:", collectorDC.getCurrentPosition());

        }
    }

//    void COLLECTOREXTCLOSE (double power) {
//        sweeperDC.setPower(1);
//        while (collectorServo.getPosition() > cMid) {
//            cPos = collectorServo.getPosition();
//            cPos -= cSpeed;
//            collectorServo.setPosition(cPos);
//        }
//        if (cPos < cMid) {
//            cPos = cMid;
//            collectorServo.setPosition(cPos);
//        }
//        sweeperDC.setPower(0);
//        while (!collectorExtLimit.isPressed()) {
//            collectorDC.setPower(power);
//        }
//        collectorDC.setPower(0);
//    }
}
//arm closed = 0.26