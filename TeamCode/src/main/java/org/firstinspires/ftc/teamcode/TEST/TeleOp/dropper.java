package org.firstinspires.ftc.teamcode.TEST.TeleOp;

import android.view.Display;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.back.SsaToRop;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "Dropper", group = "mech-test")

public class dropper extends LinearOpMode {

    //DRIVE
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

    //COLLECTOR
    private DcMotor collectorDC;
    private CRServo sweeperServo;
    private Servo collectorServo;
    private Servo flapServo;
    private ModernRoboticsTouchSensor collectorLimit;

    private static double FO = 0.55, FC = 0.18;

    private double cPos;
    private double cMid;
    private double cDrop;
    private double cOpen;
    private double cClose;

    //DROPPING
    private DcMotor dropperDC;
    private Servo dropperServo;
    private ModernRoboticsTouchSensor dropperLimit;

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

        motorFrontLeft.setDirection(REVERSE);
        motorBackLeft.setDirection(REVERSE);

//        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
//        motorFrontLeft.setMode(STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(STOP_AND_RESET_ENCODER);
//
//        motorFrontRight.setMode(RUN_USING_ENCODER);
//        motorFrontLeft.setMode(RUN_USING_ENCODER);
//        motorBackRight.setMode(RUN_USING_ENCODER);
//        motorBackLeft.setMode(RUN_USING_ENCODER);

        //COLLECTOR
        collectorDC = hardwareMap.dcMotor.get("collectDC");
        sweeperServo = hardwareMap.crservo.get("sweepServo");
//        sweeperServo.setDirection(REVERSE);
        flapServo = hardwareMap.servo.get("flapServo");
        collectorServo = hardwareMap.servo.get("cServo");
        collectorLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "C");

        //sweeperDC.setDirection(REVERSE);
        //collectorDC.setDirection(REVERSE);

        collectorDC.setMode(STOP_AND_RESET_ENCODER);
        collectorDC.setMode(RUN_USING_ENCODER);

//        cSpeed = 0.07;

//        cExtPos = collectorDC.getCurrentPosition();
        cOpen = 0.9;
        cClose = 0.05;
        cMid = 0.61;
        cDrop = 0.25;

        //DROPPING
        dropperDC = hardwareMap.dcMotor.get("dropDC");
        dropperServo = hardwareMap.servo.get("dServo");
        dropperLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "D");

        dLoad = 0.71;
        dUnload = 0.27;

        dropperDC.setDirection(REVERSE);

        dropperDC.setMode(STOP_AND_RESET_ENCODER);
        dropperDC.setMode(RUN_USING_ENCODER);

        //DRIVE ENCODERS
        xServo = hardwareMap.servo.get("xServo");
        yServo = hardwareMap.servo.get("yServo");

        xUp = 0.4;
        yUp = 0.59;

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
            if (dropperLimit.isPressed()) {
                dropperDC.setMode(STOP_AND_RESET_ENCODER);
                dropperDC.setMode(RUN_USING_ENCODER);
            }

            if (!gamepad2.a && !gamepad2.b && !gamepad2.x && gamepad2.y) {
//                startTime = System.currentTimeMillis();
                while (dropperServo.getPosition() > dUnload) {
                    dPos = dropperServo.getPosition();
                    dPos -= 0.023;
                    dropperServo.setPosition(dPos);
                }
                if (dPos < dUnload) {
                    dPos = dUnload;
                    dropperServo.setPosition(dPos);
                }
            }
            if (!gamepad2.y && gamepad2.x) {
                // startTime = System.currentTimeMillis();
                while (dropperServo.getPosition() < dLoad) {
                    dPos = dropperServo.getPosition();
                    dPos += 0.042;
                    dropperServo.setPosition(dPos);
                }
                if (dPos > dLoad) {
                    dPos = dLoad;
                    dropperServo.setPosition(dPos);
                }
            }

            if (gamepad2.left_trigger > 0.1 && gamepad2.right_trigger < 0.1) {
                while (dropperServo.getPosition() > dLoad) {
                    dPos = dropperServo.getPosition();
                    dPos += 0.04;
                    dropperServo.setPosition(dPos);
                }
                if (dPos < dLoad) {
                    dPos = dLoad;
                    dropperServo.setPosition(dPos);
                }
//        startTime = System.currentTimeMillis();
                while (dropperDC.getCurrentPosition() > 400) {
                    dropperDC.setPower(-1);
                }
                dropperDC.setPower(0);
                while (!dropperLimit.isPressed()) {
                    dropperDC.setPower(-0.3);
                }
                dropperDC.setMode(STOP_AND_RESET_ENCODER);
                dropperDC.setMode(RUN_USING_ENCODER);

                dropperDC.setPower(0);
            }

            if (gamepad2.left_trigger < 0.1 && gamepad2.right_trigger > 0.1 && dropperDC.getCurrentPosition() < 100) {
                dropperDC.setMode(STOP_AND_RESET_ENCODER);
                dropperDC.setMode(RUN_USING_ENCODER);
                while (collectorServo.getPosition() > cDrop) {
                    cPos = collectorServo.getPosition();
                    cPos -= 0.07;
                    collectorServo.setPosition(cPos);
                }
                if (cPos < cDrop) {
                    collectorServo.setPosition(cDrop);
                }
                while (dropperDC.getCurrentPosition() < 900) {
                    dropperDC.setPower(1);
                }
                dropperDC.setPower(0);
                dropperDC.setPower(0);
                dropperDC.setPower(0);
                while (dropperServo.getPosition() > dUnload) {
                    dPos = dropperServo.getPosition();
                    dPos -= 0.023;
                    dropperServo.setPosition(dPos);
                }
                if (dPos < dUnload) {
                    dropperServo.setPosition(dUnload);
                }
            }

            if (dropperDC.getCurrentPosition() < 900) {
                if (!gamepad2.a && gamepad2.b) {
                    dropperDC.setPower(1);
                }
            }
            if (dropperDC.getCurrentPosition() >= 900) {
                if (!gamepad2.a && gamepad2.b) {
                    dropperDC.setPower(0);
                }
            }
            if (dropperLimit.isPressed()) {
                if (gamepad2.a && !gamepad2.b) {
                    dropperDC.setPower(0);
                }
            }
            if (!dropperLimit.isPressed()) {
                if (dropperDC.getCurrentPosition() > 200) {
                    if (gamepad2.a && !gamepad2.b) {
                        dropperDC.setPower(-1);
                    }
                }
                if (dropperDC.getCurrentPosition() <= 200) {
                    if (gamepad2.a && !gamepad2.b) {
                        dropperDC.setPower(-0.4);
                    }
                }
            }
            if (!gamepad2.a && !gamepad2.b) {
                dropperDC.setPower(0);
            }

            telemetry.addData("ddc: ", dropperDC.getCurrentPosition());
            telemetry.update();
        }
    }
}
