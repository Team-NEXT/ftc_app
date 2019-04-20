package org.firstinspires.ftc.teamcode.AUTO;

import android.view.Display;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.ar.pl.DebugLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Autonomous(name = "Crater", group = "final")

public class Crater extends LinearOpMode {

    //DRIVE
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

    private DcMotor yAxisDC;

    //LATCHING
    private DcMotor latchingDC;
    private ModernRoboticsTouchSensor latchUpperLimit;
    private ModernRoboticsTouchSensor latchLowerLimit;

    //COLLECTOR
    private DcMotor collectorDC;
    private CRServo sweeperServo;
    private Servo collectorServo;
    private Servo flapServo;
    private ModernRoboticsTouchSensor collectorLimit;

    private static double FO = 0.55, FC = 0.18;

    private double cPos;
    private double cMid;
    private double cInitial;
    private double cOpen;
    private double cMarker;
    private double cClose;
    private double cDrop;

    private static double singleTicks = 13;
    private static double degree = 34;

//    private double cSpeed;
//    private double cExtPos;

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
    private double xDown;
    private double yUp;
    private double yDown;

    private ElapsedTime timer = new ElapsedTime();

    private ElapsedTime gyroTimer = new ElapsedTime();

    private double startTime;

    private static IntegratingGyroscope gyro;
    private static ModernRoboticsI2cGyro mrgyro;

    private ElapsedTime runtime = new ElapsedTime();

    int mineralPos;

    @Override
    public void runOpMode() throws InterruptedException {

        //DRIVING
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");

        motorFrontRight.setDirection(REVERSE);
        motorBackRight.setDirection(REVERSE);

        yAxisDC = hardwareMap.dcMotor.get("yAxisEncoder");

//        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
//        motorFrontLeft.setMode(STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(STOP_AND_RESET_ENCODER);
//
//        motorFrontRight.setMode(RUN_USING_ENCODER);
//        motorFrontLeft.setMode(RUN_USING_ENCODER);
//        motorBackRight.setMode(RUN_USING_ENCODER);
//        motorBackLeft.setMode(RUN_USING_ENCODER);

        //LATCHING
        latchingDC = hardwareMap.dcMotor.get("latchingDC");
        latchingDC.setDirection(REVERSE);
        latchUpperLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "LD");
        latchLowerLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "LU");

        //COLLECTOR
        collectorDC = hardwareMap.dcMotor.get("collectDC");
        sweeperServo = hardwareMap.crservo.get("sweepServo");
        sweeperServo.setDirection(REVERSE);
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
        cMid = 0.6;
        cClose = 0.03;
        cInitial = 0.01;
        cDrop = 0.25;
        cMarker = 0.7;

        //DROPPING
        dropperDC = hardwareMap.dcMotor.get("dropDC");
        dropperServo = hardwareMap.servo.get("dServo");
        dropperLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "D");

        dLoad = 0.71;
        dUnload = 0.29;

        dropperDC.setDirection(REVERSE);

        dropperDC.setMode(STOP_AND_RESET_ENCODER);
        dropperDC.setMode(RUN_USING_ENCODER);

        //DRIVE ENCODERS
        xServo = hardwareMap.servo.get("xServo");
        yServo = hardwareMap.servo.get("yServo");

        xUp = 0.4;
        xDown = 0.64;
        yUp = 0.59;
        yDown = 0.39;

        //INITIALIZATION

        runtime.reset();

        dropperServo.setPosition(dLoad);

        xServo.setPosition(xUp);
        yServo.setPosition(yUp);
        flapServo.setPosition(FC);

        collectorDC.setPower(0);
        while(!collectorLimit.isPressed()) {
            collectorDC.setPower(-0.7);
            telemetry.addLine("collector initializing");
            telemetry.update();
        }
        collectorDC.setPower(0);

        collectorServo.setPosition(cInitial);

        telemetry.addLine("initialized");
        telemetry.update();
        //GYRO

        mrgyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) mrgyro;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        mrgyro.calibrate();

        // Wait until the gyro calibration is complete
        timer.reset();
        while (!isStopRequested() && mrgyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        mineralPos = 4;

        waitForStart();

        /**CODE AFTER STARTING*/

        LATCHING(1);

//        Thread.sleep(3000);

        yServo.setPosition(yDown);
        xServo.setPosition(xDown);

        BACKWARD(50, 0.2);

        Thread.sleep(100);


        SWAYRIGHT(300);
        RAMPFORWARD(890, 0.1, 0.1, 0.5, 0.05, 0.06);
        Thread.sleep(200);
        GYROAXISLEFT(42, 0.0074, 900);
//        AXISLEFT(41, 0.3);
        collectorDC.setPower(0);
        collectorDC.setMode(STOP_AND_RESET_ENCODER);
        collectorDC.setMode(RUN_USING_ENCODER);
        while (collectorDC.getCurrentPosition() < 2000) {
            collectorDC.setPower(1);
        }
        collectorDC.setPower(0);
        while (collectorServo.getPosition() < cMarker) {
            cPos = collectorServo.getPosition();
            cPos += 0.03;
            collectorServo.setPosition(cPos);
        }
        if (cPos > cMarker) {
            collectorServo.setPosition(cMarker);
        }
        sweeperServo.setPower(-1);
        Thread.sleep(700);
        sweeperServo.setPower(0);
        while (collectorDC.getCurrentPosition() > 500) {
            collectorDC.setPower(-1);
        }
        while (!collectorLimit.isPressed()) {
            collectorDC.setPower(-0.4);
        }
        collectorDC.setPower(0);
        while (collectorServo.getPosition() > cClose) {
            cPos = collectorServo.getPosition();
            cPos -= 0.07;
            collectorServo.setPosition(cPos);
        }
        if (cPos < cClose) {
            collectorServo.setPosition(cClose);
        }

        if (mineralPos == 1) {
            GYROAXISRIGHT(-41, 0.00864, 900);
//            FORWARD(830, 0.3);
            RAMPBACKWARD(1050, -0.1, -0.1, -0.5, 0.05, 0.06);
            GYROAXISRIGHT(-85, 0.0049, 900);
            FORWARD(175, 0.3);
            collectorDC.setPower(0);
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 300) {
                collectorDC.setPower(1);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() < cOpen) {
                cPos = collectorServo.getPosition();
                cPos += 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos > cOpen) {
                collectorServo.setPosition(cOpen);
            }
            flapServo.setPosition(FC);
            sweeperServo.setPower(1);
            while (collectorDC.getCurrentPosition() < 900) {
                collectorDC.setPower(0.6);
            }
            collectorDC.setPower(0);
            Thread.sleep(500);
            while (collectorServo.getPosition() > cMid) {
                cPos = collectorServo.getPosition();
                cPos -= 0.045;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cMid) {
                collectorServo.setPosition(cMid);
            }
            while (collectorDC.getCurrentPosition() > 350) {
                collectorDC.setPower(-1);
            }
            collectorDC.setPower(0);
            while (!collectorLimit.isPressed()) {
                collectorDC.setPower(-0.4);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() > cClose) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cClose) {
                collectorServo.setPosition(cClose);
            }
            flapServo.setPosition(FO);
            GYROAXISRIGHT(-15, 0.0075, 900);
            BACKWARD(85, 0.3);
            sweeperServo.setPower(0);
            DROP();
            collectorDC.setPower(0);
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 700) {
                collectorDC.setPower(1);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() < cOpen) {
                cPos = collectorServo.getPosition();
                cPos += 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos > cOpen) {
                collectorServo.setPosition(cOpen);
            }
            flapServo.setPosition(FC);
            sweeperServo.setPower(1);
            while (collectorDC.getCurrentPosition() < 1450) {
                collectorDC.setPower(0.6);
            }
            collectorDC.setPower(0);
            Thread.sleep(400);
            while (collectorServo.getPosition() > cMid) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cMid) {
                collectorServo.setPosition(cMid);
            }
            while (collectorDC.getCurrentPosition() > 420) {
                collectorDC.setPower(-1);
            }
            collectorDC.setPower(0);
            while (!collectorLimit.isPressed()) {
                collectorDC.setPower(-0.4);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() > cClose) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cClose) {
                collectorServo.setPosition(cClose);
            }
            flapServo.setPosition(FO);
            Thread.sleep(600);
            sweeperServo.setPower(0);
            halfDROP();
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 600) {
                collectorDC.setPower(1);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() < cMid) {
                cPos = collectorServo.getPosition();
                cPos += 0.8;
                collectorServo.setPosition(cPos);
            }
            if (cPos > cMid) {
                collectorServo.setPosition(cMid);
            }
//            Thread.sleep(200);
//            GYROAXISLEFT(124, 0.0048, 900);
//            FORWARD(760, 0.3);
//            GYROAXISLEFT(36, 0.0075, 900);
//            Thread.sleep(200);
//            FORWARD(50, 0.3);
//            collectorDC.setPower(0);
//            collectorDC.setMode(STOP_AND_RESET_ENCODER);
//            collectorDC.setMode(RUN_USING_ENCODER);
//            while (collectorDC.getCurrentPosition() < 400) {
//                collectorDC.setPower(1);
//            }
//            collectorDC.setPower(0);
//            while (collectorServo.getPosition() < cOpen) {
//                cPos = collectorServo.getPosition();
//                cPos += 0.05;
//                collectorServo.setPosition(cPos);
//            }
//            if (cPos > cOpen) {
//                collectorServo.setPosition(cOpen);
//            }
//            flapServo.setPosition(FC);
//            sweeperServo.setPower(1);
//            while (collectorDC.getCurrentPosition() < 700) {
//                collectorDC.setPower(0.4);
//            }
//            collectorDC.setPower(0);
//            Thread.sleep(600);
//            while (collectorServo.getPosition() > cMid) {
//                cPos = collectorServo.getPosition();
//                cPos -= 0.05;
//                collectorServo.setPosition(cPos);
//            }
//            if (cPos < cMid) {
//                collectorServo.setPosition(cMid);
//            }
//            while (collectorDC.getCurrentPosition() > 350) {
//                collectorDC.setPower(-1);
//            }
//            collectorDC.setPower(0);
//            while (!collectorLimit.isPressed()) {
//                collectorDC.setPower(-0.4);
//            }
//            collectorDC.setPower(0);
//            while (collectorServo.getPosition() > cClose) {
//                cPos = collectorServo.getPosition();
//                cPos -= 0.05;
//                collectorServo.setPosition(cPos);
//            }
//            if (cPos < cClose) {
//                collectorServo.setPosition(cClose);
//            }
//            flapServo.setPosition(FO);
//            GYROAXISLEFT(46, 0.007, 900);
//            SWEEPER(0);
//            DROPlow();
//            while (collectorDC.getCurrentPosition() < 900) {
//                collectorDC.setPower(1);
//            }
//            collectorDC.setPower(0);
//            while (collectorServo.getPosition() < cOpen) {
//                cPos = collectorServo.getPosition();
//                cPos += 0.05;
//                collectorServo.setPosition(cPos);
//            }
//            if (cPos > cOpen) {
//                collectorServo.setPosition(cOpen);
//            }
//            flapServo.setPosition(FC);
//            sweeperServo.setPower(1);
//            while (collectorDC.getCurrentPosition() < 1400) {
//                collectorDC.setPower(0.4);
//            }
//            collectorDC.setPower(0);
//            Thread.sleep(800);
//            while (collectorServo.getPosition() > cMid) {
//                cPos = collectorServo.getPosition();
//                cPos -= 0.05;
//                collectorServo.setPosition(cPos);
//            }
//            if (cPos < cMid) {
//                collectorServo.setPosition(cMid);
//            }
//            while (collectorDC.getCurrentPosition() > 420) {
//                collectorDC.setPower(-1);
//            }
//            collectorDC.setPower(0);
//            while (!collectorLimit.isPressed()) {
//                collectorDC.setPower(-0.4);
//            }
//            collectorDC.setPower(0);
//            while (collectorServo.getPosition() > cClose) {
//                cPos = collectorServo.getPosition();
//                cPos -= 0.05;
//                collectorServo.setPosition(cPos);
//            }
//            if (cPos < cClose) {
//                collectorServo.setPosition(cClose);
//            }
//            flapServo.setPosition(FO);
//            Thread.sleep(600);
//            sweeperServo.setPower(0);
//            halfDROP_low();
//            collectorDC.setMode(STOP_AND_RESET_ENCODER);
//            collectorDC.setMode(RUN_USING_ENCODER);
//            while (collectorDC.getCurrentPosition() < 950) {
//                collectorDC.setPower(1);
//            }
//            collectorDC.setPower(0);
//            while (collectorServo.getPosition() < cMid) {
//                cPos = collectorServo.getPosition();
//                cPos += 0.8;
//                collectorServo.setPosition(cPos);
//            }
//            if (cPos > cMid) {
//                collectorServo.setPosition(cMid);
//            }
        }

        if (mineralPos == 2) {
            GYROAXISRIGHT(-41, 0.0087, 900);
//            FORWARD(830, 0.3);
            RAMPBACKWARD(700, -0.1, -0.1, -0.5, 0.05, 0.06);
            GYROAXISRIGHT(-85, 0.0049, 900);
            FORWARD(200, 0.3);
            collectorDC.setPower(0);
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 300) {
                collectorDC.setPower(1);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() < cOpen) {
                cPos = collectorServo.getPosition();
                cPos += 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos > cOpen) {
                collectorServo.setPosition(cOpen);
            }
            flapServo.setPosition(FC);
            sweeperServo.setPower(1);
            while (collectorDC.getCurrentPosition() < 1000) {
                collectorDC.setPower(0.6);
            }
            collectorDC.setPower(0);
            Thread.sleep(600);
            while (collectorServo.getPosition() > cMid) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cMid) {
                collectorServo.setPosition(cMid);
            }
            while (collectorDC.getCurrentPosition() > 350) {
                collectorDC.setPower(-1);
            }
            collectorDC.setPower(0);
            while (!collectorLimit.isPressed()) {
                collectorDC.setPower(-0.4);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() > cClose) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cClose) {
                collectorServo.setPosition(cClose);
            }
            flapServo.setPosition(FO);
//            tank(60, 0, -0.3);
            BACKWARD(80, 0.25);
            GYRO_RBPIVOT(-4, 0.07, 900);
            Thread.sleep(400);
            sweeperServo.setPower(0);
            DROP();
            collectorDC.setPower(0);
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 535) {
                collectorDC.setPower(1);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() < cOpen) {
                cPos = collectorServo.getPosition();
                cPos += 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos > cOpen) {
                collectorServo.setPosition(cOpen);
            }
            flapServo.setPosition(FC);
            sweeperServo.setPower(1);
            while (collectorDC.getCurrentPosition() < 1300) {
                collectorDC.setPower(0.6);
            }
            collectorDC.setPower(0);
            Thread.sleep(100);
            while (collectorServo.getPosition() > cMid) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cMid) {
                collectorServo.setPosition(cMid);
            }
            while (collectorDC.getCurrentPosition() > 420) {
                collectorDC.setPower(-1);
            }
            collectorDC.setPower(0);
            while (!collectorLimit.isPressed()) {
                collectorDC.setPower(-0.4);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() > cClose) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cClose) {
                collectorServo.setPosition(cClose);
            }
            flapServo.setPosition(FO);
            Thread.sleep(750);
            sweeperServo.setPower(0);
            halfDROP();
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 600) {
                collectorDC.setPower(1);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() < cMid) {
                cPos = collectorServo.getPosition();
                cPos += 0.8;
                collectorServo.setPosition(cPos);
            }
            if (cPos > cMid) {
                collectorServo.setPosition(cMid);
            }
        }
        /**3*/
        if (mineralPos == 3) {
            GYROAXISRIGHT(-42, 0.0078, 900);
//            FORWARD(830, 0.3);
            RAMPBACKWARD(500, -0.1, -0.1, -0.4, 0.05, 0.06);
            GYROAXISRIGHT(-85, 0.0049, 900);
            FORWARD(150, 0.3);
            collectorDC.setPower(0);
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 300) {
                collectorDC.setPower(1);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() < cOpen) {
                cPos = collectorServo.getPosition();
                cPos += 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos > cOpen) {
                collectorServo.setPosition(cOpen);
            }
            flapServo.setPosition(FC);
            sweeperServo.setPower(1);
            while (collectorDC.getCurrentPosition() < 1000) {
                collectorDC.setPower(0.6);
            }
            collectorDC.setPower(0);
            Thread.sleep(600);
            while (collectorServo.getPosition() > cMid) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cMid) {
                collectorServo.setPosition(cMid);
            }
            while (collectorDC.getCurrentPosition() > 350) {
                collectorDC.setPower(-1);
            }
            collectorDC.setPower(0);
            while (!collectorLimit.isPressed()) {
                collectorDC.setPower(-0.4);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() > cClose) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cClose) {
                collectorServo.setPosition(cClose);
            }
            flapServo.setPosition(FO);
            BACKWARD(50, 0.3);
            GYROAXISLEFT(6, 0.0095, 900);
            DROP();
            collectorDC.setPower(0);
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 535) {
                collectorDC.setPower(1);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() < cOpen) {
                cPos = collectorServo.getPosition();
                cPos += 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos > cOpen) {
                collectorServo.setPosition(cOpen);
            }
            flapServo.setPosition(FC);
            sweeperServo.setPower(1);
            while (collectorDC.getCurrentPosition() < 1350) {
                collectorDC.setPower(0.6);
            }
            collectorDC.setPower(0);
            Thread.sleep(750);
            while (collectorServo.getPosition() > cMid) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cMid) {
                collectorServo.setPosition(cMid);
            }
            while (collectorDC.getCurrentPosition() > 420) {
                collectorDC.setPower(-1);
            }
            collectorDC.setPower(0);
            while (!collectorLimit.isPressed()) {
                collectorDC.setPower(-0.4);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() > cClose) {
                cPos = collectorServo.getPosition();
                cPos -= 0.05;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cClose) {
                collectorServo.setPosition(cClose);
            }
            flapServo.setPosition(FO);
            Thread.sleep(750);
            sweeperServo.setPower(0);
            halfDROP();
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 600) {
                collectorDC.setPower(1);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() < cMid) {
                cPos = collectorServo.getPosition();
                cPos += 0.8;
                collectorServo.setPosition(cPos);
            }
            if (cPos > cMid) {
                collectorServo.setPosition(cMid);
            }
        }
        /**TO TEST RANDOM SHIT*/
        if (mineralPos == 4) {
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            dropperDC.setMode(STOP_AND_RESET_ENCODER);
            dropperDC.setMode(RUN_USING_ENCODER);
            collectorDC.setTargetPosition(1800);
            dropperDC.setTargetPosition(900);
            collectorDC.setMode(RUN_TO_POSITION);
            dropperDC.setMode(RUN_TO_POSITION);
        }
//
        Thread.sleep(1000);
//        COLLECTORCONTRACT(0.8);
//
    }
//
    /**Methods*/
    public void FORWARD (int mmDistance, double power) {

        double targetTicks = mmDistance * singleTicks;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) > Math.abs(yAxisDC.getCurrentPosition())) {
            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(yAxisDC.getCurrentPosition()));
            telemetry.addData("target: ", (targetTicks-100));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void cFORWARD (int mmDistance, double power) {

        double targetTicks = mmDistance * singleTicks;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) >= Math.abs(yAxisDC.getCurrentPosition())) {
            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(yAxisDC.getCurrentPosition()));
            telemetry.addData("target: ", (targetTicks-100));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        double correctionTicks = yAxisDC.getCurrentPosition() - targetTicks;

        if (Math.abs(yAxisDC.getCurrentPosition()) > (targetTicks+100)) {
            telemetry.addLine("over");
            telemetry.update();
            yAxisDC.setMode(STOP_AND_RESET_ENCODER);
            yAxisDC.setMode(RUN_USING_ENCODER);
            while (Math.abs(yAxisDC.getCurrentPosition()) < correctionTicks-100) {
                motorFrontRight.setPower(-0.15);
                motorFrontLeft.setPower(-0.15);
                motorBackLeft.setPower(-0.15);
                motorBackRight.setPower(-0.15);
            }
        } else if (Math.abs(yAxisDC.getCurrentPosition()) < (targetTicks-100)) {
            telemetry.addLine("under");
            telemetry.update();
            yAxisDC.setMode(STOP_AND_RESET_ENCODER);
            yAxisDC.setMode(RUN_USING_ENCODER);
            while (Math.abs(yAxisDC.getCurrentPosition()) < correctionTicks-100) {
                motorFrontRight.setPower(0.15);
                motorFrontLeft.setPower(0.15);
                motorBackLeft.setPower(0.15);
                motorBackRight.setPower(0.15);
            }
        } else {
            telemetry.addLine("correct");
            telemetry.update();
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
        telemetry.addLine("stop");
        telemetry.update();
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void RAMPFORWARD (int mmDistance, double startPower, double endPower, double maxPower, double increment, double decrement) {

        double targetTicks = mmDistance * singleTicks;
        double power = startPower;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) > Math.abs(yAxisDC.getCurrentPosition())) {

            if (Math.abs(yAxisDC.getCurrentPosition()) <= (targetTicks/2)) {
                telemetry.addData("RUP", "");
                if (power > maxPower) {
                    power = maxPower;
                } else {
                    power += increment;
                }
            } else if (Math.abs(yAxisDC.getCurrentPosition()) >= (targetTicks/2)) {
                telemetry.addData("RD", "");
                if (power < endPower) {
                    power = endPower;
                } else {
                    power -= decrement;
                }
            }

            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(yAxisDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void RAMPBACKWARD (int mmDistance, double startPower, double endPower, double maxPower, double increment, double decrement) {

        double targetTicks = mmDistance * singleTicks;
        double power = startPower;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) > Math.abs(yAxisDC.getCurrentPosition())) {

            if (Math.abs(yAxisDC.getCurrentPosition()) <= (targetTicks/2)) {
                telemetry.addData("RUP", "");
                if (power < maxPower) {
                    power = maxPower;
                } else {
                    power -= increment;
                }
            } else if (Math.abs(yAxisDC.getCurrentPosition()) >= (targetTicks/2)) {
                telemetry.addData("RD", "");
                if (power > endPower) {
                    power = endPower;
                } else {
                    power += decrement;
                }
            }

            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(yAxisDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void cRAMPFORWARD (int mmDistance, double startPower, double endPower, double maxPower, double increment, double decrement) {

        double targetTicks = mmDistance * singleTicks;
        double power = startPower;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) > Math.abs(yAxisDC.getCurrentPosition())) {

            if (Math.abs(yAxisDC.getCurrentPosition()) <= (targetTicks/2)) {
                telemetry.addData("RUP", "");
                if (power > maxPower) {
                    power = maxPower;
                } else {
                    power += increment;
                }
            } else if (Math.abs(yAxisDC.getCurrentPosition()) >= (targetTicks/2)) {
                telemetry.addData("RD", "");
                if (power < endPower) {
                    power = endPower;
                } else {
                    power -= decrement;
                }
            }

            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(yAxisDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        double correctionTicks = yAxisDC.getCurrentPosition() - targetTicks;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        telemetry.addData("y: ", Math.abs(yAxisDC.getCurrentPosition()));
        telemetry.addData("correctionticks: ", correctionTicks);
        telemetry.update();

//        if (Math.abs(yAxisDC.getCurrentPosition()) > (targetTicks+100)) {
//            telemetry.addLine("over");
//            telemetry.update();
//            yAxisDC.setMode(STOP_AND_RESET_ENCODER);
//            yAxisDC.setMode(RUN_USING_ENCODER);
//            while (Math.abs(yAxisDC.getCurrentPosition()) < correctionTicks) {
//                motorFrontRight.setPower(-0.2);
//                motorFrontLeft.setPower(-0.2);
//                motorBackLeft.setPower(-0.2);
//                motorBackRight.setPower(-0.2);
//            }
//        } else if (Math.abs(yAxisDC.getCurrentPosition()) < (targetTicks-100)) {
//            telemetry.addLine("under");
//            telemetry.update();
//            yAxisDC.setMode(STOP_AND_RESET_ENCODER);
//            yAxisDC.setMode(RUN_USING_ENCODER);
//            while (Math.abs(yAxisDC.getCurrentPosition()) < Math.abs(correctionTicks)) {
//                motorFrontRight.setPower(0.2);
//                motorFrontLeft.setPower(0.2);
//                motorBackLeft.setPower(0.2);
//                motorBackRight.setPower(0.2);
//            }
//        } else {
//            telemetry.addLine("correct");
//            telemetry.update();
//            motorFrontRight.setPower(0);
//            motorFrontLeft.setPower(0);
//            motorBackLeft.setPower(0);
//            motorBackRight.setPower(0);
//        }
        telemetry.addData("correctionTicks: ", correctionTicks);
        telemetry.addData("y:", yAxisDC.getCurrentPosition());
        telemetry.addLine("stop");
        telemetry.update();
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }
//        targetTicks = mmDistance * singleTicks;
//
//        latchingDC.setMode(STOP_AND_RESET_ENCODER);
//        latchingDC.setMode(RUN_USING_ENCODER);
//
//        while ((targetTicks - 100) > Math.abs(latchingDC.getCurrentPosition())) {
//            motorFrontRight.setPower(power);
//            motorBackRight.setPower(power);
//            motorFrontLeft.setPower(power);
//            motorBackLeft.setPower(power);
//
//            telemetry.addData("FR", motorFrontRight.getPower());
//            telemetry.addData("BR", motorBackRight.getPower());
//            telemetry.addData("FL", motorFrontLeft.getPower());
//            telemetry.addData("BL", motorBackLeft.getPower());
//            telemetry.addData("y: ", Math.abs(latchingDC.getCurrentPosition()));
//            telemetry.update();
//        }
//
//    }

    public void BACKWARD (int mmDistance, double power) {

        double targetTicks = mmDistance * singleTicks;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(yAxisDC.getCurrentPosition())) {
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(yAxisDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void COASTBACKWARD (int mmDistance, double power) {

        double targetTicks = mmDistance * singleTicks;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(yAxisDC.getCurrentPosition())) {
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(yAxisDC.getCurrentPosition()));
            telemetry.update();
        }

    }

    public void SWAYLEFT (int mmDistance) {

        double targetTicks = mmDistance * singleTicks;

        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(latchingDC.getCurrentPosition())) {
            motorFrontRight.setPower(0.4);
            motorBackRight.setPower(-0.4);
            motorFrontLeft.setPower(-0.4);
            motorBackLeft.setPower(0.4);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("x: ", Math.abs(latchingDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }
//
//    public void C2_SWAYLEFT (int mmDistance) {
//
//        double targetTicks = mmDistance * singleTicks;
//
//        latchingDC.setMode(STOP_AND_RESET_ENCODER);
//        latchingDC.setMode(RUN_USING_ENCODER);
//
//        while ((targetTicks-100) > Math.abs(latchingDC.getCurrentPosition())) {
//            motorFrontRight.setPower(0.5);
//            motorBackRight.setPower(-0.4);
//            motorFrontLeft.setPower(-0.6);
//            motorBackLeft.setPower(0.8);
//
//            telemetry.addData("FR", motorFrontRight.getPower());
//            telemetry.addData("BR", motorBackRight.getPower());
//            telemetry.addData("FL", motorFrontLeft.getPower());
//            telemetry.addData("BL", motorBackLeft.getPower());
//            telemetry.addData("x: ", Math.abs(latchingDC.getCurrentPosition()));
//            telemetry.update();
//        }
//
//        motorFrontRight.setPower(0);
//        motorBackRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//
//    }

//    public void COASTSWAYLEFT (int mmDistance, double power) {
//
//        targetTicks = mmDistance * singleTicks;
//
//        sweeperDC.setMode(STOP_AND_RESET_ENCODER);
//        sweeperDC.setMode(RUN_USING_ENCODER);
//
//        while ((targetTicks-100) > Math.abs(sweeperDC.getCurrentPosition())) {
//            motorFrontRight.setPower(power);
//            motorBackRight.setPower(-power);
//            motorFrontLeft.setPower(-power);
//            motorBackLeft.setPower(power);
//
//            telemetry.addData("FR", motorFrontRight.getPower());
//            telemetry.addData("BR", motorBackRight.getPower());
//            telemetry.addData("FL", motorFrontLeft.getPower());
//            telemetry.addData("BL", motorBackLeft.getPower());
//            telemetry.addData("x: ", Math.abs(sweeperDC.getCurrentPosition()));
//            telemetry.update();
//        }
//
//    }

    public void SWAYRIGHT (int mmDistance) {

        double targetTicks = mmDistance * singleTicks;

        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(latchingDC.getCurrentPosition())) {
            motorFrontRight.setPower(-0.43);
            motorBackRight.setPower(0.4);
            motorFrontLeft.setPower(0.43);
            motorBackLeft.setPower(-0.4);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("x: ", Math.abs(latchingDC.getCurrentPosition()));
            telemetry.update();
        }

        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);

    }

    public void SWAYRIGHT1 (int mmDistance) {

        double targetTicks = mmDistance * singleTicks;

        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(latchingDC.getCurrentPosition())) {
            motorFrontRight.setPower(-0.45);
            motorBackRight.setPower(0.4);
            motorFrontLeft.setPower(0.45);
            motorBackLeft.setPower(-0.4);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("x: ", Math.abs(latchingDC.getCurrentPosition()));
            telemetry.update();
        }

        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);

    }

//    public void SWAYRIGHT2 (int mmDistance) {
//
//        targetTicks = mmDistance * singleTicks;
//
//        sweeperDC.setMode(STOP_AND_RESET_ENCODER);
//        sweeperDC.setMode(RUN_USING_ENCODER);
//
//        while ((targetTicks-100) > Math.abs(sweeperDC.getCurrentPosition())) {
//            motorFrontRight.setPower(-0.6);
//            motorBackRight.setPower(0.4);
//            motorFrontLeft.setPower(0.8);
//            motorBackLeft.setPower(-0.8);
//
//            telemetry.addData("FR", motorFrontRight.getPower());
//            telemetry.addData("BR", motorBackRight.getPower());
//            telemetry.addData("FL", motorFrontLeft.getPower());
//            telemetry.addData("BL", motorBackLeft.getPower());
//            telemetry.addData("x: ", Math.abs(sweeperDC.getCurrentPosition()));
//            telemetry.update();
//        }
//
//        motorFrontRight.setPower(0);
//        motorBackRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//
//    }

    public void LATCHING (double power) {
        while (!latchUpperLimit.isPressed()) {
            latchingDC.setPower(power);
        }
        latchingDC.setPower(0);
    }

    public void AXISLEFT (double degrees, double power) {

        double targetTicks = degrees * degree;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) > Math.abs(yAxisDC.getCurrentPosition())) {
            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);
            motorBackRight.setPower(power);
        }

        telemetry.addData("turning ", power);
        telemetry.addData("FR: ", motorFrontRight.getPower());
        telemetry.addData("BR: ", motorBackRight.getPower());
        telemetry.addData("FL: ", motorFrontLeft.getPower());
        telemetry.addData("BL: ", motorBackLeft.getPower());
        telemetry.addData("y: ", Math.abs(yAxisDC.getCurrentPosition()));
        telemetry.update();

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void AXISRIGHT (double power, double degrees) {

        double targetTicks = degrees * degree;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) > Math.abs(yAxisDC.getCurrentPosition())) {
            motorFrontRight.setPower(-power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(-power);
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void COLLECTOREXPAND (double limit, double power) {

        //limit 1600 max
        collectorDC.setMode(STOP_AND_RESET_ENCODER);
        collectorDC.setMode(RUN_USING_ENCODER);

        while ((limit - 10) < Math.abs(collectorDC.getCurrentPosition())) {
            collectorDC.setPower(power);
        }

        collectorDC.setPower(0);

    }

    public void COLLECTORCONTRACT (double power){

        collectorDC.setMode(STOP_AND_RESET_ENCODER);
        collectorDC.setMode(RUN_USING_ENCODER);

        while (!collectorLimit.isPressed()) {
            collectorDC.setPower(-power);
        }

        collectorDC.setPower(0);

    }

    public void CSERVODOWN (double power) { //0.02

        while (collectorServo.getPosition() < cMid) {
            cPos = collectorServo.getPosition();
            cPos += power;
            collectorServo.setPosition(cPos);
        }
        if (cPos > cMid) {
            cPos = cMid;
            collectorServo.setPosition(cPos);
        }

    }

    public void SWEEPER (double power) {

        sweeperServo.setPower(power);

    }

    public void CSERVOUP (double power) { //0.07

        while (collectorServo.getPosition() > cClose) {
            cPos = collectorServo.getPosition();
            cPos -= power;
            collectorServo.setPosition(cPos);
        }
        if (cPos < cClose) {
            cPos = cClose;
            collectorServo.setPosition(cPos);
        }

    }

//    public void COASTSWAYRIGHT (int mmDistance, double power) {
//
//        targetTicks = mmDistance * singleTicks;
//
//        sweeperDC.setMode(STOP_AND_RESET_ENCODER);
//        sweeperDC.setMode(RUN_USING_ENCODER);
//
//        while ((targetTicks - 100) > Math.abs(sweeperDC.getCurrentPosition())) {
//            motorFrontRight.setPower(-power);
//            motorBackRight.setPower(power);
//            motorFrontLeft.setPower(power);
//            motorBackLeft.setPower(-power);
//
//            telemetry.addData("FR", motorFrontRight.getPower());
//            telemetry.addData("BR", motorBackRight.getPower());
//            telemetry.addData("FL", motorFrontLeft.getPower());
//            telemetry.addData("BL", motorBackLeft.getPower());
//            telemetry.addData("x: ", Math.abs(sweeperDC.getCurrentPosition()));
//            telemetry.update();
//        }
//
//    }

//    public void RAMP_FORWARD (int mmDistance, double power, double lowerLimit, double rampDownFactor) {
//
//        targetTicks = mmDistance * singleTicks;
//
//        rampTicks = targetTicks * 0.4;
//
//        while (targetTicks*0.6 > Math.abs(latchingDC.getCurrentPosition())) {
//            motorFrontRight.setPower(power);
//            motorBackRight.setPower(power);
//            motorFrontLeft.setPower(power);
//            motorBackLeft.setPower(power);
//        }
//
//        while ((targetTicks-100) > Math.abs(latchingDC.getCurrentPosition())) {
//            power -= rampDownFactor;
//            if (power <= lowerLimit) {
//                power = lowerLimit;
//            }
//            motorFrontRight.setPower(power);
//            motorBackRight.setPower(power);
//            motorFrontLeft.setPower(power);
//            motorBackLeft.setPower(power);
//        }
//
//        motorFrontRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//        motorBackRight.setPower(0);
//
//    }
//
//    public void RAMP_BACKWARD (int mmDistance, double power, double lowerLimit, double rampDownFactor) {
//
//        targetTicks = mmDistance * singleTicks;
//
//        rampTicks = targetTicks * 0.25;
//
//        while (targetTicks*0.75 > Math.abs(latchingDC.getCurrentPosition())) {
//            motorFrontRight.setPower(-power);
//            motorBackRight.setPower(-power);
//            motorFrontLeft.setPower(-power);
//            motorBackLeft.setPower(-power);
//        }
//
//        while (targetTicks > Math.abs(latchingDC.getCurrentPosition())) {
//            power -= rampDownFactor;
//            if (power <= lowerLimit) {
//                power = lowerLimit;
//            }
//            motorFrontRight.setPower(-power);
//            motorBackRight.setPower(-power);
//            motorFrontLeft.setPower(-power);
//            motorBackLeft.setPower(-power);
//        }
//
//        motorFrontRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//        motorBackRight.setPower(0);
//
//    }

//    public void RAMP_SWAYLEFT (int mmDistance, double power, double lowerLimit, double rampDownFactor) {
//
//        targetTicks = mmDistance * singleTicks;
//
//        rampTicks = targetTicks * 0.25;
//
//        while (targetTicks*0.75 > Math.abs(sweeperDC.getCurrentPosition())) {
//            motorFrontRight.setPower(power);
//            motorBackRight.setPower(-power);
//            motorFrontLeft.setPower(-power);
//            motorBackLeft.setPower(power);
//        }
//
//        while (targetTicks > Math.abs(sweeperDC.getCurrentPosition())) {
//            power -= rampDownFactor;
//            if (power <= lowerLimit) {
//                power = lowerLimit;
//            }
//            motorFrontRight.setPower(power);
//            motorBackRight.setPower(-power);
//            motorFrontLeft.setPower(-power);
//            motorBackLeft.setPower(power);
//        }
//
//        motorFrontRight.setPower(0);
//        motorBackRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//
//    }

//    public void RAMP_SWAYRIGHT (int mmDistance, double power, double lowerLimit, double rampDownFactor) {
//
//        targetTicks = mmDistance * singleTicks;
//
//        rampTicks = targetTicks * 0.25;
//
//        while (targetTicks*0.75 > Math.abs(sweeperDC.getCurrentPosition())) {
//            motorFrontRight.setPower(-power);
//            motorBackRight.setPower(power);
//            motorFrontLeft.setPower(power);
//            motorBackLeft.setPower(-power);
//        }
//
//        while (targetTicks > Math.abs(sweeperDC.getCurrentPosition())) {
//            power -= rampDownFactor;
//            if (power <= lowerLimit) {
//                power = lowerLimit;
//            }
//            motorFrontRight.setPower(-power);
//            motorBackRight.setPower(power);
//            motorFrontLeft.setPower(power);
//            motorBackLeft.setPower(-power);
//        }
//
//        motorFrontRight.setPower(0);
//        motorBackRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//
//    }

//    public void GYRO_TANK (double targetHeading, double leftPower, double rightPower) {
//        while (Math.abs(mrgyro.getIntegratedZValue()) < (targetHeading-5)) {
//            motorFrontRight.setPower(rightPower);
//            motorBackRight.setPower(rightPower);
//            motorFrontLeft.setPower(leftPower);
//            motorBackLeft.setPower(leftPower);
//
//            telemetry.addData("z: ", mrgyro.getIntegratedZValue());
//            telemetry.update();
//        }
//        motorFrontRight.setPower(0);
//        motorBackRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//    }

    public void tank (double mm, double leftPower, double rightPower) {
        double targetTicks = mm * singleTicks;
        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);
        while (Math.abs(yAxisDC.getCurrentPosition()) < (targetTicks - 100)) {
            motorFrontRight.setPower(rightPower);
            motorFrontLeft.setPower(leftPower);
            motorBackLeft.setPower(leftPower);
            motorBackRight.setPower(rightPower);
        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void motorpower (double mm, double fr, double br, double fl, double bl) {
        double targetTicks = mm * singleTicks;
        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);
        while (Math.abs(yAxisDC.getCurrentPosition()) < (targetTicks - 100)) {
            motorFrontRight.setPower(fr);
            motorFrontLeft.setPower(fl);
            motorBackLeft.setPower(bl);
            motorBackRight.setPower(br);
        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void DROP() {
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
        while (dropperServo.getPosition() > dUnload) {
            dPos = dropperServo.getPosition();
            dPos -= 0.035;
            dropperServo.setPosition(dPos);
        }
        if (dPos < dUnload) {
            dropperServo.setPosition(dUnload);
        }
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {

        }
        while (dropperServo.getPosition() > dLoad) {
            dPos = dropperServo.getPosition();
            dPos += 0.05;
            dropperServo.setPosition(dPos);
        }
        if (dPos < dLoad) {
            dPos = dLoad;
            dropperServo.setPosition(dPos);
        }
        while (dropperDC.getCurrentPosition() >= 400) {
            dropperDC.setPower(-1);
        }
        dropperDC.setPower(0);
        while (!dropperLimit.isPressed()) {
            dropperDC.setPower(-0.3);
        }
        dropperDC.setPower(0);
    }

    public void DROPlow() {
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
        while (dropperDC.getCurrentPosition() < 800) {
            dropperDC.setPower(1);
        }
        dropperDC.setPower(0);
        while (dropperServo.getPosition() > dUnload) {
            dPos = dropperServo.getPosition();
            dPos -= 0.04;
            dropperServo.setPosition(dPos);
        }
        if (dPos < dUnload) {
            dropperServo.setPosition(dUnload);
        }
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {

        }
        while (dropperServo.getPosition() > dLoad) {
            dPos = dropperServo.getPosition();
            dPos += 0.05;
            dropperServo.setPosition(dPos);
        }
        if (dPos < dLoad) {
            dPos = dLoad;
            dropperServo.setPosition(dPos);
        }
        while (dropperDC.getCurrentPosition() >= 400) {
            dropperDC.setPower(-1);
        }
        dropperDC.setPower(0);
        while (!dropperLimit.isPressed()) {
            dropperDC.setPower(-0.3);
        }
        dropperDC.setPower(0);
    }

    public void halfDROP() {
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
        while (dropperServo.getPosition() > dUnload) {
            dPos = dropperServo.getPosition();
            dPos -= 0.035;
            dropperServo.setPosition(dPos);
        }
        if (dPos < dUnload) {
            dropperServo.setPosition(dUnload);
        }
    }

    public void halfDROP_low() {
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
        while (dropperDC.getCurrentPosition() < 800) {
            dropperDC.setPower(1);
        }
        dropperDC.setPower(0);
        while (dropperServo.getPosition() > dUnload) {
            dPos = dropperServo.getPosition();
            dPos -= 0.04;
            dropperServo.setPosition(dPos);
        }
        if (dPos < dUnload) {
            dropperServo.setPosition(dUnload);
        }
    }

    public void GYROAXISLEFT (double targetHeading, double multiplicationFactor, double maxT) {

        mrgyro.resetZAxisIntegrator();

        double error, correction;
        double exitTime = 0;

//        inRange = false;

        boolean loopBreak = false;

        float gTime = 0;
        float gTimeStart = System.currentTimeMillis();

        runtime.reset();
        while ((mrgyro.getIntegratedZValue() < (targetHeading)) && !loopBreak) {
            error = targetHeading - mrgyro.getIntegratedZValue();
            correction = error * multiplicationFactor;

//            gTime = System.currentTimeMillis() - gTimeStart;

//            telemetry.addData("time: ", runtime.time());
            telemetry.addData("millis: ", runtime.milliseconds());
            telemetry.update();

            if (runtime.milliseconds() < maxT) {
                if (error > 0) {
                    //AXIS LEFT
                    motorFrontRight.setPower(correction);
                    motorFrontLeft.setPower(-correction);
                    motorBackLeft.setPower(-correction);
                    motorBackRight.setPower(correction);
                } else if (error < 0) {
                    //AXIS RIGHT
                    motorFrontRight.setPower(-correction);
                    motorFrontLeft.setPower(correction);
                    motorBackLeft.setPower(correction);
                    motorBackRight.setPower(-correction);
                } else {
                    motorFrontRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                }
            }
            else {
                exitTime = runtime.milliseconds();
                loopBreak = true;
            }

//            if (maxT > gTime) {
//                motorFrontRight.setPower(correction);
//                motorFrontLeft.setPower(-correction);
//                motorBackLeft.setPower(-correction);
//                motorBackRight.setPower(correction);
//            } else {
//                loopBreak = true;
//            }

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("z: ", mrgyro.getIntegratedZValue());
            telemetry.update();
        }

        telemetry.addData("gyro exited at: ", mrgyro.getIntegratedZValue());
        telemetry.addData("time elapsed: ", exitTime);
        telemetry.update();

        while (targetHeading != mrgyro.getIntegratedZValue()) {
            error = targetHeading - mrgyro.getIntegratedZValue();
            if (error > 0) {
                //AXIS LEFT
                motorFrontRight.setPower(0.15);
                motorFrontLeft.setPower(-0.15);
                motorBackLeft.setPower(-0.15);
                motorBackRight.setPower(0.15);
            } else if (error < 0) {
                //AXIS RIGHT
                motorFrontRight.setPower(-0.15);
                motorFrontLeft.setPower(0.15);
                motorBackLeft.setPower(0.15);
                motorBackRight.setPower(-0.15);
            } else {
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }
        }

        telemetry.addData("afterEC: ", mrgyro.getIntegratedZValue());
        telemetry.update();

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void GYROAXISRIGHT (double targetHeading, double multiplicationFactor, double maxT) {

        mrgyro.resetZAxisIntegrator();

        double error, correction;

//        inRange = false;

        float gTime = 0;
        float gTimeStart = System.currentTimeMillis();

        boolean loopBreak = false;

        runtime.reset();
        while ((mrgyro.getIntegratedZValue() > (targetHeading)) && (!loopBreak)) {
            error = targetHeading - mrgyro.getIntegratedZValue();
            correction = error * multiplicationFactor;

//            gTime = System.currentTimeMillis() - gTimeStart;

            if (runtime.milliseconds() < maxT) {
                if (error > 0) {
                    //AXIS LEFT
                    motorFrontRight.setPower(-correction);
                    motorFrontLeft.setPower(correction);
                    motorBackLeft.setPower(correction);
                    motorBackRight.setPower(-correction);
                } else if (error < 0) {
                    //AXIS RIGHT
                    motorFrontRight.setPower(correction);
                    motorFrontLeft.setPower(-correction);
                    motorBackLeft.setPower(-correction);
                    motorBackRight.setPower(correction);
                } else {
                    motorFrontRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                }
            } else {
                loopBreak = true;
            }

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("z: ", mrgyro.getIntegratedZValue());
            telemetry.update();
        }

        while (targetHeading != mrgyro.getIntegratedZValue()) {
            error = targetHeading - mrgyro.getIntegratedZValue();
            if (error > 0) {
                //AXIS LEFT
                motorFrontRight.setPower(0.15);
                motorFrontLeft.setPower(-0.15);
                motorBackLeft.setPower(-0.15);
                motorBackRight.setPower(0.15);
            } else if (error < 0) {
                //AXIS RIGHT
                motorFrontRight.setPower(-0.15);
                motorFrontLeft.setPower(0.15);
                motorBackLeft.setPower(0.15);
                motorBackRight.setPower(-0.15);
            } else {
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }
        }

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void GYRO_LBPIVOT (double targetHeading, double multiplicationFactor, double maxT) {

        mrgyro.resetZAxisIntegrator();

        double error, correction;

//        inRange = false;

        float gTime = 0;
        float gTimeStart = System.currentTimeMillis();

        while ((Math.abs(mrgyro.getIntegratedZValue()) < (targetHeading-1)) && (gTime < maxT)) {
            error = targetHeading - mrgyro.getIntegratedZValue();
            correction = error * multiplicationFactor;

            gTime = System.currentTimeMillis() - gTimeStart;

//            if (error > 0) {
//                //AXIS LEFT
//                motorFrontRight.setPower(correction);
//                motorFrontLeft.setPower(-correction);
//                motorBackLeft.setPower(-correction);
//                motorBackRight.setPower(correction);
//            } else if (error < 0) {
//                //AXIS RIGHT
//                motorFrontRight.setPower(-correction);
//                motorFrontLeft.setPower(correction);
//                motorBackLeft.setPower(correction);
//                motorBackRight.setPower(-correction);
//            } else {
//                motorFrontRight.setPower(0);
//                motorFrontLeft.setPower(0);
//                motorBackLeft.setPower(0);
//                motorBackRight.setPower(0);
//            }

            motorFrontRight.setPower(-correction);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(-correction);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("z: ", mrgyro.getIntegratedZValue());
            telemetry.update();
        }

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void GYRO_RBPIVOT (double targetHeading, double multiplicationFactor, double maxT) {

        mrgyro.resetZAxisIntegrator();

        double error, correction;

//        inRange = false;

        boolean loopBreak = false;

        float gTime = 0;
        float gTimeStart = System.currentTimeMillis();

        runtime.reset();
        while ((mrgyro.getIntegratedZValue() > targetHeading) && (!loopBreak)) {
            error = targetHeading - mrgyro.getIntegratedZValue();
            correction = error * multiplicationFactor;

            if (runtime.milliseconds() < maxT) {
                if (error < 0) {
                    //AXIS LEFT
                    motorFrontRight.setPower(correction);
                    motorFrontLeft.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(correction);
                } else if (error > 0) {
                    //AXIS RIGHT
                    motorFrontRight.setPower(-correction);
                    motorFrontLeft.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(-correction);
                } else {
                    motorFrontRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                }
            } else {
                loopBreak = true;
            }

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("z: ", mrgyro.getIntegratedZValue());
            telemetry.update();
        }

        while (targetHeading != mrgyro.getIntegratedZValue()) {
            error = targetHeading - mrgyro.getIntegratedZValue();
            if (error > 0) {
                //BP LEFT
                motorFrontRight.setPower(0.15);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0.15);
            } else if (error < 0) {
                //BP RIGHT
                motorFrontRight.setPower(-0.15);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(-0.15);
            } else {
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }
        }

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

}
