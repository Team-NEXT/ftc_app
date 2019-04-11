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
    private double cClose;

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


        //DROPPING
        dropperDC = hardwareMap.dcMotor.get("dropDC");
        dropperServo = hardwareMap.servo.get("dServo");
        dropperLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "D");

        dLoad = 0.71;
        dUnload = 0.3;

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

        mineralPos = 3;

        waitForStart();

        /**CODE AFTER STARTING*/

//        LATCHING(1);

        Thread.sleep(3000);

        yServo.setPosition(yDown);
        xServo.setPosition(xDown);

        BACKWARD(50, 0.2);

        Thread.sleep(100);

        /**CODE FOR IMAGE RECOGNITION*/

        SWAYRIGHT(300);
        RAMPFORWARD(890, 0.1, 0.1, 0.5, 0.05, 0.06);
        Thread.sleep(200);
        GYROAXISLEFT(42, 0.008, 800);
//        AXISLEFT(41, 0.3);
        collectorDC.setPower(0);
        collectorDC.setMode(STOP_AND_RESET_ENCODER);
        collectorDC.setMode(RUN_USING_ENCODER);
        while (collectorDC.getCurrentPosition() < 1800) {
            collectorDC.setPower(1);
        }
        collectorDC.setPower(0);
        while (collectorServo.getPosition() < cOpen) {
            cPos = collectorServo.getPosition();
            cPos += 0.05;
            collectorServo.setPosition(cPos);
        }
        if (cPos > cOpen) {
            cPos = cOpen;
            collectorServo.setPosition(cPos);
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

            //RAMPFORWARD(1000, 0.1, 0.1, 0.7, 0.005, 0.06);
            cRAMPFORWARD(600, 0.1, 0.1, 0.6, 0.05, 0.06);
            Thread.sleep(10000);

        }

        if (mineralPos == 2) {
            Thread.sleep(100);
            AXISLEFT(158, 0.3);
            FORWARD(350, 0.3);
            collectorDC.setPower(0);
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 750) {
                collectorDC.setPower(0.8);
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
                collectorDC.setPower(0.4);
            }
            collectorDC.setPower(0);
            Thread.sleep(1200);
            sweeperServo.setPower(0);
            while (collectorServo.getPosition() > cMid) {
                cPos = collectorServo.getPosition();
                cPos -= 0.07;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cMid) {
                cPos = cMid;
                collectorServo.setPosition(cPos);
            }
            sweeperServo.setPower(0.6);
            while (collectorDC.getCurrentPosition() > 400) {
                collectorDC.setPower(-1);
            }
            sweeperServo.setPower(0);
            while (collectorDC.getCurrentPosition() <= 400 && !collectorLimit.isPressed()) {
                collectorDC.setPower(-0.3);
            }
            collectorDC.setPower(0);
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() <= 180) {
                collectorDC.setPower(0.4);
            }
            collectorDC.setPower(0);
            while (collectorServo.getPosition() >= cClose) {
                cPos = collectorServo.getPosition();
                cPos -= 0.07;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cClose) {
                cPos = cClose;
                collectorServo.setPosition(cPos);
            }
            flapServo.setPosition(FO);
            SWEEPER(1);
            tank(135, 0, -0.3);
            RAMPFORWARD(500, 0.1, 0.1, 0.5, 0.05, 0.06);
            SWEEPER(0);
            AXISLEFT(82, 0.3);
            DROP();
            collectorDC.setMode(STOP_AND_RESET_ENCODER);
            collectorDC.setMode(RUN_USING_ENCODER);
            while (collectorDC.getCurrentPosition() < 470) {
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
            while (collectorDC.getCurrentPosition() < 1100) {
                collectorDC.setPower(0.4);
            }
            collectorDC.setPower(0);
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
            sweeperServo.setPower(0);
            while (collectorDC.getCurrentPosition() > 150) {
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
            sweeperServo.setPower(1);
            Thread.sleep(1000);
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
            Thread.sleep(100);
            GYROAXISLEFT(151, 0.005, 850);
//            AXISLEFT(165, 0.3);
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
            FORWARD(450, 0.3);
            Thread.sleep(500);
            while (collectorServo.getPosition() > cClose) {
                cPos = collectorServo.getPosition();
                cPos -= 0.07;
                collectorServo.setPosition(cPos);
            }
            if (cPos < cClose) {
                collectorServo.setPosition(cClose);
            }
            flapServo.setPosition(FO);
            Thread.sleep(200);
//            AXISLEFT(60, 0.3);
//            BACKWARD(160, 0.3);
//            sweeperServo.setPower(0);
//            Thread.sleep(100);
//            SWAYRIGHT(380);
//            FORWARD(50, 0.3);
//            DROP();
//            collectorDC.setMode(STOP_AND_RESET_ENCODER);
//            collectorDC.setMode(RUN_USING_ENCODER);
//            while (collectorDC.getCurrentPosition() < 800) {
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
//            while (collectorDC.getCurrentPosition() < 1700) {
//                collectorDC.setPower(0.4);
//            }
//            collectorDC.setPower(0);
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
//            sweeperServo.setPower(0);
//            while (collectorDC.getCurrentPosition() > 150) {
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
//            sweeperServo.setPower(1);
//            Thread.sleep(1200);
//            sweeperServo.setPower(0);
//            halfDROP();
//            collectorDC.setMode(STOP_AND_RESET_ENCODER);
//            collectorDC.setMode(RUN_USING_ENCODER);
//            while (collectorDC.getCurrentPosition() < 800) {
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
//
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
            motorFrontRight.setPower(-0.5);
            motorFrontLeft.setPower(0.4);
            motorBackLeft.setPower(-0.4);
            motorBackRight.setPower(0.5);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("x: ", Math.abs(latchingDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

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
        while (dropperDC.getCurrentPosition() < 950) {
            dropperDC.setPower(1);
        }
        dropperDC.setPower(0);
        dropperDC.setPower(0);
        dropperDC.setPower(0);
        while (dropperServo.getPosition() > dUnload) {
            dPos = dropperServo.getPosition();
            dPos -= 0.03;
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
            dPos += 0.04;
            collectorServo.setPosition(dPos);
        }
        if (dPos < dLoad) {
            dPos = dLoad;
            dropperServo.setPosition(dPos);
        }
        while (dropperDC.getCurrentPosition() >= 450) {
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
        while (dropperDC.getCurrentPosition() < 950) {
            dropperDC.setPower(1);
        }
        dropperDC.setPower(0);
        while (dropperServo.getPosition() > dUnload) {
            dPos = dropperServo.getPosition();
            dPos -= 0.03;
            dropperServo.setPosition(dPos);
        }
        if (dPos < dUnload) {
            dropperServo.setPosition(dUnload);
        }
    }

    public void GYROAXISLEFT (double targetHeading, double multiplicationFactor, double maxT) {

        mrgyro.resetZAxisIntegrator();

        double error, correction;

//        inRange = false;

        float gTime = 0;
        float gTimeStart = System.currentTimeMillis();

        while ((mrgyro.getIntegratedZValue() < (targetHeading-1)) || gTime < maxT) {
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

            motorFrontRight.setPower(correction);
            motorFrontLeft.setPower(-correction);
            motorBackLeft.setPower(-correction);
            motorBackRight.setPower(correction);

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

}
