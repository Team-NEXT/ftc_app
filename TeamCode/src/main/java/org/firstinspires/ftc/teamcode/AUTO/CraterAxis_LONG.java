package org.firstinspires.ftc.teamcode.AUTO;

import android.provider.Telephony;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//OpenCV
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Mat;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Disabled
@Autonomous(name = "80 point Crater with Axis Turns", group = "auto")

public class CraterAxis_LONG extends LinearOpMode{

    private GoldAlignDetector detector;

    //DRIVE
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

    //LATCHING
    private DcMotor latchingDC;
    private ModernRoboticsTouchSensor latchUpperLimit;
    private ModernRoboticsTouchSensor latchLowerLimit;

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

//    private double cSpeed;
//    private double cExtPos;

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
    private double xDown;
    private double yDown;

    private int mineralPos;

//    private static double x_encoder;
//    private static double y_encoder;

    private static double rampDownFactor = 0;
    private static double rampTicks = 0;

    private static double targetTicks = 0;
    private static final double finalTicks = 2400;
    private static final double singleTicks = 16;

    private static final double wheelDiameter = 48;

    private static final double degree = 42;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{

        //DRIVING
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");

        motorFrontRight.setDirection(REVERSE);
        motorBackRight.setDirection(REVERSE);

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
        latchUpperLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "latchingU");
        latchLowerLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "latchingL");
        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        //COLLECTOR
        collectorDC = hardwareMap.dcMotor.get("collectDC");
        sweeperDC = hardwareMap.dcMotor.get("sweeperDC");
        collectorServo = hardwareMap.servo.get("cServo");
        collectorServoLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "collectorExt");
        collectorExtLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "collectorServo");

        sweeperDC.setDirection(REVERSE);
        collectorDC.setDirection(REVERSE);

        sweeperDC.setMode(STOP_AND_RESET_ENCODER);
        sweeperDC.setMode(RUN_USING_ENCODER);

        collectorDC.setMode(STOP_AND_RESET_ENCODER);
        collectorDC.setMode(RUN_USING_ENCODER);

//        cSpeed = 0.07;

//        cExtPos = collectorDC.getCurrentPosition();
        cOpen = 0.92;
        cClose = 0.12;
        cMid = 0.6;

        //DROPPING
        dropperDC = hardwareMap.dcMotor.get("dropDC");
        dropperServo = hardwareMap.servo.get("dServo");
        dropperExtLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "dropper");

        dLoad = 0.66;
        dUnload = 0.29;

        //DRIVE ENCODERS
        xServo = hardwareMap.servo.get("xServo");
        yServo = hardwareMap.servo.get("yServo");

        xUp = 0.21;//0.15
        yUp = 0.38;
        xDown = 0.79;
        yDown = 0.13;


        //INITIALIZATION
        dropperServo.setPosition(dLoad);
        collectorServo.setPosition(cClose);
        xServo.setPosition(xUp);
        yServo.setPosition(xUp);

        /**DetectorINIT*/
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 220; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        //mineralPos = 2;

        /**START*/
        waitForStart();

        /*CODE AFTER STARTING*/

        LATCHING(1);

        yServo.setPosition(yDown);
        xServo.setPosition(xDown);

        BACKWARD(50, 0.2);

        Thread.sleep(100);

        /**CODE FOR IMAGE RECOGNITION*/

        //getaligned&isfound

        if (detector.isFound()) {
            if (detector.getAligned()) {
                mineralPos = 2;
                telemetry.addData("found and aligned", mineralPos);
                telemetry.addData("mineralPos: " , mineralPos);
                telemetry.update();
            }
            else {
                mineralPos = 3;
                telemetry.addData("found", mineralPos);
                telemetry.addData("mineralPos: " , mineralPos);
                telemetry.update();
            }
        }
        else {
            mineralPos = 1;
            telemetry.addData("none", mineralPos);
            telemetry.addData("mineralPos: " , mineralPos);
            telemetry.update();
        }

//        Thread.sleep(3000);

        if (mineralPos == 1) {

            SWAYRIGHT(220);

            Thread.sleep(100);

            xServo.setPosition(xUp);

            FORWARD(400, 0.3);

            Thread.sleep(100);

            AXISRIGHT(0.4, 90);

            Thread.sleep(100);

            FORWARD(280, 0.3);

            Thread.sleep(100);
//
//            BACKWARD(320, 0.3);
//
//            Thread.sleep(100);
//
//            AXISLEFT(90, 0.3);
//
//            Thread.sleep(100);
//
//            FORWARD(700, 0.3);

           //  CSERVODOWN(0.04);

             BACKWARD(210, 0.3);

             Thread.sleep(100);

             AXISLEFT(90,0.3);

             Thread.sleep(100);

             FORWARD(530, 0.3);

             Thread.sleep(100);

             AXISLEFT(44, 0.3);

             FORWARD(920, 0.3);

             CSERVOS(0.03);

             SWEEPER(-1);
             Thread.sleep(1500);
             SWEEPER(0);

             CSERVOUP(0.07);

             BACKWARD(1420 ,0.3);


        }

        if (mineralPos == 2) {

            SWAYRIGHT(200);

            Thread.sleep(100);

            xServo.setPosition(xUp);

            FORWARD(80, 0.3);

            Thread.sleep(100);

            AXISRIGHT(0.3, 90);

            Thread.sleep(100);

            FORWARD(210, 0.3);

//            Thread.sleep(100);
//
//            BACKWARD(350, 0.3);
//
//            Thread.sleep(100);
//
//            AXISLEFT(90, 0.3);
//
//            Thread.sleep(100);
//
//            FORWARD(950, 0.3);

           // CSERVODOWN(0.04);

             BACKWARD(200, 0.3);

             Thread.sleep(100);

             AXISLEFT(87, 0.2);

             Thread.sleep(100);

             FORWARD(920, 0.4);

             Thread.sleep(100);

             AXISLEFT(44, 0.3);

             Thread.sleep(100);

             FORWARD(920, 0.3);

             CSERVOS(0.03);

             SWEEPER(-1);
             Thread.sleep(1500);
             SWEEPER(0);

             CSERVOUP(0.07);

             c3lBACKWARD(1420, 0.4);

        }

        if (mineralPos == 3) {

            SWAYRIGHT(280); //distance = 90

            xServo.setPosition(xUp);

            Thread.sleep(100);

//            COASTBACKWARD(260, 0.7);23456u5432654362uu
            BACKWARD(170, 0.3);

            Thread.sleep(100);

            AXISRIGHT(0.3, 85);

            Thread.sleep(100);

            FORWARD(220, 0.3);

            Thread.sleep(100);
//
//            BACKWARD(250, 0.3);
//
//            Thread.sleep(100);
//
//            AXISLEFT(90, 0.3);
//
//            Thread.sleep(100);
//
//            FORWARD(1300, 0.3);

            // CSERVODOWN(0.04);

             BACKWARD(170, 0.3);

             AXISLEFT(91, 0.4);

             FORWARD(1160, 0.3);

             AXISLEFT(44, 0.2);

             FORWARD(980, 0.4);

             CSERVOS(0.03);

             SWEEPER(-1);
             Thread.sleep(1500);
             SWEEPER(0);

             CSERVOUP(0.07);

             AXISLEFT(5,0.2);

             c3lBACKWARD(1500, 0.4);

        }


//        COLLECTORCONTRACT(0.8);

    }

    /**Methods*/
    public void FORWARD (int mmDistance, double power) {

        targetTicks = mmDistance * singleTicks;

        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) > Math.abs(latchingDC.getCurrentPosition())) {
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(latchingDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

//    public void COASTFORWARD (int mmDistance, double power) {
//
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

        targetTicks = mmDistance * singleTicks;

        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(latchingDC.getCurrentPosition())) {
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(latchingDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void c3lBACKWARD (int mmDistance, double power) {

        targetTicks = mmDistance * singleTicks;

        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(latchingDC.getCurrentPosition())) {
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(latchingDC.getCurrentPosition()));
            telemetry.update();
        }

        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public void COASTBACKWARD (int mmDistance, double power) {

        targetTicks = mmDistance * singleTicks;

        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(latchingDC.getCurrentPosition())) {
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("y: ", Math.abs(latchingDC.getCurrentPosition()));
            telemetry.update();
        }

    }

    public void SWAYLEFT (int mmDistance) {

        targetTicks = mmDistance * singleTicks;

        sweeperDC.setMode(STOP_AND_RESET_ENCODER);
        sweeperDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(sweeperDC.getCurrentPosition())) {
            motorFrontRight.setPower(0.5);
            motorBackRight.setPower(-0.4);
            motorFrontLeft.setPower(-0.6);
            motorBackLeft.setPower(0.8);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("x: ", Math.abs(sweeperDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public void C2_SWAYLEFT (int mmDistance) {

        targetTicks = mmDistance * singleTicks;

        sweeperDC.setMode(STOP_AND_RESET_ENCODER);
        sweeperDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(sweeperDC.getCurrentPosition())) {
            motorFrontRight.setPower(0.5);
            motorBackRight.setPower(-0.4);
            motorFrontLeft.setPower(-0.6);
            motorBackLeft.setPower(0.8);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("x: ", Math.abs(sweeperDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

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

        targetTicks = mmDistance * singleTicks;

        sweeperDC.setMode(STOP_AND_RESET_ENCODER);
        sweeperDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(sweeperDC.getCurrentPosition())) {
            motorFrontRight.setPower(-0.6);
            motorBackRight.setPower(0.4);
            motorFrontLeft.setPower(0.8);
            motorBackLeft.setPower(-0.8);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("x: ", Math.abs(sweeperDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public void SWAYRIGHT2 (int mmDistance) {

        targetTicks = mmDistance * singleTicks;

        sweeperDC.setMode(STOP_AND_RESET_ENCODER);
        sweeperDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(sweeperDC.getCurrentPosition())) {
            motorFrontRight.setPower(-0.6);
            motorBackRight.setPower(0.4);
            motorFrontLeft.setPower(0.8);
            motorBackLeft.setPower(-0.8);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("x: ", Math.abs(sweeperDC.getCurrentPosition()));
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public void LATCHING (double power) {
        while (!latchUpperLimit.isPressed()) {
            latchingDC.setPower(power);
        }
        latchingDC.setPower(0);
    }

    public void AXISLEFT (double degrees, double power) {

        targetTicks = degrees * degree;

        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 60) > Math.abs(latchingDC.getCurrentPosition())) {
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
        telemetry.addData("y: ", Math.abs(latchingDC.getCurrentPosition()));
        telemetry.update();

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void AXISRIGHT (double power, double degrees) {

        targetTicks = degrees * degree;

        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) > Math.abs(latchingDC.getCurrentPosition())) {
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

        while (!collectorExtLimit.isPressed()) {
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

    public void CSERVOS (double power) { //0.02

        while (collectorServo.getPosition() < cOpen) {
            cPos = collectorServo.getPosition();
            cPos += power;
            collectorServo.setPosition(cPos);
        }
        if (cPos > cOpen) {
            cPos = cOpen;
            collectorServo.setPosition(cPos);
        }

    }

    public void SWEEPER (double power) {

        sweeperDC.setPower(power);

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

}
