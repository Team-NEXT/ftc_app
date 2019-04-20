package org.firstinspires.ftc.teamcode.TEST.AUTO;

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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;

import org.firstinspires.ftc.teamcode.TEST.TeleOp.Encoder;
import org.firstinspires.ftc.teamcode.TEST.TeleOp.MultithreadingTest;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Autonomous(name = "MT - Crater", group = "final")

public class multithreadedCrater extends LinearOpMode {

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

    private static double FO = 0.4, FC = 0.23;

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

//    public double yCount, xCount;

//    public boolean yReset, xReset;

    boolean detect = false;
    boolean detectionComplete = false;

    private static IntegratingGyroscope gyro;
    private static ModernRoboticsI2cGyro mrgyro;

    private GoldAlignDetector detector;

    private ElapsedTime timer = new ElapsedTime();

    private double startTime;

    private boolean loopComplete = false;

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
        cOpen = 0.92;
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
        collectorServo.setPosition(cInitial);
        xServo.setPosition(xUp);
        yServo.setPosition(yUp);
        flapServo.setPosition(FC);

        //GYRO
        mrgyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) mrgyro;

        while (!collectorLimit.isPressed()) {
            collectorDC.setPower(-0.6);
        }

        collectorDC.setPower(0);

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        mrgyro.calibrate();

//        xReset = true;
//        yReset = true;

        // Wait until the gyro calibration is complete
        timer.reset();
        while (!isStopRequested() && mrgyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        mineralPos = 1;

        Thread ultThread = new UltThread();
        Thread imgThread = new ImgThread();
//        Thread encoderThread = new EncoderThread();

        waitForStart();

        ultThread.start();
        imgThread.start();
//        encoderThread.start();

        /**CODE AFTER STARTING*/

        while (!loopComplete) {
            telemetry.addLine("AutoRunning");
            telemetry.update();
        }

        ultThread.interrupt();
        imgThread.interrupt();
//        encoderThread.interrupt();

//        COLLECTORCONTRACT(0.8);
    }

//    private class EncoderThread extends Thread {
//        public EncoderThread() {
//            this.setName("EncoderThread");
//        }
//        @Override
//        public void run() {
//            while (!isInterrupted()) {
//
//                yCount = yAxisDC.getCurrentPosition();
//
//                xCount = latchingDC.getCurrentPosition();
//
//                if (yReset) {
//                    yAxisDC.setMode(STOP_AND_RESET_ENCODER);
//                    yAxisDC.setMode(RUN_USING_ENCODER);
//                    yReset = false;
//                }
//
//                if (xReset) {
//                    latchingDC.setMode(STOP_AND_RESET_ENCODER);
//                    latchingDC.setMode(RUN_USING_ENCODER);
//                    xReset = false;
//                }
//
//                telemetry.addData("x: ", latchingDC.getCurrentPosition());
//                telemetry.addData("y: ", yAxisDC.getCurrentPosition());
//                telemetry.update();
//
//            }
//        }
//    }

    private class ImgThread extends Thread {
        public ImgThread() {
            this.setName("ImgThread");
        }
        @Override
        public void run() {
            while(!isInterrupted()) {
                if (detect && !detectionComplete) {
                    if (detector.isFound()) {
                        if (detector.getAligned()) {
                            mineralPos = 2;
                        } else {
                            mineralPos = 1;
                        }
                    } else {
                        mineralPos = 3;
                    }

                    detectionComplete = true;
                    detect = false;
                }
            }
        }
    }

    private class UltThread extends Thread {
        public UltThread() {
            this.setName("DriveThread");
        }
        @Override
        public void run() {
            while (!isInterrupted()) {
                        LATCHING(1);

                yServo.setPosition(yDown);
                xServo.setPosition(xDown);

                /**IMAGE RECOGNITION*/
                timer.reset();
                detect = true;
                while (!isStopRequested() && !detectionComplete)  {
                    telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
                    telemetry.update();
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e) {
                        telemetry.addLine("got interrupted");
                    }
                }

//                BACKWARD(50, 0.2);

                if (mineralPos == 1) {
                    SWAYRIGHT1(280);
                    AXISRIGHT(0.3, 29);
                    FORWARD(280, 0.4);
                    SWAYLEFT(100);
                    RAMPFORWARD(360, 0.2, 0.2, 0.6, 0.07, 0.14);
                    AXISLEFT(65, 0.4);
                    RAMPFORWARD(280, 0.2, 0.2, 0.6, 0.07, 0.14);
                    collectorDC.setPower(0);
                    collectorDC.setMode(STOP_AND_RESET_ENCODER);
                    collectorDC.setMode(RUN_USING_ENCODER);
                    while (collectorDC.getCurrentPosition() < 1500) {
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
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        telemetry.addLine("Got Interrupted");
                    }
                    sweeperServo.setPower(0);
                    while (collectorDC.getCurrentPosition() > 500) {
                        collectorDC.setPower(-1);
                    }
                    collectorDC.setPower(0);
                    while (collectorServo.getPosition() > cClose) {
                        cPos = collectorServo.getPosition();
                        cPos -= 0.07;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos < cClose) {
                        cPos = cClose;
                        collectorServo.setPosition(cPos);
                    }
                    while (!collectorLimit.isPressed()) {
                        collectorDC.setPower(-0.4);
                    }
                    collectorDC.setPower(0);
                    BACKWARD(200, 0.3);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        telemetry.addLine("Got Interrupted");
                    }
                    AXISLEFT(180, 0.36);
                }

                if (mineralPos == 2) {
                    AXISRIGHT(0.3, 90);
                }

                if (mineralPos == 3) {
                    SWAYRIGHT(280);
                    AXISLEFT(28, 0.3);
                    BACKWARD(160, 0.3);
                    BACKWARD(70, 0.2);
                    FORWARD(100, 0.4);
                    mrgyro.resetZAxisIntegrator();
                    GYRO_TANK(36, 0.2, 0);
                    RAMPFORWARD(660, 0.2, 0.2, 0.6, 0.07, 0.1);
                    AXISLEFT(30, 0.3);
                    AXISLEFT(13, 0.2);
                    RAMPFORWARD(400, 0.2, 0.2, 0.5, 0.1, 0.08);
                    collectorDC.setPower(0);
                    collectorDC.setMode(STOP_AND_RESET_ENCODER);
                    collectorDC.setMode(RUN_USING_ENCODER);
                    while (collectorDC.getCurrentPosition() < 1000) {
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
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        telemetry.addLine("Got Interrupted");
                    }
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
                        cPos = cClose;
                        collectorServo.setPosition(cPos);
                    }
                    AXISLEFT(179, 0.3);
                    tank(500, 0.3, 0.48);
                    FORWARD(250, 0.2);
                    collectorDC.setPower(0);
                    collectorDC.setMode(STOP_AND_RESET_ENCODER);
                    collectorDC.setMode(RUN_USING_ENCODER);
                    while (collectorDC.getCurrentPosition() < 800) {
                        collectorDC.setPower(0.8);
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
                    flapServo.setPosition(FC);
                    sweeperServo.setPower(1);
                    BACKWARD(60, 0.3);
                    FORWARD(170, 0.2);
                    AXISRIGHT(0.2, 10);
                    while (collectorServo.getPosition() < cMid) {
                        cPos = collectorServo.getPosition();
                        cPos += 0.03;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos > cMid) {
                        collectorServo.setPosition(cMid);
                    }
                    while (collectorDC.getCurrentPosition() > 500) {
                        collectorDC.setPower(-1);
                    }
                    while (!collectorLimit.isPressed()) {
                        collectorDC.setPower(-0.4);
                    }
                    collectorDC.setPower(0);
                    while (collectorDC.getCurrentPosition() < 240) {
                        collectorDC.setPower(0.2);
                    }
                    collectorDC.setPower(0);
                    sweeperServo.setPower(0);
                    while (collectorServo.getPosition() > cClose) {
                        cPos = collectorServo.getPosition();
                        cPos -= 0.07;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos < cClose) {
                        cPos = cClose;
                        collectorServo.setPosition(cPos);
                    }
                    flapServo.setPosition(FO);
                    sweeperServo.setPower(1);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        telemetry.addLine("Got Interrupted");
                    }
                    sweeperServo.setPower(0);
                    collectorDC.setMode(STOP_AND_RESET_ENCODER);
                    collectorDC.setMode(RUN_USING_ENCODER);
                    while (collectorDC.getCurrentPosition() < 500) {
                        collectorDC.setPower(1);
                    }
                    collectorDC.setPower(0);
                    while (collectorServo.getPosition() < cMid) {
                        cPos = collectorServo.getPosition();
                        cPos += 0.05;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos > cMid) {
                        collectorServo.setPosition(cMid);
                    }
                }
                loopComplete = true;
                telemetry.addLine("loopComplete = true");
                telemetry.update();
            }
        }
    }

    /**Methods*/
    public void FORWARD (int mmDistance, double power) {

        double targetTicks = mmDistance * singleTicks;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) > Math.abs(yAxisDC.getCurrentPosition())) {
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);

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

    public void RAMPFORWARD (int mmDistance, double startPower, double endPower, double maxPower, double increment, double decrement) {

        double targetTicks = mmDistance * singleTicks;
        double power = startPower;

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        yAxisDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks - 100) > Math.abs(yAxisDC.getCurrentPosition())) {
            if (yAxisDC.getCurrentPosition() <= (targetTicks/2)) {
                telemetry.addData("RUP", "");
                if (power > maxPower) {
                    power = maxPower;
                } else {
                    power += increment;
                }
            } else {
                telemetry.addData("RD", "");
                if (power < endPower) {
                    power = endPower;
                } else {
                    power -= decrement;
                }
            }
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);

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
            motorFrontRight.setPower(0.5);
            motorBackRight.setPower(-0.5);
            motorFrontLeft.setPower(-0.5);
            motorBackLeft.setPower(0.5);

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
            motorFrontRight.setPower(-0.4);
            motorBackRight.setPower(0.4);
            motorFrontLeft.setPower(0.4);
            motorBackLeft.setPower(-0.4);

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

    public void SWAYRIGHT1 (int mmDistance) {

        double targetTicks = mmDistance * singleTicks;

        latchingDC.setMode(STOP_AND_RESET_ENCODER);
        latchingDC.setMode(RUN_USING_ENCODER);

        while ((targetTicks-100) > Math.abs(latchingDC.getCurrentPosition())) {
            motorFrontRight.setPower(-0.5);
            motorBackRight.setPower(0.5);
            motorFrontLeft.setPower(0.4);
            motorBackLeft.setPower(-0.4);

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

    public void GYRO_TANK (double targetHeading, double leftPower, double rightPower) {
        while (Math.abs(mrgyro.getIntegratedZValue()) < (targetHeading-5)) {
            motorFrontRight.setPower(rightPower);
            motorBackRight.setPower(rightPower);
            motorFrontLeft.setPower(leftPower);
            motorBackLeft.setPower(leftPower);

            telemetry.addData("z: ", mrgyro.getIntegratedZValue());
            telemetry.update();
        }
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
    }

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

}
