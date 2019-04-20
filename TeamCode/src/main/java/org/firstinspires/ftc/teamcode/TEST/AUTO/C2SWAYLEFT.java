package org.firstinspires.ftc.teamcode.TEST.AUTO;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.opencv.core.Mat;
import org.opencv.video.BackgroundSubtractor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Autonomous(name = "SWAYLEFT C2", group = "sway")

public class C2SWAYLEFT extends LinearOpMode{

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
        cMid = 0.8;

        //DROPPING
        dropperDC = hardwareMap.dcMotor.get("dropDC");
        dropperServo = hardwareMap.servo.get("dServo");
        dropperExtLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "dropper");

        dLoad = 0.64;
        dUnload = 0.3;

        //DRIVE ENCODERS
        xServo = hardwareMap.servo.get("xServo");
        yServo = hardwareMap.servo.get("yServo");

        xUp = 0.12;
        yUp = 0.38;
        xDown = 0.73;
        yDown = 0.07;


        //INITIALIZATION
        dropperServo.setPosition(dLoad);
        collectorServo.setPosition(cClose);
        xServo.setPosition(xUp);
        yServo.setPosition(yUp);

        /**DetectorINIT*/
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

//        mineralPos = 1;

        /**START*/
        waitForStart();

        /*CODE AFTER STARTING*/

        C2_SWAYLEFT(400);

    }

    /**Methods*/

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

}
