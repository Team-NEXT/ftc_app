/**2400 ticks = 1 encoder rotation*/

package org.firstinspires.ftc.teamcode.TEST.AUTO;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Autonomous(name = "MR GYRO TEST", group = "test")

public class MRGYROTEST extends LinearOpMode{

    private static DcMotor motorFrontRight;
    private static DcMotor motorBackRight;
    private static DcMotor motorFrontLeft;
    private static DcMotor motorBackLeft;

    private static IntegratingGyroscope gyro;
    private static ModernRoboticsI2cGyro mrgyro;

    private ElapsedTime timer = new ElapsedTime();

    private double targetHeading;

    private boolean inRange;

    @Override
    public void runOpMode() throws InterruptedException {

        //DRIVE
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");

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

        waitForStart();

        /**AUTO CODE*/

        AXISLEFT90(90, 0.5);

    }

    /**METHODS*/

    public void AXISLEFT90 (double targetAngle, double power) {

        mrgyro.resetZAxisIntegrator();

//        inRange = false;

        while (mrgyro.getIntegratedZValue() < 90) {
                motorFrontRight.setPower(power);
                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(-power);
                motorBackRight.setPower(power);

            telemetry.addData("FR", motorFrontRight.getPower());
            telemetry.addData("BR", motorBackRight.getPower());
            telemetry.addData("FL", motorFrontLeft.getPower());
            telemetry.addData("BL", motorBackLeft.getPower());
            telemetry.addData("z: ", mrgyro.getIntegratedZValue());
            telemetry.update();
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

}