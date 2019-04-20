package org.firstinspires.ftc.teamcode.TEST.ServoTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SERVO TEST : sweeper", group = "test")

public class sweepServoTest extends LinearOpMode{

    private static CRServo sweeperServo;
    private static Servo flapServo;

    private static double servoPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        sweeperServo = hardwareMap.crservo.get("sweepServo");
        flapServo = hardwareMap.servo.get("flapServo");

        flapServo.setPosition(0.2);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
//                Thread.sleep(750);
//                if (!gamepad1.x) {
//                    xPos += 0.01;
//                }
                servoPos += 0.01;
            }

            if (gamepad1.a) {
//                 Thread.sleep(750);
//                if (!gamepad1.a) {
//                    xPos -= 0.01;
//                }
                servoPos -= 0.01;
            }

            sweeperServo.setPower(servoPos);

            telemetry.addData("val: ", sweeperServo.getPower());
            telemetry.update();

        }
    }
}

