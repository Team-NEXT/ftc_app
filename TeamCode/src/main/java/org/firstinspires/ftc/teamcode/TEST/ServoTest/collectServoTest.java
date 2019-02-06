package org.firstinspires.ftc.teamcode.TEST.ServoTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SERVO TEST : collector", group = "test")

public class collectServoTest extends LinearOpMode{

    private static Servo collectServo;

    private static double servoPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        collectServo = hardwareMap.servo.get("cServo");

        servoPos = 0.5;

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

            collectServo.setPosition(servoPos);

            telemetry.addData("collectServo: ", collectServo.getPosition());
            telemetry.update();

        }
    }
}

