package org.firstinspires.ftc.teamcode.TEST.ServoTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SERVO TEST : encoders", group = "test")

public class encoderServoTest extends LinearOpMode{

    private static Servo xServo;
    private static Servo yServo;

    private static double xPos = 0;
    private static double yPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        xServo = hardwareMap.servo.get("xServo");
        yServo = hardwareMap.servo.get("yServo");

        xPos = 0;
        yPos = 0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
//                Thread.sleep(750);
//                if (!gamepad1.x) {
//                    xPos += 0.01;
//                }
                xPos += 0.01;
            }

            if (gamepad1.a) {
//                 Thread.sleep(750);
//                if (!gamepad1.a) {
//                    xPos -= 0.01;
//                }
                xPos -= 0.01;
            }

            if (gamepad1.y) {
//                Thread.sleep(750);
//                if (!gamepad1.y) {
//                    yPos += 0.01;
//                }
                yPos += 0.01;
            }

            if (gamepad1.b) {
//                Thread.sleep(750);
//                if (!gamepad1.b) {
//                    yPos -= 0.01;
//                }
                yPos -= 0.01;
            }

            xServo.setPosition(xPos);
            yServo.setPosition(yPos);

            telemetry.addData("xServo: ", xServo.getPosition());
            telemetry.addData("yServo: ", yServo.getPosition());
            telemetry.update();

        }
    }
}

