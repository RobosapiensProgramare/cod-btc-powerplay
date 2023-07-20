package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.Robot;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Odometrie;

@TeleOp(group = "driver")
public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    public final static int ZERO = 5, MEDIUM = -700, TALL = -1250;
    public final static double DOWN_MULTIPLIER = 0.7;
    boolean encodersDown = false;

    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            /** OUTTAKE **/

            double poz = robot.intake.motorGlisieraOriz.getCurrentPosition();
            //Aimbot
            if (gamepad1.touchpad) robot.outtake.setLevel(ZERO, DOWN_MULTIPLIER);

            if (gamepad1.square) robot.outtake.setLevel(MEDIUM, DOWN_MULTIPLIER);

            if (gamepad1.triangle) {
                robot.outtake.setLevel(TALL, DOWN_MULTIPLIER);
            }

            if(gamepad1.right_bumper) {
                robot.outtake.inchideBat();
                sleep(200);
                robot.outtake.ridicaCupa();
            }

            if(gamepad1.left_bumper){
                robot.outtake.deschideBat();
                sleep(700);
                robot.outtake.coboaraCupa();
                //robot.outtake.setLevel(ZERO, DOWN_MULTIPLIER);
            }
            //            if (gamepad1.square) robot.outtake.setLevel(TALL, DOWN_MULTIPLIER);

            //Daca mor encoderele de la glisiera
            if(gamepad1.left_stick_button && gamepad1.right_stick_button)
            {
                robot.outtake.setPower(0);//Run without encoder si power 0
                encodersDown = true;
            }

            if(encodersDown)
            {
                if(gamepad1.left_trigger >= 0.1)
                {
                    robot.outtake.setPower(gamepad2.left_trigger);
                }
                else if(gamepad1.right_trigger >= 0.1)
                {
                    robot.outtake.setPower(-gamepad1.right_trigger);
                }
                else
                {
                    robot.outtake.setPower(0);
                }
            }

            if (gamepad1.left_trigger > 0.1) {
                //if (robot.outtake.getPosition() >= ZERO) {
                    robot.outtake.manualTarget = robot.outtake.motorGlisiera.getCurrentPosition() + calculateThrottle(gamepad1.left_trigger * 12);
                    robot.outtake.manualTarget++;
                    robot.outtake.manualLevel(robot.outtake.manualTarget);
                //}
            }

            if (gamepad1.right_trigger > 0.1) {
               // if (robot.outtake.getPosition() < TALL) {
                    robot.outtake.manualTarget = robot.outtake.motorGlisiera.getCurrentPosition() - calculateThrottle(gamepad1.right_trigger * 12);
                    robot.outtake.manualTarget--;
                    robot.outtake.manualLevel(robot.outtake.manualTarget);
               // }
            }


            /** INTAKE **/

            if(gamepad2.square) {
                robot.intake.manualLevel(-835);
                robot.intake.servoincheietura.setPosition(0.75); //0.3
                sleep(300);
                robot.intake.servobaza2.setPosition(0);
                robot.intake.servobaza1.setPosition(1);
                sleep(300);
                robot.intake.servoclesterot.setPosition(0.4);
                robot.intake.servoclestein.setPosition(0.7);
                sleep(300);
                robot.outtake.coboaraCupa();
                robot.outtake.deschideBat();
            }

            if(gamepad2.circle) {
                robot.intake.manualLevel(-670);
                robot.intake.servobaza2.setPosition(0.6);
                robot.intake.servobaza1.setPosition(1-0.6);
                robot.intake.servoincheietura.setPosition(0.6);
                sleep(600);
                robot.intake.servoclesterot.setPosition(0.3);
            }
            if(gamepad2.triangle) {
                robot.intake.manualLevel(-100);
            }
            if(robot.intake.motorGlisieraOriz.getCurrentPosition() < 0 && robot.intake.motorGlisieraOriz.getCurrentPosition() > -1240 )
            {
                if (gamepad2.left_trigger > 0.1)
                {
                    robot.intake.manualTarget = robot.intake.motorGlisieraOriz.getCurrentPosition() + calculateThrottle(gamepad2.left_trigger * 12);
                    robot.intake.manualTarget--;
                    robot.intake.manualLevel(robot.intake.manualTarget);
                }

                if (gamepad2.right_trigger > 0.1)
                {
                    robot.intake.manualTarget = robot.intake.motorGlisieraOriz.getCurrentPosition() - calculateThrottle(gamepad2.right_trigger * 12);
                    robot.intake.manualTarget++;
                    robot.intake.manualLevel(robot.intake.manualTarget);
                }
            }

            if (gamepad2.right_bumper)
                robot.intake.servoclestein.setPosition(0.7);

            if (gamepad2.left_bumper)
                robot.intake.servoclestein.setPosition(0.3);


            //Drive
            robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)), calculateThrottle((float) (-gamepad1.left_stick_x)), calculateThrottle((float) (-gamepad1.right_stick_x))));
            telemetry.addData("MOTORORIZTICKS: ", robot.intake.motorGlisieraOriz.getCurrentPosition());
            telemetry.addData("MOTORORIZTARGET: ", robot.intake.motorGlisieraOriz.getCurrentPosition());
            telemetry.addData("MOTORglisiera2 ", robot.outtake.motorGlisiera2.getCurrentPosition());
            telemetry.addData("MOTORglisiera1 ", robot.outtake.motorGlisiera.getCurrentPosition());
            telemetry.addData("MOTORglisiera1Targe ", robot.outtake.motorGlisiera2.getTargetPosition());
            telemetry.addData("getposition", robot.outtake.getPosition());

            telemetry.update();
        }
    }
}
