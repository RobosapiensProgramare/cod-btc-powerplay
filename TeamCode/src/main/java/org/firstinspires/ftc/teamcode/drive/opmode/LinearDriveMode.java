package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Robot;

import java.util.ArrayList;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Odometrie;

@TeleOp(group = "driver")
public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    public final static int ZERO = 5, MEDIUM = -700, TALL = -1250;
    public final static double DOWN_MULTIPLIER = 0.7;
    boolean outtakeEncodersDown = false;
    public boolean hasReached = true, isReadyForTransfer = false;

    public int hDess = 8;

    public int levelCon = 4;

    public int level = 0;
    ArrayList<Pair<Integer, Integer>> pozitiiStack = new ArrayList<>(5);

    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }

    public void setServosBaza(double pos) {
        robot.intake.servobaza2.setPosition(pos);
        robot.intake.servobaza1.setPosition(1 - pos);
    }

    public void extindeLevel0() {
        robot.intake.servoincheietura.setPosition(0.75); //0.3
        sleep(300);
        setServosBaza(0);
        sleep(300);
        robot.intake.servoClesteRot.setPosition(0.4);
//        robot.intake.servoclestein.setPosition(0.7);
    }

    public void extindeLevel1() {
        robot.intake.servoincheietura.setPosition(0.67); //0.3
        sleep(300);
        setServosBaza(0.09);
        sleep(300);
        robot.intake.servoClesteRot.setPosition(0.32);
//        robot.intake.servoclestein.setPosition(0.7);
    }

    public void extindeLevel2() {
        robot.intake.servoincheietura.setPosition(0.63); //0.3
        sleep(300);
        setServosBaza(0.1);
        sleep(300);
        robot.intake.servoClesteRot.setPosition(0.25);
//        robot.intake.servoclestein.setPosition(0.7);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        pozitiiStack.add(new Pair<>(11, 17));
        pozitiiStack.add(new Pair<>(14, 13));
        pozitiiStack.add(new Pair<>(16, 14));
        pozitiiStack.add(new Pair<>(19, 16));
        pozitiiStack.add(new Pair<>(22, 16));

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
            if (gamepad1.touchpad) {
                robot.outtake.setLevel(ZERO, DOWN_MULTIPLIER);
            }

            if (gamepad1.square) {
                robot.outtake.setLevel(MEDIUM, DOWN_MULTIPLIER);
            }

            if (gamepad1.triangle) {
                robot.outtake.setLevel(TALL, DOWN_MULTIPLIER);
            }

            if (gamepad1.right_bumper) {
                robot.outtake.inchideBat();
                sleep(200);
                robot.outtake.ridicaCupa();
            }

            if (gamepad1.left_bumper) {
                robot.outtake.deschideBat();
                sleep(250);
                robot.outtake.coboaraCupa();
                //robot.outtake.setLevel(ZERO, DOWN_MULTIPLIER);
            }
            //            if (gamepad1.square) robot.outtake.setLevel(TALL, DOWN_MULTIPLIER);

            //Daca mor encoderele de la glisiera
            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                robot.outtake.setPower(0);//Run without encoder si power 0
                outtakeEncodersDown = true;
            }

            if (outtakeEncodersDown) { /** DACA CRAPA ENCODERELE **/
                if (gamepad1.left_trigger >= 0.1) {
                    robot.outtake.setPower(gamepad1.left_trigger);
                } else if (gamepad1.right_trigger >= 0.1) {
                    robot.outtake.setPower(-gamepad1.right_trigger);
                } else {
                    robot.outtake.setPower(0);
                }
            } else {/** DACA MERG ENCODERELE **/
                if (gamepad1.right_trigger > 0.1) {
                    //if (robot.outtake.getPosition() >= ZERO) {
                    robot.outtake.manualTarget = robot.outtake.motorGlisiera1.getCurrentPosition() + calculateThrottle(gamepad1.right_trigger * 12);
                    robot.outtake.manualTarget++;
                    robot.outtake.manualLevel(robot.outtake.manualTarget);
                    //}
                }

                if (gamepad1.left_trigger > 0.1) {
                    // if (robot.outtake.getPosition() < TALL) {
                    robot.outtake.manualTarget = robot.outtake.motorGlisiera1.getCurrentPosition() - calculateThrottle(gamepad1.left_trigger * 12);
                    robot.outtake.manualTarget--;
                    robot.outtake.manualLevel(robot.outtake.manualTarget);
                    //}
                }
            }

            /** INTAKE **/

//            if(gamepad2.square) {
//                robot.intake.manualLevel(-835);
//
//                sleep(300);
//                robot.outtake.coboaraCupa();
//                robot.outtake.deschideBat();
//
//            }

            /** asta e gen sa misti glisierele manual **/
            if (robot.intake.motorGlisieraOriz.getCurrentPosition() < 0 && robot.intake.motorGlisieraOriz.getCurrentPosition() > -1240) {
                if (gamepad2.left_trigger > 0.1) {
                    robot.intake.manualTarget = robot.intake.motorGlisieraOriz.getCurrentPosition() + calculateThrottle(gamepad2.left_trigger * 12);
                    robot.intake.manualTarget--;
                    robot.intake.manualLevel(robot.intake.manualTarget);
                }

                if (gamepad2.right_trigger > 0.1) {
                    robot.intake.manualTarget = robot.intake.motorGlisieraOriz.getCurrentPosition() - calculateThrottle(gamepad2.right_trigger * 12);
                    robot.intake.manualTarget++;
                    robot.intake.manualLevel(robot.intake.manualTarget);
                }
            }


            if (gamepad2.circle) {
                sleep(100);
                robot.intake.servobaza2.setPosition(0.6);
                robot.intake.servobaza1.setPosition(1 - 0.6);
                robot.intake.servoincheietura.setPosition(1);
                sleep(600);
                robot.intake.servoClesteRot.setPosition(0.4);

                robot.intake.manualLevel(-700);
                sleep(500);
                isReadyForTransfer = true;
            }

            if (gamepad2.right_bumper) {
                robot.intake.servoclesteIn.setPosition(0.7);
                sleep(500);
                robot.intake.setHeight(28);
            }
            if (isReadyForTransfer) {
                if (gamepad2.left_bumper) {
                    robot.intake.servoincheietura.setPosition(0.4);
                    sleep(600);
                    robot.intake.servoclesteIn.setPosition(0.3);
                    robot.intake.manualLevel(-800);
                }
            } else {
                if (gamepad2.left_bumper)
                    robot.intake.servoclesteIn.setPosition(0.3);
            }
            if (gamepad2.triangle) {
                robot.intake.distCon = pozitiiStack.get(levelCon).second;
                robot.intake.setHeight(pozitiiStack.get(levelCon).first);
                robot.intake.desfaCleste();
                isReadyForTransfer = false;
            }
            if (gamepad2.cross) {
                hasReached = false;
            }

            if (!hasReached) {
                if (robot.intake.senzorDistanta.getDistance(DistanceUnit.CM) >= 5) {
                    robot.intake.motorGlisieraOriz.setPower(-0.75);
                    hasReached = false;
                } else if (robot.intake.senzorDistanta.getDistance(DistanceUnit.CM) < 5) {
                    robot.intake.motorGlisieraOriz.setPower(0);
                    hasReached = true;
                }
                if (gamepad2.left_trigger == 1-0.2 && gamepad2.right_trigger == 1 - 0.2) {
                    robot.intake.motorGlisieraOriz.setPower(0);
//                    gamepad2.left_stick_button = false;
//                    gamepad2.right_stick_button = false;
                    hasReached = true;
                }
            }





            if (levelCon > 4) levelCon = 4;
            if (levelCon < 0) levelCon = 0;

            if (gamepad2.dpad_up) {
                levelCon++;
                sleep(100);
            }
            if (gamepad2.dpad_down) {
                levelCon--;
                sleep(100);
            }

            //con5 - hdes - 21 cand apuca conu, 25 ca sa se ridice si distcon - 16,
            //con4 - hdes - 19 cand apuca conu, 25 ca sa se ridice si distcon - 16,
            //con3 - hdes - 16 cand apuca conu, 25 ca sa se ridice si distcon - 14,
            //con2 - hdes - 14 cand apuca conu, 25 ca sa se ridice si distcon - 13
            //con1 - hdes - 11 cand apuca conu, 25 ca sa se ridice si distcon - 17

//            switch (levelCon){
//                case 5: {
//                    hDess = 21;
//                    robot.intake.DistCon = 16;
//                    break;
//                }
//                case 4: {
//                    hDess = 19;
//                    robot.intake.DistCon = 16;
//                    break;
//                }
//                case 3: {
//                    hDess = 16;
//                    robot.intake.DistCon = 14;
//                    break;
//                }
//                case 2: {
//                    hDess = 14;
//                    robot.intake.DistCon = 13;
//                    break;
//                }
//                case 1: {
//                    hDess = 11;
//                    robot.intake.DistCon = 17;
//                    break;
//                }
//            }

            //Drive
            robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)), calculateThrottle((float) (-gamepad1.left_stick_x)), calculateThrottle((float) (-gamepad1.right_stick_x))));
            telemetry.addData("levelCon: ", levelCon + 1);
            telemetry.addData("?hasreached: ", hasReached);
            telemetry.addData("MOTORORIZTICKS: ", robot.intake.motorGlisieraOriz.getCurrentPosition());
            telemetry.addData("MOTORORIZTARGET: ", robot.intake.motorGlisieraOriz.getCurrentPosition());
            telemetry.addData("MOTORglisiera2 ", robot.outtake.motorGlisiera2.getCurrentPosition());
            telemetry.addData("MOTORglisiera1 ", robot.outtake.motorGlisiera1.getCurrentPosition());
            telemetry.addData("MOTORglisiera1Targe ", robot.outtake.motorGlisiera2.getTargetPosition());
            telemetry.addData("getposition", robot.outtake.getPosition());
            telemetry.addData("servobazapos: ", robot.intake.servobaza1.getPosition());
            telemetry.addData("servoincheieturapos: ", robot.intake.servoClesteRot.getPosition());
            telemetry.addData("servorotpos: ", robot.intake.servoincheietura.getPosition());
            telemetry.addData("Hdess: ", hDess);
            telemetry.addData("FirstAng: ", robot.intake.FirstAng);
            telemetry.addData("MidAng:  ", robot.intake.MidAng);
            telemetry.addData("FinalAng: ", robot.intake.FinalAng);
            telemetry.addData("DistCon: ", robot.intake.distCon);
            telemetry.addData("senzorDistantaCM: ", robot.intake.senzorDistanta.getDistance(DistanceUnit.CM));

            int ceva = 1;
            ceva++;
            telemetry.addData("ceva:", ceva);

            telemetry.update();
        }
    }
}

