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
public class LinearDriveModeV2 extends LinearOpMode {
    private Robot robot = null;
    public final static int ZERO = 5, MEDIUM = -700, TALL = -1250;
    public final static double DOWN_MULTIPLIER = 0.7;
    boolean outtakeEncodersDown = false, intakeEncodersDown = false;
    public boolean hasReached = true, isReadyForTransfer = false;

    public int hDess = 8;

    public int levelCon = 4;

    public double outtakeServoPosition, intakeServoPosition;
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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        pozitiiStack.add(new Pair<>(15, 18));
        pozitiiStack.add(new Pair<>(17, 18));
        pozitiiStack.add(new Pair<>(19, 18));
        pozitiiStack.add(new Pair<>(21, 18));
        pozitiiStack.add(new Pair<>(23, 17));

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();

        robot.intake.intakeToOuttake();
        robot.outtake.coboaraCupa();

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
                    robot.outtake.manualTarget = robot.outtake.motorGlisiera.getCurrentPosition() + calculateThrottle(gamepad1.right_trigger * 12);
                    robot.outtake.manualTarget++;
                    robot.outtake.manualLevel(robot.outtake.manualTarget);
                    //}
                }

                if (gamepad1.left_trigger > 0.1) {
                    // if (robot.outtake.getPosition() < TALL) {
                    robot.outtake.manualTarget = robot.outtake.motorGlisiera.getCurrentPosition() - calculateThrottle(gamepad1.left_trigger * 12);
                    robot.outtake.manualTarget--;
                    robot.outtake.manualLevel(robot.outtake.manualTarget);
                    //}
                }
            }

            if(gamepad1.dpad_up)
            {
                outtakeServoPosition = outtakeServoPosition + 0.05;
                robot.outtake.setCupa(outtakeServoPosition);
            }
            if(gamepad1.dpad_down)
            {
                outtakeServoPosition = outtakeServoPosition - 0.05;
                robot.outtake.setCupa(outtakeServoPosition);
            }


            //servoBaza1 0.7, servoBaza2 0.3




            /** INTAKE **/

            if(gamepad2.left_stick_button && gamepad2.right_stick_button){
                robot.intake.setPower(0);
                intakeEncodersDown = true;
            }

            if(intakeEncodersDown){
                if (gamepad2.left_trigger >= 0.1) {
                    robot.intake.setPower(gamepad2.left_trigger);
                } else if (gamepad2.right_trigger >= 0.1) {
                    robot.intake.setPower(-gamepad2.right_trigger);
                } else {
                    robot.intake.setPower(0);
                }
            }
            else{
                if (gamepad2.left_trigger > 0.1) {
                    //if (robot.outtake.getPosition() >= ZERO) {
                    robot.intake.manualTarget = robot.intake.motorGlisieraOriz.getCurrentPosition() + calculateThrottle(gamepad2.left_trigger * 12);
                    robot.intake.manualTarget++;
                    robot.intake.manualLevel(robot.intake.manualTarget);
                    //}
                }

                if (gamepad2.right_trigger > 0.1) {
                    // if (robot.outtake.getPosition() < TALL) {
                    robot.intake.manualTarget = robot.intake.motorGlisieraOriz.getCurrentPosition() - calculateThrottle(gamepad2.right_trigger * 12);
                    robot.intake.manualTarget--;
                    robot.intake.manualLevel(robot.intake.manualTarget);
                    //}
                }
            }

            if(gamepad2.touchpad){
                robot.intake.manualLevel(5);
            }

            if (gamepad2.circle) {
                robot.intake.intakeToOuttake();
                isReadyForTransfer = true;
            }

            if (gamepad2.right_bumper) {
                robot.intake.servoclestein.setPosition(0.7);
            }

            if(gamepad2.left_bumper){
                robot.intake.servoclestein.setPosition(0.3);
            }

            if (gamepad2.triangle) {
                robot.intake.distCon = pozitiiStack.get(levelCon).second;
                robot.intake.setHeight(pozitiiStack.get(levelCon).first);
                robot.intake.desfaCleste();
                isReadyForTransfer = false;
            }

            if(gamepad2.cross){
                hasReached = false;
            }

//            if(gamepad2.dpad_right){
//                robot.intake.servoclestein.setPosition(0.7);
//                sleep(200);
//                robot.intake.setHeight(28);
//                robot.intake.servobaza2.setPosition(0.6);
//                robot.intake.servobaza1.setPosition(1 - 0.6);
//                robot.intake.servoincheietura.setPosition(1);
//                sleep(300);
//                robot.intake.servoClesteRot.setPosition(0.4);
//
//                robot.intake.manualLevel(-700);
//                sleep(200);
//                isReadyForTransfer = true;
//            }

            if(!hasReached){
                if (robot.intake.senzorDistanta.getDistance(DistanceUnit.CM) >= 5) {
                    robot.intake.manualTarget = robot.intake.motorGlisieraOriz.getCurrentPosition() - calculateThrottle(12);
                    robot.intake.manualTarget--;
                    robot.intake.manualLevel(robot.intake.manualTarget);
                }else {
                    robot.intake.manualLevel(robot.intake.motorGlisieraOriz.getCurrentPosition());
                    hasReached = true;
                }
            }

            if(gamepad2.square){
                robot.intake.motorGlisieraOriz.setPower(0);
                hasReached = true;
            }

            if(gamepad2.dpad_left)
            {
                intakeServoPosition = intakeServoPosition + 0.05;
                robot.intake.servoClesteRot.setPosition(intakeServoPosition);
            }
            if(gamepad2.dpad_right)
            {
                intakeServoPosition = intakeServoPosition - 0.05;
                robot.intake.servoClesteRot.setPosition(intakeServoPosition);
            }




            if (gamepad2.dpad_up) {
                levelCon++;
                if (levelCon > 4) levelCon = 4;
                sleep(100);
            }
            if (gamepad2.dpad_down) {
                levelCon--;
                if (levelCon < 0) levelCon = 0;
                sleep(100);
            }

            //con5 - hdes - 21 cand apuca conu, 25 ca sa se ridice si distcon - 16,
            //con4 - hdes - 19 cand apuca conu, 25 ca sa se ridice si distcon - 16,
            //con3 - hdes - 16 cand apuca conu, 25 ca sa se ridice si distcon - 14,
            //con2 - hdes - 14 cand apuca conu, 25 ca sa se ridice si distcon - 13
            //con1 - hdes - 11 cand apuca conu, 25 ca sa se ridice si distcon - 17

            /** DRIVE **/

            robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)), calculateThrottle((float) (-gamepad1.left_stick_x)), calculateThrottle((float) (-gamepad1.right_stick_x))));

            /** TELEMETRY **/

            telemetry.addData("levelCon: ", levelCon + 1);
            telemetry.addData("?hasreached: ", hasReached);
            telemetry.addData("MOTORORIZTICKS: ", robot.intake.motorGlisieraOriz.getCurrentPosition());
            telemetry.addData("MOTORORIZTARGET: ", robot.intake.motorGlisieraOriz.getCurrentPosition());
            telemetry.addData("MOTORglisiera2 ", robot.outtake.motorGlisiera2.getCurrentPosition());
            telemetry.addData("MOTORglisiera1 ", robot.outtake.motorGlisiera.getCurrentPosition());
            telemetry.addData("MOTORglisiera1Targe ", robot.outtake.motorGlisiera2.getTargetPosition());
            telemetry.addData("getposition", robot.outtake.getPosition());
            telemetry.addData("servobazapos: ", robot.intake.servobaza1.getPosition());
            telemetry.addData("servoincheieturapos: ", robot.intake.servoClesteRot.getPosition());
            telemetry.addData("servorotpos: ", robot.intake.servoincheietura.getPosition());
            telemetry.addData("servoCupaPos:", robot.outtake.getServoPos());
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

