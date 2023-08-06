package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Robot;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Odometrie;

@TeleOp(group = "driver")
public class LinearDriveModeV2 extends LinearOpMode {
    private Robot robot = null;
    public final static int ZERO = 5, MEDIUM = 700, TALL = 1250;
    public final static double DOWN_MULTIPLIER = 0.7;
    boolean outtakeEncodersDown = false, intakeEncodersDown = false;
    public int levelCon = 4;
    boolean upPress = false, downPress = false;
    public double outtakeServoPosition, intakeServoPosition;

    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }

    public static RunnableTask horizontal;
    public static RunnableTask vertical;

    public boolean checkExtend()
    {
        if (robot.intake.senzorDistanta.getDistance(DistanceUnit.CM) <= 30)
            return true;
        return false;
    }

    public boolean checkReached() {
        if (robot.intake.senzorDistanta.getDistance(DistanceUnit.CM) <= 5)
            return true;
        return false;
    }

//    public void extend(int dist) {
//        if (!checkReached() && dist<100) {
//            robot.intake.manualLevel(robot.intake.motorGlisieraOriz.getCurrentPosition() - 30);
//            hasReached = false;
//            extend(dist+10);
//        }
//        else
//            hasReached = true;
//    }

    public void faza1() {
        switch (levelCon) {
            case 0:
                robot.intake.setServoBaza(1);
                break;
            case 1:
                robot.intake.setServoBaza(0.96);
                break;
            case 2:
                robot.intake.setServoBaza(0.92);
                break;
            case 3:
                robot.intake.setServoBaza(0.88);
                break;
            case 4:
                robot.intake.setServoBaza(0.84);
                break;
        }

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

        robot.intake.setServoBaza(0.4);
        sleep(500);
        robot.outtake.coboaraCupa();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            /** OUTTAKE **/

            double poz = robot.intake.motorGlisieraOriz.getCurrentPosition();
            //Aimbot
            if (gamepad1.touchpad) {
                robot.outtake.setLevel(ZERO, DOWN_MULTIPLIER);
            }

            //TODO copy paste de la triangle
            if (gamepad1.square) {
//                robot.outtake.inchideBat();
//                sleep(200);
//                robot.outtake.setServoCupa(0.45);
//                sleep(200);
//                robot.outtake.setLevel(MEDIUM, DOWN_MULTIPLIER);
//                sleep(1000);
//                robot.outtake.ridicaCupa();
//                sleep(200);
//                robot.outtake.deschideBat();
//                sleep(250);
//                robot.outtake.coboaraCupa();
//                sleep(200);
//                robot.outtake.setLevel(ZERO, DOWN_MULTIPLIER);
                if (vertical == null || !vertical.isAlive()) {
                    vertical = new RunnableTask(2, robot.intake, robot.outtake, "vertical");
                    vertical.start();
                }
            }

            if (gamepad1.triangle) {
//                robot.outtake.setLevel(TALL, DOWN_MULTIPLIER);
//                sleep(200);
//                robot.outtake.inchideBat();
//                sleep(200);
//                robot.outtake.ridicaCupa();
//                sleep(850);
//                robot.outtake.deschideBat();
//                sleep(250);
//                robot.outtake.coboaraCupa();
//                sleep(200);
//                robot.outtake.setLevel(ZERO, DOWN_MULTIPLIER);
                if (vertical == null || !vertical.isAlive()) {
                    vertical = new RunnableTask(1, robot.intake, robot.outtake, "vertical");
                    vertical.start();
                }
            }

            if (gamepad1.right_bumper) {
                robot.outtake.inchideBat();
                sleep(200);
                robot.outtake.ridicaCupa();
            }

            if (gamepad1.left_bumper) {
//                robot.outtake.deschideBat();
//                sleep(250);
//                robot.outtake.coboaraCupa();
                if (vertical == null || !vertical.isAlive()) {
                    vertical = new RunnableTask(3, robot.intake, robot.outtake, "vertical");
                    vertical.start();
                }
            }

            if (gamepad1.dpad_right)
                robot.outtake.inchideBat();

            if (gamepad1.dpad_left)
                robot.outtake.deschideBat();


            //Daca mor encoderele de la glisiera
            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                robot.outtake.setPower(0);//Run without encoder si power 0
                outtakeEncodersDown = true;
            }

            if (outtakeEncodersDown)
            { /** DACA CRAPA ENCODERELE **/
                if (gamepad1.left_trigger >= 0.1) {
                    robot.outtake.setPower(gamepad1.left_trigger);
                } else if (gamepad1.right_trigger >= 0.1) {
                    robot.outtake.setPower(-gamepad1.right_trigger);
                } else {
                    robot.outtake.setPower(0);
                }
            }
            else
            {/** DACA MERG ENCODERELE **/
                if (gamepad1.right_trigger > 0.1) {
                    //if (robot.outtake.getPosition() >= ZERO) {
                    robot.outtake.manualTarget = robot.outtake.motorGlisiera1.getCurrentPosition() + calculateThrottle(gamepad1.right_trigger * 12);
                    robot.outtake.manualTarget++;
                    if(robot.outtake.manualTarget > TALL) {
                        robot.outtake.manualTarget = TALL;
                    }
                    robot.outtake.manualLevel(robot.outtake.manualTarget);
                }

                if (gamepad1.left_trigger > 0.1) {
                    // if (robot.outtake.getPosition() < TALL) {
                    robot.outtake.manualTarget = robot.outtake.motorGlisiera1.getCurrentPosition() - calculateThrottle(gamepad1.left_trigger * 12);
                    robot.outtake.manualTarget--;
                    if(robot.outtake.manualTarget < ZERO){
                        robot.outtake.manualTarget = ZERO;
                    }
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





            /** INTAKE **/

            if(gamepad2.left_stick_button && gamepad2.right_stick_button) {
                robot.intake.setPower(0);
                intakeEncodersDown = true;
            }

            if(intakeEncodersDown)
            {
                if (gamepad2.left_trigger >= 0.1) {
                    robot.intake.setPower(gamepad2.left_trigger);
                }
                else if (gamepad2.right_trigger >= 0.1)
                {
                    robot.intake.setPower(-gamepad2.right_trigger);
                }
                else
                {
                    robot.intake.setPower(0);
                }
            }
            else
            {
                if (gamepad2.left_trigger > 0.1) {
                    //if (robot.outtake.getPosition() >= ZERO) {
                    robot.intake.manualTarget = robot.intake.motorGlisieraOriz.getCurrentPosition() + calculateThrottle(gamepad2.left_trigger * 12);
                    robot.intake.manualTarget++;
                    if(robot.intake.manualTarget > 0){
                        robot.intake.manualTarget = 0;
                    }
                    robot.intake.manualLevel(robot.intake.manualTarget);
                    //}
                }

                if (!checkReached()) {
                    if (gamepad2.right_trigger > 0.1) {
                        // if (robot.outtake.getPosition() < TALL) {
                        robot.intake.manualTarget = robot.intake.motorGlisieraOriz.getCurrentPosition() - calculateThrottle(gamepad2.right_trigger * 12);
                        robot.intake.manualTarget--;
                        if (robot.intake.manualTarget < -800) {
                            robot.intake.manualTarget = -800;
                        }
                        robot.intake.manualLevel(robot.intake.manualTarget);
                        //}
                    }
                }
            }


            if(gamepad2.touchpad)
            {
                robot.intake.manualLevel(5);
            }


            if(gamepad2.dpad_up && upPress == false && levelCon < 4)
            {
                levelCon++;
                upPress =true;
            }
            if(!gamepad2.dpad_up) upPress = false;

            if(gamepad2.dpad_down && downPress ==false && levelCon > 0)
            {
                levelCon--;
                downPress =true;
            }
            if(!gamepad2.dpad_down) downPress = false;

            if(gamepad2.dpad_right)
            {
                intakeServoPosition = intakeServoPosition + 0.05;
                robot.intake.setServoBaza(intakeServoPosition);
            }

            if(gamepad2.dpad_left)
            {
                intakeServoPosition = intakeServoPosition - 0.05;
                robot.intake.setServoBaza(intakeServoPosition);
            }



//            if(gamepad2.cross) faza1();
            if(gamepad2.cross) {
                if (horizontal == null || !horizontal.isAlive()) {
                    horizontal = new RunnableTask(1, robot.intake, robot.outtake, "horizontal");
                    horizontal.start();
                }
            }


            if (gamepad2.circle) {
//                robot.intake.strangeCleste();
//                sleep(200);
//                robot.intake.intakeToOuttake();
//                robot.outtake.coboaraCupa();
//                robot.intake.manualLevel(-500);
//                sleep(1030);
//                robot.intake.desfaCleste();
//                sleep(800);
//                robot.intake.setServoBaza(0.4);
//                robot.intake.manualLevel(0);
                if (horizontal == null || !horizontal.isAlive()) {
                    horizontal = new RunnableTask(2, robot.intake, robot.outtake, "horizontal");
                    horizontal.start();
                }
            }




            if(gamepad2.square){
                robot.intake.motorGlisieraOriz.setPower(0);
            }





            if (gamepad2.right_bumper) {
                robot.intake.strangeCleste();
            }

            if(gamepad2.left_bumper){
                robot.intake.desfaCleste();
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

//            if(!hasReached){
//                if (robot.intake.senzorDistanta.getDistance(DistanceUnit.CM) >= 5) {
//                    robot.intake.manualTarget = robot.intake.motorGlisieraOriz.getCurrentPosition() - calculateThrottle(12);
//                    robot.intake.manualTarget--;
//                    if(robot.intake.manualTarget < -800){
//                        robot.intake.manualTarget = -800;
//                    }
//                    robot.intake.manualLevel(robot.intake.manualTarget);
//                }else {
//                    robot.intake.manualLevel(robot.intake.motorGlisieraOriz.getCurrentPosition());
//                    hasReached = true;
//                }
//            }




            //con5 - hdes - 21 cand apuca conu, 25 ca sa se ridice si distcon - 16,
            //con4 - hdes - 19 cand apuca conu, 25 ca sa se ridice si distcon - 16,
            //con3 - hdes - 16 cand apuca conu, 25 ca sa se ridice si distcon - 14,
            //con2 - hdes - 14 cand apuca conu, 25 ca sa se ridice si distcon - 13
            //con1 - hdes - 11 cand apuca conu, 25 ca sa se ridice si distcon - 17

            /** DRIVE **/

            robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)), calculateThrottle((float) (-gamepad1.left_stick_x)), calculateThrottle((float) (-gamepad1.right_stick_x))));

            /** TELEMETRY **/

            telemetry.addData("levelCon: ", levelCon + 1);
            telemetry.addData("MOTORORIZTICKS: ", robot.intake.motorGlisieraOriz.getCurrentPosition());
            telemetry.addData("MOTORORIZTARGET: ", robot.intake.motorGlisieraOriz.getCurrentPosition());
            telemetry.addData("MOTORglisiera2 ", robot.outtake.motorGlisiera2.getCurrentPosition());
            telemetry.addData("MOTORglisiera1 ", robot.outtake.motorGlisiera1.getCurrentPosition());
            telemetry.addData("MOTORglisiera1Targe ", robot.outtake.motorGlisiera2.getTargetPosition());
            telemetry.addData("getposition", robot.outtake.getPosition());
            telemetry.addData("servobazapos: ", robot.intake.servobaza1.getPosition());
            telemetry.addData("servoCupaPos:", robot.outtake.getServoPos());
            telemetry.addData("senzordistanta:  ", robot.intake.senzorDistanta.getDistance(DistanceUnit.CM));
//            telemetry.addData("senzorDistantaCM: ", robot.intake.senzorDistanta.getDistance(DistanceUnit.CM));

            int ceva = 1;
            ceva++;
            telemetry.addData("ceva:", ceva);

            telemetry.update();
        }
    }
}

