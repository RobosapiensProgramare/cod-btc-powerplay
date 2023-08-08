package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.opmode.AutonomousBeclean.DOWN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.drive.opmode.AutonomousBeclean.MEDIUM;
import static org.firstinspires.ftc.teamcode.drive.opmode.AutonomousBeclean.TALL;
import static org.firstinspires.ftc.teamcode.drive.opmode.AutonomousBeclean.ZERO;
import static java.lang.Thread.sleep;

public class RunnableTask implements Runnable{
    private String type;
    private int action;
    private Thread t;
    private Intake intake;
    private Outtake outtake;

    private int pos;

    RunnableTask(int action, Intake intake, Outtake outtake, String type, int pos) {
        this.action = action;
        this.intake = intake;
        this.outtake = outtake;
        this.type = type;
        this.pos = pos;
    }

    public boolean isAlive() {
        return t.isAlive();
    }

    public void raiseToMedium() {
        try {
            outtake.inchideBat();
            sleep(150);
            outtake.deschideBat();
            sleep(150);
            outtake.setLevel(MEDIUM, DOWN_MULTIPLIER);
            sleep(150);
            outtake.inchideBat();
            sleep(150);
            outtake.deschideBat();
            sleep(150);
            outtake.inchideBat();
            sleep(200);
            outtake.ridicaCupa();
        } catch (InterruptedException e){

        }
    }

    public void raiseToHigh() {
        try {
            outtake.inchideBat();
            sleep(150);
            outtake.deschideBat();
            sleep(150);
            outtake.setLevel(TALL, DOWN_MULTIPLIER);
            sleep(150);
            outtake.inchideBat();
            sleep(150);
            outtake.deschideBat();
            sleep(150);
            outtake.inchideBat();
            sleep(200);
            outtake.ridicaCupa();
        } catch (InterruptedException e){

        }
    }

    public void releaseCone()
    {
        try {
            outtake.deschideBat();
            sleep(250);
            outtake.coboaraCupa();
            sleep(200);
            outtake.setLevel(ZERO, DOWN_MULTIPLIER);
        } catch (InterruptedException e) {

        }
    }
    public void grabCone(int pos) {
        switch (pos) {
            case 0:
                intake.setServoBaza(1);
                break;
            case 1:
                intake.setServoBaza(0.96);
                break;
            case 2:
                intake.setServoBaza(0.92);
                break;
            case 3:
                intake.setServoBaza(0.88);
                break;
            case 4:
                intake.setServoBaza(0.84);
                break;
        }
    }

    public void intakeToOuttake() {
        try {
            intake.strangeCleste();
            sleep(200);
            intake.intakeToOuttake();
            outtake.coboaraCupa();
            intake.manualLevel(-500);
            sleep(1030);
            intake.desfaCleste();
            sleep(800);
            intake.setServoBaza(0.4);
            intake.manualLevel(0);
        } catch (InterruptedException e) {

        }
    }

    public void run() {
        if (type == "horizontal") {
            switch (action) {
                case 1:
                    grabCone(pos);
                    break;
                case 2:
                    intakeToOuttake();
                    break;
                default:
                    return;
            }
        }

        if (type == "vertical") {
            switch (action) {
                case 1:
                    raiseToHigh();
                    break;
                case 2:
                    raiseToMedium();
                    break;
                case 3:
                    releaseCone();
                    break;
            }
        }
    }

    public void start() {
        if (t == null) {
            t = new Thread(this, "thread"+action);
            t.start();
        }
    }
}
