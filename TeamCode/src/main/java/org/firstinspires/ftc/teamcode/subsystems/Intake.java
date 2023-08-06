package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.Math;

public class Intake {
    private static final double POWER = 1;
    public DcMotor motorGlisieraOriz;
   public DistanceSensor senzorDistanta;
    public Servo servobaza1;
    public Servo servobaza2;
    public Servo servocleste;

    public boolean mergeMotor;
    public double manualTarget = 0;
        //branch 2
    public double HDes = 0;

    boolean canextend=false;


    public double cmToTicks(double x) {
        return x * 34.24;
    }

    public Intake(HardwareMap hardwareMap){
        motorGlisieraOriz = hardwareMap.dcMotor.get("motorGlisieraOriz");
        servobaza1 = hardwareMap.servo.get("servoBaza1");
        servobaza2 = hardwareMap.servo.get("servoBaza2");
        servocleste = hardwareMap.servo.get("servoCleste");
        senzorDistanta = hardwareMap.get(DistanceSensor.class, "senzorDistanta");

        //Motor initialization
        motorGlisieraOriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisieraOriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisieraOriz.setDirection(DcMotorSimple.Direction.FORWARD);
    }




    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }

    public void desfaCleste() {
        servocleste.setPosition(0.3);
    }

    public void strangeCleste() {
        servocleste.setPosition(0.8);
    }

    public void setPower(double power){
        motorGlisieraOriz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisieraOriz.setPower(power);
    }

    public void setLevel(int target, double multiplier){
        motorGlisieraOriz.setTargetPosition(target);
        motorGlisieraOriz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisieraOriz.getCurrentPosition() > target) {
            motorGlisieraOriz.setPower(-POWER);
        }
        else {
            motorGlisieraOriz.setPower(POWER * multiplier);
        }
    }
    public void manualLevel(double manualTarget) {
        motorGlisieraOriz.setTargetPosition((int) manualTarget);
        motorGlisieraOriz.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motorGlisieraOriz.getCurrentPosition() > manualTarget )
        {
            motorGlisieraOriz.setPower(POWER);
        }
        else{
            motorGlisieraOriz.setPower(-POWER);
        }
    }

    public void autoExtend() {
        if (senzorDistanta.getDistance(DistanceUnit.CM) >= 3 && motorGlisieraOriz.getCurrentPosition() < cmToTicks(30)) {
                manualLevel(motorGlisieraOriz.getCurrentPosition() + 10);
        }

    }

    public boolean isGoing(int target){
        if (motorGlisieraOriz.getCurrentPosition() < target + 20 && motorGlisieraOriz.getCurrentPosition() > target - 20){
            return  false;
        }
        return true;

    }

    public void setServoBaza(double pos){
        servobaza1.setPosition(pos);
        servobaza2.setPosition(1-pos);
    }

    public void intakeToOuttake() {
//        try {
//            sleep(100);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
        setServoBaza(0.05);

    }

    public void outtakeToIntake() {
        setServoBaza(1);
    }

}


