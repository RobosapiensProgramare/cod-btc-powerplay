package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.Math;

public class Intake {

    public double Lseg1 = 14.3, Lseg2 = 12.7, Hpivot1 = 20, Lcleste = 6.5, distCon = 10, L1, MidAng, FirstAng, FinalAng, A, B, D, Marcel;
    private static final double POWER = 1;
    public DcMotor motorGlisieraOriz;
    public DistanceSensor senzorDistanta;
    public Servo servobaza1;
    public Servo servobaza2;
    public Servo servoclesteIn;
    public Servo servoincheietura;
    public Servo servoClesteRot;

    public boolean mergeMotor;
    public double manualTarget = 0;

    public double HDes = 0;
    public Intake(HardwareMap hardwareMap){
        motorGlisieraOriz = hardwareMap.dcMotor.get("motorGlisieraOriz");
        servobaza1 = hardwareMap.servo.get("servoBaza1");
        servobaza2 = hardwareMap.servo.get("servoBaza2");
        servoclesteIn = hardwareMap.servo.get("servoClesteIn");
        servoClesteRot = hardwareMap.servo.get("servoClesteRot");
        servoincheietura = hardwareMap.servo.get("servoIncheietura");
        senzorDistanta = hardwareMap.get(DistanceSensor.class, "senzorDistanta");

        //Motor initialization
        motorGlisieraOriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisieraOriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisieraOriz.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void desfaCleste() {
        servoclesteIn.setPosition(0.3);
    }

    public void strangeCleste() {
        servoclesteIn.setPosition(0.7);
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

//    public void goUntilWall(){
//        if(senzorDistanta.getDistance(DistanceUnit.CM) >= 8){
//            motorGlisieraOriz.setPower(-1);
//        }
//        else {
//            motorGlisieraOriz.setPower(0);
//        }
//    }

    public void autoExtend(){
        if(senzorDistanta.getDistance(DistanceUnit.CM) >= 8){
            motorGlisieraOriz.setPower(-0.75);
        }
        else{
            motorGlisieraOriz.setPower(0);

        }
    }

    public boolean isGoing(int target){
        if (motorGlisieraOriz.getCurrentPosition() < target + 20 && motorGlisieraOriz.getCurrentPosition() > target - 20){
            return  false;
        }
        return true;

    }

    public void setHeight(double HDes){
        L1 = Math.sqrt ((Hpivot1 - HDes) * (Hpivot1 - HDes) + distCon * distCon);
        MidAng =57.2958 * Math.acos ((Lseg2 * Lseg2 - L1 * L1 + Lseg1 * Lseg1) / (2 * Lseg1 * Lseg2));
        A =Math.abs((Math.acos ((L1 * L1 + Lseg1 * Lseg1 - Lseg2 * Lseg2) / (2 * Lseg1 * L1))) * 57.2958);
        D = Math.atan (distCon / (Hpivot1 - HDes)) * 57.2958;
        FirstAng = 180 - D - A;
        if(FirstAng > 180) FirstAng = FirstAng - 180;
        B = Math.abs(180 - A - MidAng);
        Marcel = Math.acos (distCon / L1) * 57.2958;
        FinalAng = 180 - B - Marcel;
        servobaza1.setPosition((FirstAng / 180) + 0.5);
        servobaza2.setPosition(1 - ((FirstAng / 180) + 0.5));
        servoClesteRot.setPosition(0.5 - ((180-MidAng) / 250));
        servoincheietura.setPosition(0.54 - ((180-FinalAng) /180));
    }

    public void setServoBaza(double pos){
        servobaza2.setPosition(pos);
        servobaza1.setPosition(1-pos);
    }

    public void intakeToOuttake(){
        try {
            sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        setServoBaza(0.52);
        servoincheietura.setPosition(0);
        try {
            sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        servoClesteRot.setPosition(0.575);
    }

}


