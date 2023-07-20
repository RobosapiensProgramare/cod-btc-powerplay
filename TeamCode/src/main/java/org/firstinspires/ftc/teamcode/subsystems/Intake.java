package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private static final double POWER = 1;
    public DcMotor motorGlisieraOriz;
    public Servo servobaza1;
    public Servo servobaza2;
    public Servo servoclestein;
    public Servo servoincheietura;
    public Servo servoclesterot;

    public boolean mergeMotor;
    public double manualTarget = 0;

    public Intake(HardwareMap hardwareMap){
        motorGlisieraOriz = hardwareMap.dcMotor.get("motorGlisieraOriz");
        servobaza1 = hardwareMap.servo.get("servoBaza1");
        servobaza2 = hardwareMap.servo.get("servoBaza2");
        servoclestein = hardwareMap.servo.get("servoClesteIn");
        servoclesterot = hardwareMap.servo.get("servoClesteRot");
        servoincheietura = hardwareMap.servo.get("servoIncheietura");


        //Motor initialization
        motorGlisieraOriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisieraOriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisieraOriz.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void desfaCleste() {
        servoclestein.setPosition(0);
    }

    public void strangeCleste() {
        servoclestein.setPosition(0.4);
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

    public boolean isGoing(int target){
        if (motorGlisieraOriz.getCurrentPosition() < target + 20 && motorGlisieraOriz.getCurrentPosition() > target - 20){
            return  false;
        }
        return true;

    }


}


