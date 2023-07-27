package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    private static final double POWER = 1;
    public DcMotor motorGlisiera;
    public DcMotor motorGlisiera2;

    private Servo servoCupa1, servoCupa2, servoBat;

    public boolean mergeMotor;
    public double manualTarget = 0;

    public Outtake(HardwareMap hardwareMap){
        motorGlisiera = hardwareMap.dcMotor.get("motorGlisiera");
        motorGlisiera2 = hardwareMap.dcMotor.get("motorGlisiera2");

        servoCupa1 = hardwareMap.servo.get("servoCupa1");
        servoCupa2 = hardwareMap.servo.get("servoCupa2");
        servoBat = hardwareMap.servo.get("servoBat");

        //Motor initialization
        motorGlisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera.setDirection(DcMotorSimple.Direction.FORWARD);

        motorGlisiera2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //SERVOURI CUPA:

    public void ridicaCupa(){
        servoCupa1.setPosition(0.9);
        servoCupa2.setPosition(1-0.9);
    }

    public void coboaraCupa(){
        servoCupa1.setPosition(0.25);
        servoCupa2.setPosition(1-0.25);
    }

    public void setCupa(double pos){
        servoCupa1.setPosition(pos);
        servoCupa2.setPosition(1 - pos);
    }

    public void inchideBat(){
        servoBat.setPosition(0.2);
    }

    public void deschideBat(){
        servoBat.setPosition(0.5);
    }

    public void deschideCupa(){
        servoBat.setPosition(1);
    }

    public void inchideCupa(){
        servoBat.setPosition(0);
    }

    //MOTOARE GLISIERA:

    public void setPower(double power){
        motorGlisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisiera.setPower(power);

        motorGlisiera2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisiera2.setPower(power);
    }

    public void manualLevel(double manualTarget) {
        motorGlisiera.setTargetPosition((int) manualTarget);
        motorGlisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorGlisiera2.setTargetPosition((int) manualTarget);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motorGlisiera.getCurrentPosition() > manualTarget && motorGlisiera2.getCurrentPosition() > manualTarget)
        {
            motorGlisiera.setPower(-POWER);
            motorGlisiera2.setPower(-POWER);
        }
        else{
            motorGlisiera.setPower(POWER);
            motorGlisiera2.setPower(POWER);
        }
    }

    public void setLevel(int target, double multiplier){
        motorGlisiera.setTargetPosition(-target);
        motorGlisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorGlisiera2.setTargetPosition(-target);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motorGlisiera.getCurrentPosition() > -target && motorGlisiera2.getCurrentPosition() > -target) {
            motorGlisiera.setPower(-POWER);
            motorGlisiera2.setPower(-POWER);
        }
        else
        {
            motorGlisiera.setPower(POWER * multiplier);
            motorGlisiera2.setPower(POWER * multiplier);
        }
    }

    public boolean isGoing(int target){
        if (motorGlisiera.getCurrentPosition() < target + 20 && motorGlisiera.getCurrentPosition() > target - 20){
            return  false;
        }
        return true;

    }

    public int getPosition(){
        return motorGlisiera.getCurrentPosition();
    }

    public double getServoPos(){
        return servoCupa1.getPosition();
    }
}


