package org.firstinspires.ftc.teamcode;
/**
public class AutoDrive extends PIDControl {

    DriveTrain  dt = new DriveTrain();
    IMUOdometry imu = new IMUOdometry();


    private double  currentHeading = getCurrentValue();
    @Override
    public double getCurrentValue(){

        return imu.getContinuousHeading();

    }

    @Override
    public void applyOutput(){


    }

}
