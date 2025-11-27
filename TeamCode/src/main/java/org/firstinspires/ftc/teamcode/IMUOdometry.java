package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUOdometry {

    private IMU     imu;
    private static double  lastRawYawDeg;
    private static double continuousYawDeg;


    public void init(HardwareMap hwMap){

        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters;

        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(parameters);
    }

    /*------------------------------------------------------------------
    Getters
     ------------------------------------------------------------------*/
    public YawPitchRollAngles getRobotOrientation() {return imu.getRobotYawPitchRollAngles();}
    public double getHeadingDeg() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}
    public double getRollDeg() {return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);}

    public double getContinuousHeadingDeg(){

            double rawYawDeg = getHeadingDeg();
            double deltaDeg = rawYawDeg - lastRawYawDeg;

            if (deltaDeg < -180.0) { deltaDeg += 360.0; }
            if (deltaDeg > 180.0) {deltaDeg -= 360.0; }

            continuousYawDeg += deltaDeg;
            lastRawYawDeg = rawYawDeg;

            return continuousYawDeg;
    }
}