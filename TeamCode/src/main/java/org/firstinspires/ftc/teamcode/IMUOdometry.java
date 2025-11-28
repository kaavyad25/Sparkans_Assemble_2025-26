package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUOdometry {

    private BNO055IMU imu;
    private static double  lastRawYawDeg;
    private static double continuousYawDeg;


    public void init(HardwareMap hwMap){

        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();

        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.mode = BNO055IMU.SensorMode.IMU;

        imu.initialize(params);

    }

    public Orientation getRobotOrientation(){

        return imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        );
    }

    public double getHeadingDeg(){ return getRobotOrientation().firstAngle; }
    public double getRollDeg() { return getRobotOrientation().thirdAngle; }

    public double getContinuousHeadingDeg(){

        double rawYawDeg = getHeadingDeg();
        double deltaDeg = rawYawDeg - lastRawYawDeg;

        // Wrap around handling (-180 to 180)
        if (deltaDeg < -180) deltaDeg += 360;
        if (deltaDeg > 180) deltaDeg -= 360;

        continuousYawDeg += deltaDeg;
        lastRawYawDeg = rawYawDeg;

        return continuousYawDeg;

    }
}