



package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Testing {
    private static double   speedScalar = 0.0;

    // initialize motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    public boolean  isMoving = false;


    public void init(HardwareMap hwMap){

        this.frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        this.frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        this.backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        this.backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");

        this.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         /** -----------------------------------------
        IF THE ROBOT IS DRIVING IN REVERSE, FLIP THESE
        ---------------------------------------------
         */

        this.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        this.backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        this.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        this.backRightMotor.setDirection(DcMotor.Direction.FORWARD);


    }

    // restrict the angle reading, in radians, from the imu [-π, π]
    private double angleWrap(double rad){
        while (rad > Math.PI) { rad -= Math.PI * 2;}
        while (rad < -Math.PI) { rad += Math.PI *2;}
        return rad;
    }
        /**
        field oriented drive; movement control is relative to the field, disregarding robot direction
        ex. If the robot is facing backward relative to the field, but the joystick points forward, the robot drives forward
        */

    public void fieldOrientedTranslate(double targetPowerX, double targetPowerY, double rotation, double currentRotation) {


        // direction robot is facing, in degrees
        double robotYawDeg = Math.toDegrees(angleWrap(Math.toRadians(currentRotation)));

        // direction stick is pointing, mapped to [-180, 180]
        double stickRotationDeg = Math.toDegrees(Math.atan2(targetPowerY, targetPowerX));

        // offset joystick vector to account for robot orientation
        double thetaDeg = 360.0 - robotYawDeg + stickRotationDeg;

        // sets power to be length of joystick vector
        double power = Math.hypot(targetPowerX, targetPowerY);

        // restricting power between [-1, 1] because of a bug;
        // if abs(targetPowerX), abs(targetPowerY) = 1, then power would be abs(√2)
        power = Range.clip(power, -1.0, 1.0);

    }

        // android studio was complaining about the complex if statement
        private boolean motorsNotNull() {
            return frontLeftMotor != null && frontRightMotor != null && backLeftMotor != null && backRightMotor != null;
        }

        // drive relative to the robot
    // similar to the above code, though without angle adjustments
        public void robotOrientedTranslate(double targetPowerX, double targetPowerY, double rotation){

            double thetaRad = Math.atan2(targetPowerY, targetPowerX);
            double power = Math.hypot(targetPowerX,targetPowerY);

            double sin = Math.sin(thetaRad - Math.PI/4);
            double cos = Math.cos(thetaRad - Math.PI/4);

            double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

            double frontRightPower;
            double frontLeftPower;
            double backRightPower;
            double backLeftPower;

            rotation *= -1;

            frontLeftPower = power*cos/maxSinCos + rotation;
            backLeftPower = power*sin/maxSinCos + rotation;

            frontRightPower = power*cos/maxSinCos - rotation; /** Swap if rotation is inverted */
            backRightPower = power*sin/maxSinCos - rotation;

            double frontMax = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            double backMax = Math.max(Math.abs(backLeftPower), Math.abs(backRightPower));

            double maxPower = Math.max(frontMax, backMax);

            if (maxPower > 1.0){
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;

                backLeftPower /= maxPower;
                backRightPower /= maxPower;

            }


            if (this.isMoving && this.motorsNotNull()){

                this.frontLeftMotor.setPower(frontLeftPower * this.speedScalar);
                this.frontRightMotor.setPower(frontRightPower * this.speedScalar);

                this.backLeftMotor.setPower(backLeftPower * this.speedScalar);
                this.backRightMotor.setPower(backRightPower * this.speedScalar);
            }
        }
        public void setSpeedScalar(double change) {
            // assigned from main function (whenever we build it)
            this.speedScalar = Range.clip(change, 0.0, 1.0);
        }
}