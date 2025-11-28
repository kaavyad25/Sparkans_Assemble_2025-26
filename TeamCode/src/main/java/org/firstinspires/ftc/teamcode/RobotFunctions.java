
/*
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




// Class is incomplete; REMEMBER TO COMMENT OUT AIMING RELATED CODE BEFORE NEXT COMP
//TODO: Refactor entire class; Turn into manager for subsystems, i.e., TurretSystem, IntakeSystem, OuttakeSystem, HuskyLensCamera.
//This will be easier to debug, more logical implementation of PIDControl.

public class RobotFunctions extends PIDControl{

    DcMotor intakeMotor, outtakeMotor;
    DcMotor turretMotor;
    Servo launchServo;

    private static double       motorScalar = 1.0;
    private static final double CUSTOM_SERVO_EXTEND = 0.55;
    public static final double  CENTER_OF_VIEW_X = 160.0;


    // TODO: Build setOuttakePower (current setOuttakeScalar() is useless).
    // Moreover, disregard setOuttakeScalar() entirely

    private static final double APRILTAG_SIDELENGTH_INCHES = 6.5;
    private static final int HUSKY_SCREEN_WIDTH_PIXELS = 160;
    private static final int    HUSKY_CAMERA_FOV_DEGREES = 160;
    private static final double SLOW_TURN = 0.25; // For when an AprilTag isn't detected.
    public static boolean       isExtended = false;
    private HuskyLensCamera husky;

    //-----------------------------------------------------------------------------------------------------------------
    //Construction
    //-----------------------------------------------------------------------------------------------------------------
    public RobotFunctions(double Kp, double Ki, double Kd, HuskyLensCamera husky, ElapsedTime elapsedTime) {

        super(Kp, Ki, Kd, elapsedTime);
        this.husky = husky;
    }


    public void init(HardwareMap hwMap){

        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        outtakeMotor = hwMap.get(DcMotor.class, "outtakeMotor");
        turretMotor = hwMap.get(DcMotor.class, "turretMotor");

        launchServo = hwMap.get(Servo.class, "launchServo");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public final void fullExtend() {launchServo.setPosition(1.0);}
    public final void customExtend() {launchServo.setPosition(CUSTOM_SERVO_EXTEND);}
    public final void close() {launchServo.setPosition(0.0);}

    public final void activateIntake(double power, boolean isHeld){

        if (isHeld) intakeMotor.setPower(power);
        else intakeMotor.setPower(0);
    }

    public final void activateOuttake(double power, boolean isHeld){

        if (isHeld) outtakeMotor.setPower(power);
        else outtakeMotor.setPower(0);
    }

    public final void setOuttakeScalar(double newScale){ motorScalar = newScale; }

    public final void servoEvent(boolean isPressed){

        if (isPressed && !isExtended){customExtend();}
        else if (isPressed && isExtended) {close();}
    }

    @Override
    public double getCurrentValue(){

        double current = husky.getAprilTagOffset();
        return  current;
    }
    @Override
    public void applyOutput(double output){

        output = Range.clip(output, -1.0, 1.0);
        turretMotor.setPower(output);
    }

    public void continuousUpdate(){

        // No AprilTag found... look around for one.
        if (-1 == getCurrentValue()){

            applyOutput(SLOW_TURN);
            return;
        }
        update(CENTER_OF_VIEW_X);
    }


    public double calculateDistance() {

        double tagWidthPx = husky.getTagWidth();

        // Fraction of the camera's horizontal resolution
        double fractionOfImageWidth = tagWidthPx / HUSKY_SCREEN_WIDTH_PIXELS;

        // True width of the image plane at that distance
        double screenWidthInches = APRILTAG_SIDELENGTH_INCHES / fractionOfImageWidth;

        // Convert FOV to radians
        double halfFovRadians = Math.toRadians(HUSKY_CAMERA_FOV_DEGREES / 2);

        double distance = (screenWidthInches / 2) / Math.tan(halfFovRadians);

        return distance;
    }

}
*/