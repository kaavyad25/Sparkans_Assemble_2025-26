package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;


/**
 * RobotFunctions is responsible for all operations the robot is able
 * to carry out i.e., intake, outtake, aiming. It is a subclass of
 * PIDControl.
 */
// Class is incomplete; REMEMBER TO COMMENT OUT AIMING RELATED CODE BEFORE NEXT COMP
public class RobotFunctions extends PIDControl{

    DcMotor leftIntakeMotor, rightIntakeMotor, leftOuttakeMotor, rightOuttakeMotor;
    DcMotor turretMotor;
    Servo launchServo;

    private static double       motorScalar = 1.0;
    private static final double CUSTOM_SERVO_EXTEND = 0.55; /** FIXME: Test for more suitable value*/
    public static final double  CENTER_OF_VIEW_X = 160.0;

    // TODO: Build calculateDistance
    // TODO: Build setOuttakePower (current setOuttakeScalar() is useless).
    // Moreover, disregard setOuttakeScalar() entirely

    private static final double APRILTAG_SIDELENGTH_INCHES = 6.5; /** FIXME: Measure more accurately*/
    private static final int    HUSKY_SCREEN_WIDTH = 160;
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

        this.leftIntakeMotor = hwMap.get(DcMotor.class, "leftIntakeMotor");
        this.rightIntakeMotor = hwMap.get(DcMotor.class, "rightIntakeMotor");

        this.leftOuttakeMotor = hwMap.get(DcMotor.class, "leftOuttakeMotor");
        this.rightOuttakeMotor = hwMap.get(DcMotor.class, "rightOuttakeMotor");

        turretMotor = hwMap.get(DcMotor.class, "turretMotor");

        this.launchServo = hwMap.get(Servo.class, "launchServo");

        this.leftIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.leftOuttakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightOuttakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.leftIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        this.rightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        this.leftOuttakeMotor.setDirection(DcMotor.Direction.FORWARD);
        this.rightOuttakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public final void fullExtend() {launchServo.setPosition(1.0);} /** for testing mainly*/
    public final void customExtend() {launchServo.setPosition(CUSTOM_SERVO_EXTEND);}
    public final void close() {launchServo.setPosition(0.0);}

    public final void activateIntake(double power, boolean isHeld){

        if (isHeld){
            this.leftIntakeMotor.setPower(power);
            this.rightIntakeMotor.setPower(power);
        } else {
            this.leftIntakeMotor.setPower(0.0);
            this.rightIntakeMotor.setPower(0.0);
        }
    }

    public final void activateOuttake(double power, boolean isHeld){

        if (isHeld) {
            this.leftOuttakeMotor.setPower(power * motorScalar);
            this.rightOuttakeMotor.setPower(power * motorScalar);
        } else {
            this.leftOuttakeMotor.setPower(0.0);
            this.rightOuttakeMotor.setPower(0.0);
        }
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



    //TODO: Fix, apparently entirely broken
    /*
    public double calculateDistance(){

        @NotNull
        double tagWidthPx = husky.getTagWidth();

        // We know the dimensions of the bounding box,
        // the pixel dimensions of the husky lens camera,
        // the FOV of the camera, in degrees,
        // and the pixel dimensions of the tag.

        double fractionOfImageWidth = tagWidthPx / HUSKY_SCREEN_WIDTH;

        // We can determine the dimensions of the image by dividing the
        // known April Tag dimension by the proportion of its width on
        // the screen.

        double screenWidthInches = APRILTAG_SIDELENGTH_INCHES / fractionOfImageWidth;

        // We can then imagine an isosceles triangle where the camera is at one point,
        // and opposite it is the image.
        // Then, the distance can be thought of as a line segment from the camera to the
        // image that bisects the angle HUSKY_CAMERA_FOV_DEGREES, and because this triangle is
        // isosceles, it perpendicularly bisects screenWidthInches.
        // Ergo creating a right-angle triangle, wherein we know the angle (HUSKY_CAMERA_FOV_DEGREES / 2)
        // and the side opposite it (screenWidthInches / 2).

        double distance = (screenWidthInches / 2) / (2 * Math.tan(HUSKY_CAMERA_FOV_DEGREES / 2));


        return distance;
    }

     */
}
