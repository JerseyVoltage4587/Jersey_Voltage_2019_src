package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.robot.util.AsyncStructuredLogger;

public class Arm extends Subsystem {
	
    private static Arm mInstance = null;

     public static Arm getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Arm.class ) {
    			if ( mInstance == null ) {
    				mInstance = new Arm();
    			}
    		}
    	}
    	return mInstance;
    }
    
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
        	if(mHaveCalledOnStart == false ){
            	mStartTime = System.nanoTime();
            	mHaveCalledOnStart = true;
        	}
        }

        @Override
        public void onLoop(double timestamp) {
            mArmEncoder = can.getQuadraturePosition()*-1;//armMotor.getSelectedSensorPosition();
        	mArmPos = getPosDegrees();
        	mArmError = mArmSetpoint - mArmPos;
        	/*if(Math.abs(mArmError) < Constants.kLiftTolerance){
        		mIsAtSetpoints = true;
        	}else{
        		mIsAtSetpoints = false;
        	}*/
            synchronized (Arm.class) {
            	xArmPos = mArmPos;
        		xIsAtSetpoints = mIsAtSetpoints;
            	mArmSetpoint = xArmSetpoint;
            }
			
			doPathFollowing();

            logValues();
            mLastArmError = mArmError;
        }

        @Override
        public void onStop(double timestamp) {
            onStart(timestamp);
            mCSVWriter.flush();
        }
    };
    
    private void setArmMotorLevels(double x){
    	if(x < Constants.kArmMaxMotorDown){
    		x = Constants.kArmMaxMotorDown;
    	}
    	if(x > Constants.kArmMaxMotorUp){
    		x = Constants.kArmMaxMotorUp;
    	}
    	x = -x;
    	armMotor.set(x);
    }
	
	private void doPathFollowing(){

    	double error = mArmSetpoint - mArmPos;
    	double arm_motor_level;
    	//double armRealPos = mArmPos + 12;
		if (error>60.0){
			arm_motor_level = Constants.kArmMaxMotorUp;
			arm_motor_level -= Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}else if (error>5.0){
			arm_motor_level = Constants.kArmSlowMotorUp;
			arm_motor_level -= Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}else if(error<-60.0){
			arm_motor_level = Constants.kArmMaxMotorDown;
			arm_motor_level -= Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}else if(error<-5.0){
			arm_motor_level = Constants.kArmSlowMotorDown;
			arm_motor_level -= Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}else{
			arm_motor_level = 0.0;//getArmPIDOutput(mArmSetpoint);
			arm_motor_level -= 3*Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}
		
		SmartDashboard.putNumber("arm setpoint", mArmSetpoint);
		SmartDashboard.putNumber("arm motorlevel", arm_motor_level);
		
		setArmMotorLevels(arm_motor_level);
	}

	// S H A R E D   A C C E S S
	// These member variables can be accessed by either thread, but only by calling the appropriate getter method.
    
    private double mArmPos;
    private double xArmPos;
    public double getArmPos(){
    	synchronized (Arm.class){
    		return xArmPos;
    	}
    }
    
    private double mArmSetpoint=0;
    private double xArmSetpoint=0;
    public void setArmSetpoint(double setpoint){
    	synchronized (Arm.class){
			xArmSetpoint = setpoint;

			double lowStop = Constants.kArmSoftStopLow;
			double highStop = Constants.kArmSoftStopHigh;
			if(Robot.getIntake().getHasHatch() == true){
				lowStop = Constants.kArmHatchStopLow;
				highStop = Constants.kArmHatchStopHigh;
			}

			if(setpoint < lowStop){
				xArmSetpoint = lowStop;
			}else if(setpoint > highStop){
				xArmSetpoint = highStop;
			}
    	}
    }
    public double getArmSetpoint(){
    	synchronized (Arm.class){
    		return xArmSetpoint;
    	}
    }
    
    private boolean mIsAtSetpoints = false;
    private boolean xIsAtSetpoints = false;
    public boolean isAtSetpoint(){
    	return xIsAtSetpoints;
    }
    
	// S U B S Y S T E M   A C C E S S
	// These member variables can be accessed only by the subsystem

    // Hardware
	private final WPI_TalonSRX armMotor;
	private final CANifier can;
    
    private Arm() {
		armMotor = new WPI_TalonSRX(RobotMap.ARM_TALON);
        armMotor.setNeutralMode(NeutralMode.Brake);
		armMotor.configNeutralDeadband(0.01, 10);
		armMotor.setInverted(true);
		can = new CANifier(RobotMap.CANIFIER);
		can.configFactoryDefault();
		
		
        mDebugOutput = new DebugOutput();
        mCSVWriter = new AsyncStructuredLogger<DebugOutput>("ArmLog", DebugOutput.class);
    }
    
    private long mStartTime;
	private boolean mHaveCalledOnStart = false;
	private double mArmEncoder;
    private double mArmError, mLastArmError;

    private double getPosDegrees(){
 	   return can.getQuadraturePosition() * -1 * Constants.kArmDegreesPerTic;
    }

    private double getArmPIDOutput(double setpoint_to_use){
    	double error = setpoint_to_use - mArmPos;
    	double output = error * Constants.kArmHoldKp + Math.min(error, mLastArmError) * Constants.kArmHoldKi - (error - mLastArmError) * Constants.kArmHoldKd;
		if(output>=0){//TODO should use error instead of output, output might flip signs erratically
			output -= Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}else{
			output += Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}
		mLastArmError = error;
		return output;
    }
    
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }
        
    @Override
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("Arm Setpoint", getArmSetpoint());
    	SmartDashboard.putNumber("Arm Degrees", getPosDegrees());
    	SmartDashboard.putNumber("Arm encoder", can.getQuadraturePosition()*-1);
    	SmartDashboard.putNumber("Arm Motor Percent", armMotor.get());
    }

    // Logging
    private DebugOutput mDebugOutput;
    private final AsyncStructuredLogger<DebugOutput> mCSVWriter;
    
    public static class DebugOutput{
    	public long sysTime;
    	public double armEncoder;
    	public double armPosDeg;
    	public double motorPercent;
    }
    
    public void logValues(){
	    mDebugOutput.sysTime = System.nanoTime()-mStartTime;
	    mDebugOutput.armEncoder = mArmEncoder;
	    mDebugOutput.armPosDeg = mArmPos;
	    mDebugOutput.motorPercent = armMotor.get();
		mCSVWriter.queueData(mDebugOutput);
    }

	//unused overrides
    @Override
    public void writeToLog() {}
	@Override
	protected void initDefaultCommand() {}
	@Override
	public void zeroSensors() {
		can.setQuadraturePosition(0, 10);
	}
    @Override
    public synchronized void stop() {}
}