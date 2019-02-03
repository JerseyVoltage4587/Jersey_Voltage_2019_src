package frc.robot.paths;

import java.io.BufferedReader;
import java.io.FileWriter;
import java.io.IOException;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.util.Gyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;

public class PathFollower {

	BufferedReader m_bufferedReader;
	boolean quit;
	int m_startEncoderLeft;
	int m_startEncoderRight;
	double m_startAngle;
	double m_startTime;
	double Ka = Constants.kPathFollowKa;
	double Kv = Constants.kPathFollowKv;
	double Kp = Constants.kPathFollowKp;
	double Kg = Constants.kPathFollowKg;
	
	FileWriter m_logWriter=null;
	String m_namePath;
	Trajectory m_leftPath;
	Trajectory m_rightPath;

	double aLeft;
	public double getALeft(){
		return aLeft;
	}
	double vLeft;
	public double getVLeft(){
		return vLeft;
	}
	double xLeft;
	public double getXLeft(){
		return xLeft;
	}
	double aRight;
	public double getARight(){
		return aRight;
	}
	double vRight;
	public double getVRight(){
		return vRight;
	}
	double xRight;
	public double getXRight(){
		return xRight;
	}
	int step0;
	public int getStep0(){
		return step0;
	}
	int step1;
	public int getStep1(){
		return step1;
	}
	double desiredAngle;
	public double getDesiredAngle(){
		return desiredAngle;
	}
	
	double m_finalPositionRight;
	public double getFinalPositionRight(){
		return m_finalPositionRight;
	}
	double m_finalPositionLeft;
	public double getFinalPositionLeft(){
		return m_finalPositionLeft;
	}
	double m_leftMotorSetting;
	public double getLeftMotorSetting(){
		return m_leftMotorSetting;
	}
	double m_rightMotorSetting;
	public double getRightMotorSetting(){
		return m_rightMotorSetting;
	}
	double m_leftPos;
	public double getLeftPos(){
		synchronized(Drive.class){
			return m_leftPos;
		}
	}
	double m_rightPos;
	public double getRightPos(){
		synchronized(Drive.class){
			return m_rightPos;
		}
	}
	
	private void setMotorLevels(double l, double r){
		m_leftMotorSetting = l;
		m_rightMotorSetting = r;
	}
    public PathFollower(Trajectory leftPath, Trajectory rightPath) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	m_leftPath = leftPath;
    	m_rightPath = rightPath;
    }

    // Called just before this Command runs the first time
    public void initialize() {
    	quit = false;
		
		try {
			m_logWriter = new FileWriter("/home/lvuser/" + m_namePath +"Log.csv", false);
			m_logWriter.write("aLeft,vLeft,xLeft,aRight,vRight,xRight,desiredAngle,currentAngle,realLeftEncoder,realRightEncoder,leftMotorLevel,rightMotorLevel,leftAcc,leftVel,leftPos,leftG,leftMotorLevel,rightAcc,rightVel,rightPos,rightG,rightMotorLevel,System.nanoTime()" + "\n");
		} catch ( IOException e ) {
			System.out.println(e);
			m_logWriter = null;
		}
		
    	m_startEncoderLeft = Robot.getDrive().getLeftEnc();
    	m_startEncoderRight = Robot.getDrive().getRightEnc();
    	m_startAngle = Gyro.getYaw();
    	m_startTime = System.nanoTime();

    	m_finalPositionLeft = m_leftPath.get(m_leftPath.length()-1).position * 12 / Constants.kInchesPerTic;
    	m_finalPositionRight = m_rightPath.get(m_leftPath.length()-1).position * 12 / Constants.kInchesPerTic;
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {

    	double time = System.nanoTime();
    	double dt = (time - m_startTime) / 1000000;//ms
    	step0 = (int)(dt / 10);
    	step1 = step0 + 1;
    	double offset = dt - 10 * step0;
    	
    	
    	if(step1 >= m_leftPath.length())
    	{
    		quit = true;
    		step0 = step1 = m_leftPath.length()-1;
		}
	    		Trajectory.Segment left0;
	        	Trajectory.Segment right0;
	    		Trajectory.Segment left1;
	        	Trajectory.Segment right1;
	        	
            	left0 = m_leftPath.get(step0);
            	left1 = m_leftPath.get(step1);
            	right0 = m_rightPath.get(step0);
				right1 = m_rightPath.get(step1);
		
				//System.out.println("p: "+left0.position+" v: "+left0.velocity+" a: "+left0.acceleration+" yaw: "+left0.heading);
				//System.out.println("p: "+right0.position+" v: "+right0.velocity+" a: "+right0.acceleration+" yaw: "+right0.heading);

	        	xLeft = (left0.position + ((offset / 10) * (left1.position - left0.position))) * 12 / Constants.kInchesPerTic;
	        	xRight = (right0.position + ((offset / 10) * (right1.position - right0.position))) * 12 / Constants.kInchesPerTic;
	        	if(step0 == step1){
	            	/*aLeft = 0;
	            	vLeft = 0;
	            	aRight = 0;
		        	vRight = 0;
		        	Ka = 0;
		        	Kv = 0;
		        	Kp = Constants.kPathHoldKp;
		        	Kg = Constants.kPathHoldKg;*/
	        	}else{
	        		aLeft = (left0.acceleration + ((offset / 10) * (left1.acceleration - left0.acceleration))) * 12 / Constants.kInchesPerTic / 1000 * 10 / 1000 * 10;
	        		vLeft = (left0.velocity + ((offset / 10) * (left1.velocity - left0.velocity))) * 12 / Constants.kInchesPerTic / 1000 * 10;//convert ft/sec to ticks/10ms
	        		aRight = (right0.acceleration + ((offset / 10) * (right1.acceleration - right0.acceleration))) * 12 / Constants.kInchesPerTic / 1000 * 10 / 1000 * 10;
	        		vRight = (right0.velocity + ((offset / 10) * (right1.velocity - right0.velocity))) * 12 / Constants.kInchesPerTic / 1000 * 10;
	        		Ka = Constants.kPathFollowKa;
	        		Kv = Constants.kPathFollowKv;
	        		Kp = Constants.kPathFollowKp;
	        		Kg = Constants.kPathFollowKg;
				}
				
        		desiredAngle = -right0.heading * 180 / Math.PI; //* -1;
        		double currentAngle = Gyro.getYaw();
        		int realLeftEncoder = Robot.getDrive().getLeftEnc();
        		int realRightEncoder = Robot.getDrive().getRightEnc();
        		desiredAngle += m_startAngle;
        		while(desiredAngle > 180)
        		{
        			desiredAngle -= 360;
        		}
        		while(desiredAngle < -180)
        		{
        			desiredAngle += 360;
        		}
		
        		m_leftPos = realLeftEncoder - m_startEncoderLeft;
        		m_rightPos = realRightEncoder - m_startEncoderRight;
        		Robot.getDrive().setPathPos(m_leftPos, m_rightPos);
        		
        		xLeft += m_startEncoderLeft;
        		xRight += m_startEncoderRight;
        		//---
		
        		double angleError = currentAngle - desiredAngle;
        		while(angleError>180.0){
        			angleError-=360.0;
        		}
        		while(angleError<-180.0){
        			angleError+=360.0;
				}
				
        		double leftMotorLevel = Ka * aLeft + Kv * vLeft - Kp * (realLeftEncoder - xLeft) - Kg * angleError;
        		double rightMotorLevel = Ka * aRight + Kv * vRight - Kp * (realRightEncoder - xRight) + Kg * angleError;
        		//String leftMotorThings = (Ka*aLeft)+","+(Kv*vLeft)+","+ (-Kp * (realLeftEncoder - xLeft))+ ","+(-Kg*angleError)+","+leftMotorLevel;
        		//String rightMotorThings = (Ka*aRight)+","+(Kv*vRight)+","+ (-Kp * (realRightEncoder - xLeft))+ ","+(Kg*angleError)+","+rightMotorLevel;
				
        		if(Math.abs(realLeftEncoder - m_finalPositionLeft)<Constants.kPathDoneTicsTolerance&&Math.abs(realRightEncoder - m_finalPositionRight)<Constants.kPathDoneTicsTolerance){
        			//quit = true;
				}
				
        			//---
        		setMotorLevels(leftMotorLevel, -rightMotorLevel);
        		//SmartDashboard.putNumber("left motor set to: ", leftMotorLevel);
        		//SmartDashboard.putNumber("right motor set to: ", -rightMotorLevel);
            	
        		if(m_logWriter != null)
        		{
        			try{
						m_logWriter.write((int)leftMotorLevel*1000);
						m_logWriter.write((int)rightMotorLevel*1000);
        				//m_logWriter.write(aLeft + "," + vLeft + "," + xLeft + "," + aRight + "," + vRight + "," + xRight + "," + desiredAngle + "," + currentAngle + "," + realLeftEncoder + "," + realRightEncoder + "," + leftMotorLevel + "," + rightMotorLevel +"," + time + "\n");
        			}catch(Exception e){
        				
        			}
				}
				
        	
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return quit;
    }

    // Called once after isFinished returns true
    protected void end() {
    	try
    	{
    		m_logWriter.close();
    	}
    	catch(Exception e)
    	{
    		
    	}
    }

}