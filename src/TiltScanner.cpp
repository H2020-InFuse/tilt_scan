#include "TiltScanner.hpp"

namespace tilt_scan {

  typedef std::vector<base::Point> PointVector;


  TiltScanner::TiltScanner(){
    mPointcloud = base::samples::Pointcloud();
    mSweepStatus = SweepStatus();

    mSweepStatus.counter = 0;
    mSweepStatus.curState = SweepStatus::INITIALIZING;
  }

  /**
   * Convert to pointcloud and add to global cloud
   */
  void TiltScanner::addPointsToCloud(const base::samples::LaserScan& laserScan, const Eigen::Affine3d& laser2odometry)
  {
	PointVector points;
	laserScan.convertScanToPointCloud(points, laser2odometry);
	for(PointVector::iterator it = points.begin(); it < points.end(); ++it)
	{
		mPointcloud.points.push_back(*it);
	}
   }

   base::samples::Pointcloud TiltScanner::sendData(const Eigen::Affine3d& odometry2body, base::samples::Pointcloud result)
   {
	if(mSweepStatus.cycleFinished)
	{
	   result = base::samples::Pointcloud ();
	   result.time = mSweepStatus.timeStamp;
	    for(PointVector::iterator it = mPointcloud.points.begin(); it < mPointcloud.points.end(); ++it)
	    {
		result.points.push_back(odometry2body * (*it));
	    }
	    mSweepStatus.counter++;
	}

	return result;
   }

  /**
  * clear data if necessary
  */
  void TiltScanner::clearPointcloud()
  {
	if(mSweepStatus.newCycleStarted)
	{
	  mPointcloud.points.clear();
	}
  }
   void TiltScanner::determineState(base::JointState joint, base::Time time)
   {

      mSweepStatus.cycleFinished = false;
      mSweepStatus.newCycleStarted = false;
      mSweepStatus.timeStamp = time;

      if(mSweepStatus.lastJointPosition == std::numeric_limits<double>::infinity())
      {
	     mSweepStatus.lastJointPosition = joint.position;
	     return;
      }

      SweepStatus::Direction curDirection = joint.position-mSweepStatus.lastJointPosition > 0 ? SweepStatus::UP : joint.position-mSweepStatus.lastJointPosition == 0 ? SweepStatus::NONE : SweepStatus::DOWN;     

      if(mSweepStatus.curState != SweepStatus::SWEEPING_DOWN && curDirection == SweepStatus::DOWN)
      {
	  if(mSweepStatus.curState == SweepStatus::SWEEPING_UP /*mConfiguration.sweep_back_and_forth*/)
	      mSweepStatus.cycleFinished = true;
	  mSweepStatus.newCycleStarted = true;
	  mSweepStatus.curState = SweepStatus::SWEEPING_DOWN;
      }

      else if(mSweepStatus.lastDirection ==  SweepStatus::UP && curDirection == SweepStatus::NONE)
      {
	  mSweepStatus.curState = SweepStatus::REACHED_UP_POSITION;
      }

      else if(mSweepStatus.curState == SweepStatus::SWEEPING_DOWN && curDirection == SweepStatus::UP)
      {
		mSweepStatus.cycleFinished = true;
		mSweepStatus.newCycleStarted = true;
		mSweepStatus.curState = SweepStatus::SWEEPING_UP;
      }
      mSweepStatus.lastJointPosition = joint.position;
      mSweepStatus.lastDirection = curDirection;
   }

   void TiltScanner::updateState(const Configuration& mConfiguration,const base::samples::Joints& joints, bool mTrigger, base::Time time)
   {
     mSweepStatus.cycleFinished = false;
     mSweepStatus.newCycleStarted = false;
     mSweepStatus.timeStamp = time;

     mSweepStatus.sourceName = mConfiguration.sweep_servo_name;
     base::JointState jointState = joints.getElementByName(mConfiguration.sweep_servo_name);

     //Direction curDirection = jointState.position;

     // Initialized to up position
     if(mSweepStatus.curState == SweepStatus::INITIALIZING && fabs(jointState.position - mConfiguration.sweep_angle_max) < 0.1)
     {
	      if(mConfiguration.mode == Configuration::CONTINUOUS_SWEEPING)
	      {
		  mSweepStatus.newCycleStarted = true;
		  mSweepStatus.curState = SweepStatus::SWEEPING_DOWN;

	      }
	      else
	      {
		      mSweepStatus.curState = SweepStatus::REACHED_UP_POSITION;
	      }
      }

      // Reached upper end point
      else if((mSweepStatus.curState == SweepStatus::SWEEPING_UP) && fabs(jointState.position - mConfiguration.sweep_angle_max) < 0.1)
      {
	      if(mConfiguration.mode == Configuration::CONTINUOUS_SWEEPING)
	      {

		    if(mConfiguration.sweep_back_and_forth)
		    {
			mSweepStatus.cycleFinished = true;
		    }

		    mSweepStatus.newCycleStarted = true;
		    mSweepStatus.curState = SweepStatus::SWEEPING_DOWN;

	      } 
	      else
	      {
		      mSweepStatus.curState = SweepStatus::REACHED_UP_POSITION;
	      }
      }

      // Reached lower endpoint
      else if((mSweepStatus.curState == SweepStatus::SWEEPING_DOWN) && fabs(jointState.position - mConfiguration.sweep_angle_min) < 0.1)
      {		
	      mSweepStatus.cycleFinished = true;
	      mSweepStatus.newCycleStarted = true;
	      mSweepStatus.curState = SweepStatus::SWEEPING_UP;
      }

      // Received trigger signal
      else if((mSweepStatus.curState == SweepStatus::REACHED_UP_POSITION) && mTrigger)
      {
	    mSweepStatus.newCycleStarted = true;
	    mSweepStatus.curState = SweepStatus::SWEEPING_DOWN;
      }
   }
}
