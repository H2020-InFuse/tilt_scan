#ifndef __TILT_SCANNER_HPP__
#define __TILT_SCANNER_HPP__

#include <stdint.h>
#include <base/Pose.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/LaserScan.hpp>
#include <base/samples/Pointcloud.hpp>
#include "TiltScanConfiguration.hpp"

namespace tilt_scan {

    struct SweepStatus
    {
	    base::Time timeStamp; 
      
	    SweepStatus() : counter(0), curState(INITIALIZING), lastJointPosition(std::numeric_limits<double>::infinity()),lastDirection(NONE)
	    {};

	    enum State {
		    NOT_SWEEPING,
		    INITIALIZING,
		    REACHED_UP_POSITION,
		    SWEEPING_UP,
		    SWEEPING_DOWN,
	    };
	    
	    enum Direction {
	      UP,
	      DOWN,
	      NONE,
	    };
	    ///counter of the sweeps, wraps at 255
	    uint8_t counter;
	    
	    ///current state of sweeping
	    State curState;
	    Direction lastDirection;
	    double lastJointPosition;

	    bool cycleFinished;
	    bool newCycleStarted;

	    ///name of the sweeping device
	    std::string sourceName;

	    /**
	      * Return true if one ore more sweeps
	      * in respect to the given state werde done.
	      * 
	      * Also returns true, if not sweeping at all.
	      * */
	    bool isNextSweep(SweepStatus lastState)
	    {
		    return NOT_SWEEPING || lastState.counter != counter;
	    }

	    bool isSweeping()
	    {
		    return curState!=NOT_SWEEPING;
	    }
    };

    class TiltScanner
    {
        public:
	  TiltScanner();
	  void addPointsToCloud(const base::samples::LaserScan& laserScan, const Eigen::Affine3d& laser2odometry);
	  base::samples::Pointcloud sendData(const Eigen::Affine3d& odometry2body, base::samples::Pointcloud result);
	  void clearPointcloud();
	  void updateState(const Configuration& mConfiguration,const base::samples::Joints& joints, bool mTrigger, base::Time time);

	  /**
	   * can be used instead of updateState if no configuration is known
	   */
	  void determineState(base::JointState joint, base::Time time);

	  SweepStatus mSweepStatus;
	private:
          base::samples::Pointcloud mPointcloud;

    };
}
#endif
