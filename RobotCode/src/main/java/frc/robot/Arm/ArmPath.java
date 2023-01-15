package frc.robot.Arm;

import edu.wpi.first.util.InterpolatingTreeMap;

public class ArmPath {

    public enum State{
        ACCEL_CONCAVE,
        ACCEL_CONST,
        ACCEL_CONVEX,
        LINEAR_TRAVEL,
        DECEL_CONVEX,
        DECEL_CONST,
        DECEL_CONCAVE,
    }

    //Time to linear position
    InterpolatingTreeMap<Double, Double> path;

    ArmPosition start; 
    ArmPosition end;
    double max_vel;
    double max_accel; 
    double max_jerk;
    double max_dist;

    /**
     * Create a new S-profile trajectory from start to end with the given constraints
     * See Section 5.5 of http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf for equations
     * @param start
     * @param end
     * @param max_vel_mps
     * @param max_accel_mps2
     * @param max_jerk_mps3
     */
    public ArmPath(ArmPosition start, ArmPosition end, double max_vel_mps, double max_accel_mps2, double max_jerk_mps3){
        this.start = start; 
        this.end = end;

        this.max_dist = this.start.distTo(this.end);
        this.max_vel = max_vel_mps;
        this.max_accel = max_accel_mps2; 
        this.max_jerk = max_jerk_mps3;

        path = new InterpolatingTreeMap<Double, Double>();

        var sim_Ts = 0.02;
        var simT = 0.0;
        var cur_dist = 0.0;
        var cur_vel = 0.0;
        var cur_accel = 0.0;
        var cur_jerk = 0.0;
        var cur_state = State.ACCEL_CONCAVE;
        var accel_time = 0.0; 
        var jerk_time = 0.0; 
        var totalTime = 0.0;

        path.put(simT, cur_dist);
        simT += sim_Ts;

        while(cur_dist <= max_dist){

            //In-state
            switch(cur_state){
                case ACCEL_CONCAVE:
                    cur_jerk = this.max_jerk;
                break;
                case ACCEL_CONST:
                    cur_jerk = 0;
                break;
                case ACCEL_CONVEX:
                    cur_jerk = -1.0 * this.max_jerk;
                break;
                case LINEAR_TRAVEL:
                    cur_jerk = 0.0;
                break;
                case DECEL_CONCAVE:
                    cur_jerk = -1.0 * this.max_jerk;
                break;
                case DECEL_CONST:
                    cur_jerk = 0;
                break;
                case DECEL_CONVEX:
                    cur_jerk = this.max_jerk;
                break;

            }

            cur_accel += cur_jerk * sim_Ts;
            cur_vel += cur_accel * sim_Ts;
            cur_dist += cur_vel * sim_Ts;

            path.put(simT, cur_dist);
            simT += sim_Ts;

            //State Transitions
            switch(cur_state){
                case ACCEL_CONCAVE:
                    if(cur_accel >= max_accel){
                        cur_state = State.ACCEL_CONST;
                        jerk_time = simT;
                    } else if(cur_vel >= max_vel/2.0){
                        cur_state = State.ACCEL_CONVEX;
                        jerk_time = simT;
                    } else if(cur_dist >= max_dist / 4.0){
                        cur_state = State.DECEL_CONVEX;
                        accel_time = 0.0;
                        jerk_time = simT;
                        totalTime = simT*2;
                    }
                break;
                case ACCEL_CONST:
                    if(cur_vel >= max_vel/2.0){
                        cur_state = State.ACCEL_CONVEX;
                        accel_time = simT - jerk_time;
                    }
                break;
                case ACCEL_CONVEX:
                    if(cur_vel >= max_vel || simT >= accel_time + 2.0*jerk_time) {
                        cur_state = State.LINEAR_TRAVEL;
                    }
                break;
                case LINEAR_TRAVEL:
                    if(cur_dist >= max_dist/2.0 && totalTime == 0.0){
                        totalTime = simT * 2.0;
                    }
                    if(totalTime > 0.0 && simT >= (totalTime - accel_time - 2.0*jerk_time)){
                        cur_state = State.DECEL_CONVEX;
                    }
                break;
                case DECEL_CONVEX:
                    if( simT >= (totalTime - accel_time - jerk_time)) {
                        cur_state = State.DECEL_CONST;
                    }
                break;
                case DECEL_CONST:
                    if(simT >= (totalTime - jerk_time)) {
                        cur_state = State.DECEL_CONCAVE;
                    }
                break;

            }

        }

        path.put(simT, cur_dist);

    }

    /**
     * Return the ArmPosition at a given time
     * @param time_sec
     * @return
     */
    public ArmPosition calculate(double time_sec){
        double curDist = path.get(time_sec);
        return this.start.interpolateTo(this.end, curDist/max_dist);
    }

}
