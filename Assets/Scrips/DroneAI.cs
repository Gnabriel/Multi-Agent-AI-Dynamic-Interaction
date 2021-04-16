using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    // ----- Unity objects -----
    private DroneController m_Drone;
    Rigidbody my_rigidbody;
    public GameObject my_goal_object;
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    // ----- /Unity objects -----

    public float desired_speed = 1.0f;      // TODO: Set speed.                     // Desired constant speed of the agents.

    // ----- PD controller parameters -----
    public Vector3 target_velocity;
    public Vector3 old_target_pos;
    public Vector3 desired_velocity;
    public float k_p = 2f;                                                          // TODO: Current k_p and k_d are for the car. Probably needs to be updated.
    public float k_d = 0.5f;
    // ----- /PD controller parameters -----


    private void Start()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
        my_rigidbody = GetComponent<Rigidbody>();


        Vector3 start_pos = terrain_manager.myInfo.start_pos;
        Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

        List<Vector3> my_path = new List<Vector3>();

        // Plan your path here
        my_path.Add(start_pos);

        for (int i = 0; i < 3; i++)
        {
            Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
            my_path.Add(waypoint);
        }
        my_path.Add(goal_pos);



        // Plot your path to see if it makes sense
        Vector3 old_wp = start_pos;
        foreach (var wp in my_path)
        {
            //Debug.DrawLine(old_wp, wp, Color.red, 100f);
            old_wp = wp;
        }

        
    }


    private void FixedUpdate()
    {
        // Execute your path here


        // ----- PD controller -----
        // Keep track of target position and velocity.
        target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
        old_target_pos = target_position;

        // A PD-controller to get desired velocity.
        Vector3 position_error = target_position - transform.position;
        Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
        Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

        //Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
        //Debug.DrawLine(transform.position, transform.position + my_rigidbody.velocity, Color.blue);
        //Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

        //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
        m_Drone.Move(steering, acceleration, acceleration, 0f);
        // ----- /PD controller -----


        // this is how you access information about the terrain
        //int i = terrain_manager.myInfo.get_i_index(transform.position.x);
        //int j = terrain_manager.myInfo.get_j_index(transform.position.z);
        //float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
        //float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

        //Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z), Color.white, 1f);
        
        //Vector3 relVect = my_goal_object.transform.position - transform.position;
        //m_Drone.Move_vect(relVect);
    }


    public void CollisionAvoidance(List<GameObject> agents, List<Vector3> obstacles)
    {
        // Collision avoidance using Hybrid Reciprocal Velocity Obstacles (HRVO).
        // Since the HRVO algorithm is run only on this agent, agent_i corresponds to only this agent.
        // Source: https://ieeexplore.ieee.org/abstract/document/5746538.
        // Inputs:
        // - agents: list of robots to avoid collision with that is assumed to follow the same HRVO collision avoidance procedure.
        // - obstacles: list of other obstacles to avoid collision with.

        bool vo;                                                                            // Velocity Obstacle (VO) object.
        bool rvo;                                                                           // Reciprocal Velocity Obstacle (RVO) object.
        bool hrvo;                                                                          // Hybrid Reciprocal Velocity Obstacle (HRVO) object.
        List<bool> agent_i_hrvo_list;                                                       // List of HRVO*_Ai|Aj objects for agent_i (Ai) with every other agent_j (Aj).
        List<bool> agent_i_vo_list;                                                         // List of VO*_Ai|Oj objects for agent_i (Ai) with every other obstacle_j (Oj).
        Vector3 cl;                                                                         // Center line of a HRVO.
        Vector3 agent_i_pos;                                                                // Current position of agent_i.
        Vector3 agent_i_v;                                                                  // Current velocity of agent_i.
        Vector3 agent_i_v_pref;                                                             // Preferred velocity of agent_i.
        Vector3 agent_i_v_new;                                                              // New velocity of agent_i.
        Vector3 agent_j_pos;                                                                // Current position of agent_j.
        Vector3 agent_j_v;                                                                  // Current velocity of agent_j.

        agent_i_pos = this.transform.position;
        agent_i_v = my_rigidbody.velocity;
        agent_i_hrvo_list = new List<bool>();
        agent_i_vo_list = new List<bool>();

        foreach (GameObject agent_j in agents)                                                                  // Loop over each agent_j.
        {
            if (agent_i == agent_j)
            {
                break;                                                                                          // Skip if agent_j is agent_i.
            }
            agent_j_pos = agent_j.transform.position;
            agent_j_v = agent_j.GetComponent<Rigidbody>().velocity;

            vo = VelocityObstacle(agent_i_pos, agent_i_v, agent_j_pos, agent_j_v);                              // Construct VO_Ai|Aj.
            (rvo, cl) = ReciprocalVelocityObstacle(agent_i_pos, agent_i_v, agent_j_pos, agent_j_v);             // Construct RVO_Ai|Aj.

            if (CheckToRight(agent_i_v, cl))                                                                    // TODO: What is v_a? Is it v_a_i or v_a_j?
            {
                hrvo = HybridReciprocalVelocityObstacle("left", vo, rvo);                                       // Construct HRVO_Ai|Aj using left edge of VO_Ai|Aj.
            }
            else
            {
                hrvo = HybridReciprocalVelocityObstacle("right", vo, rvo);                                      // Construct HRVO_Ai|Aj using right edge of VO_Ai|Aj.
            }

            hrvo = HybridReciprocalVelocityObstacleStar(hrvo);                                                  // Expand HRVO_Ai|Aj to HRVO*_Ai|Aj.
            agent_i_hrvo_list.Add(hrvo);                                                                        // Save the HRVO*_Ai|Aj in a list for this agent_i. 
        }

        foreach (Vector3 obstacle_j_pos in obstacles)                                                           // Loop over each obstacle position.
        {
            Vector3 obstacle_j_vel = new Vector3(0, 0, 0);                                                      // We only have static obstacles, i.e velocity zero.
            vo = VelocityObstacle(agent_i_pos, agent_i_v, obstacle_j_pos, obstacle_j_vel);                      // Construct VO_Ai|Oj.
            vo = VelocityObstacleStar(vo);                                                                      // Construct VO*_Ai|Oj.
            agent_i_vo_list.Add(vo);                                                                            // Save the VO*_Ai|Oj in a list for this agent_i. 
        }

        hrvo = false;  // Use agent_i_hrvo_list and agent_i_vo_list.                                            // Construct HRVO*_Ai from every HRVO*_Ai|Aj and VO*_Ai|Oj.
        agent_i_v_pref = GetPreferredVelocity();                                                                // Get the preferred velocity if no other agents would exist.
        agent_i_v_new = GetNewVelocity();                                                                       // Velocity that is closest to the pref. velocity but without collisions.

        return agent_i_v_new;
    }


    public bool VelocityObstacle(Vector3 agent1_pos, Vector3 agent1_vel, Vector3 agent2_pos, Vector3 agent2_vel)
    {
        return false;
    }


    public bool VelocityObstacleStar(bool vo)
    {
        return false;
    }


    public (bool, Vector3) ReciprocalVelocityObstacle(Vector3 agent1_pos, Vector3 agent1_vel, Vector3 agent2_pos, Vector3 agent2_vel)
    {
        return (false, new Vector3(0, 0, 0));
    }


    public bool HybridReciprocalVelocityObstacle(string side, bool vo, bool rvo)
    {
        return false;
    }


    public bool HybridReciprocalVelocityObstacleStar(bool hrvo)
    {
        return false;
    }


    public Vector3 GetPreferredVelocity()
    {
        // Returns the preferred velocity for this agent if no other agents would exist.
        // Output: velocity vector.
        Vector3 current_pos = this.transform.position;
        Vector3 target_pos = new Vector3(0, 0, 0);                                                                  // TODO: Get this from A*.
        return desired_speed * ((current_pos - target_pos) / (current_pos - target_pos).sqrMagnitude);              // TODO: Do they mean squared in the paper? (they write sub 2)
    }


    public Vector3 GetNewVelocity(bool hrvo, Vector3 v_pref)
    {
        // Chooses a new velocity to compute a trajectory toward the goal without collisions with any other agent.
        // Input: HRVO object, preferred velocity vector.
        // Output: velocity vector.
        Vector3 v_new;
        float min_diff = float.MaxValue;
        float diff;
        List<Vector3> permissible_new_velocities = new List<Vector3>();                                             // TODO: Get this from the HRVO object somehow.
        foreach (Vector3 v in permissible_new_velocities)
        {
            diff = (v - v_pref).sqrMagnitude;                                                                       // TODO: Do they mean squared in the paper? (they write sub 2)
            if (diff < min_diff)
            {
                v_new = v;
                min_diff = diff;
            }
        }
        return v_new;
    }


    public bool CheckToRight(Vector3 this_vector, Vector3 other_vector)
    {
        // Checks if this_vector is to the right of other_vector.
        // Output: true if to the right, false if to the left.
        if (Vector3.Dot(other_vector, this_vector) > 0)                                                             // TODO: Is this correct or should it be the other way around?
        {
            return true;
        }
        else
        {
            return false;
        }
    }


    // Update is called once per frame
    void Update()
    {
        
    }
}
