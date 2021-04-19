using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    // ----- Debugging -----
    private bool ENABLE_DEBUG = false;
    // ----- /Debugging -----

    // ----- Misc -----
    public float desired_speed = 1.0f;          // TODO: Set speed.                 // Desired constant speed of the agents.
    public float sensor_length = 10f;           // TODO: Play with value.           // Length of the obstacle detecting sensors around an agent.
    public int sensor_resolution = 30;          // TODO: Play with value.           // Angle resolution in degrees of the obstacle detecting sensors around an agent.
    public float vo_length = 30f;
    public Map map;
    private const float grid_size = 7.5f;
    List<Vector3> my_path;
    public GameObject[] friends;
    public float drone_radius = 1.0f;
    // ----- /Misc -----

    // ----- Unity objects -----
    private DroneController m_Drone;
    Rigidbody my_rigidbody;
    public GameObject my_goal_object;
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    // ----- /Unity objects -----

    // ----- PD controller -----
    public Vector3 target_velocity;
    public Vector3 old_target_pos;
    public Vector3 desired_velocity;
    public float k_p = 2f;                                                          // TODO: Current k_p and k_d are for the car. Probably needs to be updated.
    public float k_d = 0.5f;
    // ----- /PD controller -----


    private void Start()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
        my_rigidbody = GetComponent<Rigidbody>();

        friends = GameObject.FindGameObjectsWithTag("Drone");
        
        map = new Map(terrain_manager, grid_size);
        my_path = A_star(transform.position, my_goal_object.transform.position);

        Vector3 old_wp = transform.position;
        foreach (var wp in my_path)
        {
            Debug.DrawLine(old_wp, wp, Color.red, 100f);
            old_wp = wp;
        }

        
    }


    private void FixedUpdate()
    {
        List<Vector3> obstacles = ObstacleSensor();

        // Execute your path here
        Vector3 destination = my_path[0];

        Vector3 relVect = destination - transform.position;

        if (relVect.magnitude < 9 & my_path.Count != 1) {
            // RaycastHit info;
            // destination = my_path[1];
            // relVect = destination - transform.position;
            // if (Physics.Raycast(transform.position, relVect, out info, relVect.magnitude)) {
            my_path.RemoveAt(0);
            destination = my_path[0];
            relVect = destination - transform.position;
            // } else {
            // }
        }
        Vector3 new_vel = CollisionAvoidance(friends, obstacles, destination);
        m_Drone.Move_vect(new_vel);

        // ----- PD controller -----
        // Keep track of target position and velocity.
        // target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
        // old_target_pos = target_position;

        // // A PD-controller to get desired velocity.
        // Vector3 position_error = target_position - transform.position;
        // Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
        // Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

        // float steering = Vector3.Dot(desired_acceleration, transform.right);
        // float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

        //Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
        //Debug.DrawLine(transform.position, transform.position + my_rigidbody.velocity, Color.blue);
        //Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

        //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
        //m_Drone.Move(steering, acceleration, acceleration, 0f);
        // ----- /PD controller -----
    }


    public Vector3 CollisionAvoidance(List<GameObject> agents, List<Vector3> obstacles, Vector3 destination)
    {
        // Collision avoidance using Hybrid Reciprocal Velocity Obstacles (HRVO).
        // Since the HRVO algorithm is run only on this agent, agent_i corresponds to only this agent.
        // Source: https://ieeexplore.ieee.org/abstract/document/5746538.
        // Inputs:
        // - agents: list of robots to avoid collision with that is assumed to follow the same HRVO collision avoidance procedure.
        // - obstacles: list of other obstacles to avoid collision with.

        VO vo;                                                                            // Velocity Obstacle (VO) object.
        RVO rvo;                                                                           // Reciprocal Velocity Obstacle (RVO) object.
        VO hrvo;                                                                          // Hybrid Reciprocal Velocity Obstacle (HRVO) object.
        List<VO> agent_i_hrvo_list;                                                       // List of HRVO*_Ai|Aj objects for agent_i (Ai) with every other agent_j (Aj).
        List<VO> agent_i_vo_list;                                                         // List of VO*_Ai|Oj objects for agent_i (Ai) with every other obstacle_j (Oj).
        Vector3 agent_i_pos;                                                                // Current position of agent_i.
        Vector3 agent_i_v;                                                                  // Current velocity of agent_i.
        Vector3 agent_i_v_pref;                                                             // Preferred velocity of agent_i.
        Vector3 agent_i_v_new;                                                              // New velocity of agent_i.
        Vector3 agent_j_pos;                                                                // Current position of agent_j.
        Vector3 agent_j_v;                                                                  // Current velocity of agent_j.

        agent_i_pos = this.transform.position;
        agent_i_v = my_rigidbody.velocity;
        agent_i_hrvo_list = new List<VO>();
        agent_i_vo_list = new List<VO>();

        foreach (GameObject agent_j in agents)                                                                  // Loop over each agent_j.
        {
            if (this == agent_j)
            {
                break;                                                                                          // Skip if agent_j is agent_i.
            }
            agent_j_pos = agent_j.transform.position;
            agent_j_v = agent_j.GetComponent<Rigidbody>().velocity;

            vo = VelocityObstacle(agent_i_pos, agent_i_v, agent_j_pos, agent_j_v);                              // Construct VO_Ai|Aj.
            rvo = ReciprocalVelocityObstacle(agent_i_pos, agent_i_v, agent_j_pos, agent_j_v);             // Construct RVO_Ai|Aj.
            hrvo = HybridReciprocalVelocityObstacle(agent_i_v, vo, rvo);                                       // Construct HRVO_Ai|Aj using left edge of VO_Ai|Aj.
            
            agent_i_hrvo_list.Add(hrvo);                                                                        // Save the HRVO*_Ai|Aj in a list for this agent_i. 
        }

        foreach (Vector3 obstacle_j_pos in obstacles)                                                           // Loop over each obstacle position.
        {
            Vector3 obstacle_j_vel = new Vector3(0, 0, 0);                                                      // We only have static obstacles, i.e velocity zero.
            vo = VelocityObstacle(agent_i_pos, agent_i_v, obstacle_j_pos, obstacle_j_vel);                      // Construct VO_Ai|Oj.
            vo = VelocityObstacleStar(vo);                                                                      // Construct VO*_Ai|Oj.
            agent_i_vo_list.Add(vo);                                                                            // Save the VO*_Ai|Oj in a list for this agent_i. 
        }

        agent_i_v_pref = GetPreferredVelocity();                                                                // Get the preferred velocity if no other agents would exist.
        agent_i_v_new = GetNewVelocity(agent_i_hrvo_list, agent_i_vo_list, agent_i_v_pref);                                                   // Velocity that is closest to the pref. velocity but without collisions.

        return agent_i_v_new;
    }


    public class VO {
        public float distance;
        public float left_boundary;
        public float right_boundary;
        public Vector3 origin;
        public VO(float d, float left, float right, Vector3 o) {
            distance = d;
            left_boundary = left;
            right_boundary = right;
            origin = o;
        }
    }
    VO VelocityObstacle(Vector3 agent_i_pos, Vector3 agent_i_v, Vector3 agent_j_pos, Vector3 agent_j_v) {
        float distance = (agent_j_pos - agent_i_pos).magnitude;

        Vector3 origin = agent_i_pos + agent_j_v;

        float angle_from_j_to_i= (float) Math.Atan2(agent_j_pos.z - agent_i_pos.z, agent_j_pos.x - agent_i_pos.x);

        float cone_angle = (float) Math.Asin((2 * drone_radius) / distance);

        float left_boundary = angle_from_j_to_i + cone_angle; // maybe over 2

        float right_boundary = angle_from_j_to_i - cone_angle;  

        return new VO(distance, left_boundary, right_boundary, origin);
    }

    public class RVO {
        public float distance;
        public float left_boundary;
        public float right_boundary;
        public float center_line;
        public Vector3 origin;
        public RVO(float d, float left, float right, float cl, Vector3 o) {
            distance = d;
            left_boundary = left;
            right_boundary = right;
            center_line = cl;
            origin = o;
        }
    }

    RVO ReciprocalVelocityObstacle(Vector3 agent_i_pos, Vector3 agent_i_v, Vector3 agent_j_pos, Vector3 agent_j_v) {
        float distance = (agent_j_pos - agent_i_pos).magnitude;

        Vector3 origin = agent_i_pos + 0.5f * (agent_i_v + agent_j_v);

        float angle_from_j_to_i= (float) Math.Atan2(agent_j_pos.z - agent_i_pos.z, agent_j_pos.x - agent_i_pos.x);

        float cone_angle = (float) Math.Asin((2 * drone_radius) / distance);

        float left_boundary = angle_from_j_to_i + cone_angle;

        float right_boundary = angle_from_j_to_i - cone_angle;        

        return new RVO(distance, left_boundary, right_boundary, angle_from_j_to_i, origin);
    }

    VO HybridReciprocalVelocityObstacle(Vector3 agent_i_v, VO vito, RVO rvito) {
        float velocity_angle = (float) Math.Atan2(agent_i_v.z - rvito.origin.z, agent_i_v.x - rvito.origin.x); 
        Vector3 new_origin;
        if (velocity_angle > rvito.center_line) {
            //left
            Vector3 other_point = new Vector3( (float) Math.Cos(rvito.left_boundary), 0.0f, (float) Math.Sin(rvito.left_boundary));

            float m = other_point.z / other_point.x;
            float b = rvito.origin.z - m * rvito.origin.x;

            Vector3 other_point_2 = new Vector3( (float) Math.Cos(vito.right_boundary), 0.0f, (float) Math.Sin(vito.right_boundary));

            float m_2 = other_point_2.z / other_point_2.x;
            float b_2 = vito.origin.z - m_2 * vito.origin.x;

            float x = (b_2 - b) / (m - m_2);
            float z = m*x + b;

            new_origin = new Vector3(x, 0.0f, z);
        } else {
            //right
            Vector3 other_point = new Vector3( (float) Math.Cos(rvito.right_boundary), 0.0f, (float) Math.Sin(rvito.right_boundary));

            float m = other_point.z / other_point.x;
            float b = rvito.origin.z - m * rvito.origin.x;

            Vector3 other_point_2 = new Vector3( (float) Math.Cos(vito.left_boundary), 0.0f, (float) Math.Sin(vito.left_boundary));

            float m_2 = other_point_2.z / other_point_2.x;
            float b_2 = vito.origin.z - m_2 * vito.origin.x;

            float x = (b_2 - b) / (m - m_2);
            float z = m*x + b;

            new_origin = new Vector3(x, 0.0f, z);
        }

        return new VO(rvito.distance, rvito.left_boundary, rvito.right_boundary, new_origin);
    }


    bool contained_in_vo(Vector3 point, List<VO> hrvo_list, List<VO> vo_list) {
        foreach (VO hrvito in hrvo_list) {
            Vector3 new_point = point - hrvito.origin;
            if (new_point.magnitude > vo_length) {
                continue;
            }
            if (contained_in_angle(hrvito.left_boundary, hrvito.right_boundary, (float) Math.Atan2(new_point.z, new_point.x))) {
                return true;
            }
        }
        foreach (VO vito in vo_list) {
            Vector3 new_point = point - vito.origin;
            if (new_point.magnitude > vo_length) {
                continue;
            }
            if (contained_in_angle(vito.left_boundary, vito.right_boundary, (float) Math.Atan2(new_point.z, new_point.x))) {
                return true;
            }
        }
        return false;
    }


    // Angles in radians
    bool contained_in_angle(float left_boundary, float right_boundary, float angle) {
        
        //less than 2 quadrants away
        if ( Math.Abs(right_boundary - left_boundary) <= Math.PI) {
            return right_boundary <= angle & left_boundary >= angle;
        }

        
        if (angle < 0.0f) {
            angle = angle + (float) ( 2.0 * Math.PI);
        }

        if ((left_boundary < 0.0f) & (right_boundary > 0.0f)) {
            left_boundary = left_boundary + (float) ( 2.0 * Math.PI);

            return right_boundary <= angle & left_boundary >= angle;
        }

        return right_boundary >= angle & left_boundary <= angle;

    }


    public Vector3 GetPreferredVelocity()
    {
        // Returns the preferred velocity for this agent if no other agents would exist.
        // Output: velocity vector.
        Vector3 current_pos = this.transform.position;
        Vector3 target_pos = new Vector3(0, 0, 0);                                                                  // TODO: Get this from A*.
        return desired_speed * ((current_pos - target_pos) / (current_pos - target_pos).sqrMagnitude);              // TODO: Do they mean squared in the paper? (they write sub 2)
    }


    public List<Vector3> GetPermissibleNewVelocities(List<VO> hrvo_list, List<VO> vo_list, Vector3 v_pref)
    {
        // Helper function to GetNewVelocity(). Computes possible new velocities. Substitute for the ClearPath algorithm.
        // Output: list of velocity vectors.
        Vector3 my_position = this.transform.position;
        Vector3 permissible_velocity;
        List<Vector3> new_velocities;
        List<Vector3> permissible_velocities;
        List<VO> vo_combined_list = hrvo_list.Concat(vo_list);
        VO vo1;
        VO vo2;
        float vo1_line1_m; float vo1_line1_b;                                                                  // Left line spanning Velocity Obstacle 1.
        float vo1_line2_m; float vo1_line2_b;                                                                  // Right line spanning Velocity Obstacle 1.
        float vo2_line1_m; float vo2_line1_b;                                                                  // Left line spanning Velocity Obstacle 2.
        float vo2_line2_m; float vo2_line2_b;                                                                  // Right line spanning Velocity Obstacle 2.
        for (int i = 0; i < vo_combined_list.Count; i++)
        {
            vo1 = vo_combined_list[i];
            (vo1_line1_m, vo1_line1_b) = linefromangle(vo1.origin, vo1.left_boundary);                              // Get left line of this velocity obstacle.
            (vo1_line2_m, vo1_line2_b) = linefromangle(vo1.origin, vo1.right_boundary);                             // Get right line of this velocity obstacle.
            new_velocities = new List<Vector3>();
            for (int j = 0; j < vo_combined_list.Count; j++)
            {
                if (i != j)
                {
                    vo2 = vo_combined_list[j];
                    (vo2_line1_m, vo2_line1_b) = linefromangle(vo2.origin, vo2.left_boundary);                      // Get left line of other velocity obstacle.
                    (vo2_line2_m, vo2_line2_b) = linefromangle(vo2.origin, vo2.right_boundary);                     // Get right line of other velocity obstacle.

                    // Intersections of VO1's left line and VO2.
                    new_velocities.Add(intersection(vo1_line1_m, vo1_line1_b, vo2_line1_m, vo2_line1_b) - my_position);     // TODO: Subtract my_position or not?
                    new_velocities.Add(intersection(vo1_line1_m, vo1_line1_b, vo2_line2_m, vo2_line2_b) - my_position);

                    // Intersections of VO1's right line and VO2.
                    new_velocities.Add(intersection(vo1_line2_m, vo1_line2_b, vo2_line1_m, vo2_line1_b) - my_position);
                    new_velocities.Add(intersection(vo1_line2_m, vo1_line2_b, vo2_line2_m, vo2_line2_b) - my_position);
                }
            }
            // Add projections of the preferred velocity vector onto VO1's left and right line.
            new_velocities.Add(Vector3.Project(v_pref, new Vector3(1f, 0f, vo1_line1_m)) + my_position);
            new_velocities.Add(Vector3.Project(v_pref, new Vector3(1f, 0f, vo1_line2_m)) + my_position);

            foreach (Vector3 velocity in new_velocities)
            {
                if (!contained_in_vo(velocity, hrvo_list, vo_list))                                             // Check that the velocities are permissible, ie. not inside a VO.
                {
                    permissible_velocities.Add(velocity);
                }
            }
        }
        return permissible_velocities;
    }


    public Vector3 GetNewVelocity(List<VO> hrvo_list, List<VO> vo_list, Vector3 v_pref)
    {
        // Chooses a new velocity to compute a trajectory toward the goal without collisions with any other agent.
        // Input: HRVO object, preferred velocity vector.
        // Output: velocity vector.
        Vector3 v_new = new Vector3(0, 0, 0);                                                                       // Default action is to stay still.         TODO: Change this?
        float min_diff = float.MaxValue;
        float diff;
        List<Vector3> permissible_new_velocities = GetPermissibleNewVelocities(hrvo_list, vo_list, v_pref);
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


    public List<Vector3> ObstacleSensorOld()
    {
        // Checks for obstacles around this agent given a sensor length and resolution angle (initialized as instance variables).
        // Output: list of obstacle positions.
        //sensor_length = 30;
        //sensor_resolution = 20;
        Vector3 my_position = m_Drone.transform.position;
        Vector3 sensor_direction = my_position + new Vector3(0, 0, sensor_length);
        RaycastHit hit;
        List<Vector3> obstacles = new List<Vector3>();

        //my_position.z = 1.5f;                                                                                           // Raise the Raycast to not detect cars and only walls.
        //sensor_direction.z = 1.5f;
        for (int angle = 0; angle < 360; angle += sensor_resolution)
        {
            sensor_direction = Quaternion.Euler(0, angle, 0) * (sensor_direction - my_position) + my_position;
            //sensor_direction = Quaternion.Euler(0, angle, 0) * sensor_direction + my_position;
            if (Physics.Raycast(my_position, sensor_direction, out hit, sensor_length))
            {
                obstacles.Add(hit.point);
                Debug.DrawLine(my_position, hit.point, Color.red, 0);
            }
            else
            {
                Debug.DrawLine(my_position, sensor_direction, Color.green, 0);
            }
        }
        return obstacles;
    }


    public List<Vector3> ObstacleSensor()
    {
        // Efficiently checks for obstacles around this agent given a sensor length (initialized as instance variables).
        // Output: list of obstacle positions.
        Vector3 my_position = m_Drone.transform.position;
        List<Vector3> obstacles = new List<Vector3>();

        Collider[] hit_colliders = Physics.OverlapSphere(my_position, sensor_length);
        foreach (var hit_collider in hit_colliders)
        {
            MeshFilter mf = (MeshFilter) hit_collider.gameObject.GetComponent<MeshFilter>();
            if (mf)
            {
                Mesh mesh = mf.mesh;
                if (mesh.name == "Cube" || mesh.name == "Cube Instance")
                {
                    obstacles.Add(hit_collider.gameObject.transform.position);
                    if (ENABLE_DEBUG)
                    {
                        Debug.DrawLine(my_position, hit_collider.gameObject.transform.position, Color.red, 0);
                        Debug.Log("Wall detected, size: " + mesh.bounds.size);
                    }
                }
            }
        }
        return obstacles;
    }


    // Update is called once per frame
    void Update()
    {
        
    }
    /**
     * ///////////////
     * Auxiliary functions
     * ///////////////
     */

    public (float, float) linefromangle(Vector3 point, float angle) {
        float m = (float) (Math.Sin(angle) / Math.Cos(angle));
        float b = point.z - m * point.x;
        return (m,b);
    }

    public Vector3 intersection(float m_1, float b_1, float m_2, float b_2) {
        float x = (b_2 - b_1) / (m_1 - m_2);
        float z = m_1*x + b_1;

        return new Vector3(x, 0.0f, z);
    }

    /**
     * ///////////////
     * A STAR STUFF
     * ///////////////
     */

    public class Node : IComparable<Node>
    {
        public int[] pos_ij;
        public float[] pos_xz;
        public Node parent;
        public float path_cost;
        public float dist_target;
        public float f;

        public int CompareTo(Node other)
        {
            if (this.f < other.f)
            {
                return -1;
            }
            else if (this.f > other.f)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }


        public Node(int[] p_ij, float[] p_xz)
        {
            parent = null;
            pos_ij = p_ij;
            pos_xz = p_xz;
            dist_target = -1;
            path_cost = -1;
        }

    }

    private List<Vector3> A_star(Vector3 start_pos_xyz, Vector3 goal_pos_xyz, bool return_f = false)
    {

        int[] start_pos_ij = { terrain_manager.myInfo.get_i_index(start_pos_xyz[0]), terrain_manager.myInfo.get_j_index(start_pos_xyz[2]) };

        int[] goal_pos_ij = { terrain_manager.myInfo.get_i_index(goal_pos_xyz[0]), terrain_manager.myInfo.get_j_index(goal_pos_xyz[2]) };


        Node start = new Node(start_pos_ij, new float[] { start_pos_xyz[0], start_pos_xyz[2] });
        Node goal = new Node(goal_pos_ij, new float[] { goal_pos_xyz[0], goal_pos_xyz[2] });
        // todo add temporary values for goal


        // Add cost values for start node
        start.path_cost = 0;
        start.dist_target = compute_dist(start, goal);
        start.f = start.path_cost + start.dist_target;


        List<Node> path = new List<Node>();
        List<Node> open = new List<Node>();
        List<Node> closed = new List<Node>();
        Node current_node = start;

        // Add the first starting point to be examined
        open.Add(start);

        //if we still have nodes to check AND does not have our goal in the closed/checked list
        // Todo: add max iterations to while loop
        int iterations = 1000;
        while (open.Count != 0 || iterations >= 1)
        {
            //Debug.Log(iterations);
            // Find Node with smallest f value in open set and add to closed set
            open.Sort();
            current_node = open[0];
            open.Remove(current_node);
            closed.Add(current_node);

            // Check if current node is goal
            if (current_node.pos_ij[0] == goal.pos_ij[0] && current_node.pos_ij[1] == goal.pos_ij[1])
            {
                // Goal found
                // Change actual coordinates so that they are not in the center
                if (return_f == false)
                {
                    current_node.pos_xz = goal.pos_xz;
                    Debug.Log("Found Path to Goal");
                    //List<Vector3> final_path = ReconstructPath(current_node);
                    return ReconstructPath(current_node);
                }
                else
                {
                    List<Vector3> final_f = new List<Vector3>();
                    Vector3 f_score = new Vector3();
                    //nb.dist_target = compute_dist(nb, goal);
                    //nb.path_cost = nb.parent.path_cost + 1 + deceleration_cost; //path cost + potential deceleration cost
                    //nb.f = nb.path_cost + nb.dist_target;
                    f_score.x = current_node.parent.f + compute_dist(current_node, current_node.parent);

                    f_score.y = 0f;
                    f_score.z = 0f;
                    final_f.Add(f_score);
                    return final_f;
                }

            }
            else
            {
                // Update neighbours
                (List<Node> o, List<Node> c) = UpdateNeighbours(current_node, goal, open, closed);
                open = o;
                closed = c;
            }

            iterations -= iterations;
        }

        // Return no path found if open set empty OR max iterations reached
        Debug.Log("FOUND NO PATH");
        return new List<Vector3>();
    }

    private float compute_dist(Node n, Node g)
    {
        //Compute euc distance from current node to goal
        float di = n.pos_xz[0] - g.pos_xz[0];
        float dj = n.pos_xz[1] - g.pos_xz[1];
        float distance = (float)Math.Sqrt(di * di + dj * dj);
        return distance;
    }

    private bool check_safety(Node n, Node g)
    {
        //if (n.pos_ij[0] == g.pos_ij[0] && n.pos_ij[1] == g.pos_ij[1])
        //{
        //    //If the goal is too close to a border we swap it to the center of its current grid
        //    g.pos_xz[0] = terrain_manager.myInfo.get_x_pos(g.pos_ij[0]);
        //    g.pos_xz[1] = terrain_manager.myInfo.get_z_pos(g.pos_ij[1]);
        //    return false; // if this is a goal node we let it pass
        //}


        //checks if the surrounding area is safe to drive in by checking a safety margin in 8 directions
        float p_x = n.pos_xz[0];

        float p_z = n.pos_xz[1];

        float margin = 3f; //SANITY CHECK THIS with car/drone size?, should we map to X,Z since they will be different?
        List<(float, float)> combos = new List<(float, float)>();
        combos.Add((p_x + margin, p_z));
        combos.Add((p_x - margin, p_z));
        combos.Add((p_x, p_z + margin));
        combos.Add((p_x, p_z - margin));

        combos.Add((p_x + margin, p_z + margin));
        combos.Add((p_x - margin, p_z - margin));
        combos.Add((p_x - margin, p_z + margin));
        combos.Add((p_x + margin, p_z - margin));

        foreach ((float x, float z) in combos)
        {
            int ii = terrain_manager.myInfo.get_i_index(x);
            int jj = terrain_manager.myInfo.get_j_index(z);

            if (terrain_manager.myInfo.traversability[ii, jj] == 1)
            {
                return true; //too close
            }
        }

        return false;//if it passes all checks
    }

    private (List<Node>, List<Node>) UpdateNeighbours(Node current, Node goal, List<Node> open, List<Node> closed)
    {

        // Get all four neighbours and compute costs
        float step_size_ratio = 0.3f; //THIS MIGHT NEED TO BE dynamic w.r.t the map
        float step_x = step_size_ratio * ((terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / terrain_manager.myInfo.x_N);
        float step_z = step_size_ratio * ((terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N);

        //float x = terrain_manager.myInfo.x_low + step / 2 + step * i;
        List<(float, float)> n_index2 = new List<(float, float)>();
        float x_hor = current.pos_xz[0];
        float h_vert = current.pos_xz[1];
        n_index2.Add((x_hor + step_x, h_vert)); //(Z,X)
        n_index2.Add((x_hor - step_x, h_vert));
        n_index2.Add((x_hor, h_vert + step_z));
        n_index2.Add((x_hor, h_vert - step_z));

        //n_index2.Add((x_hor + step_x, h_vert + step_z)); //(Z,X)
        //n_index2.Add((x_hor - step_x, h_vert - step_z));
        //n_index2.Add((x_hor - step_x, h_vert + step_z));
        //n_index2.Add((x_hor + step_x, h_vert - step_z));

        List<Node> valid_neighbours = new List<Node>();

        float deceleration_cost = 0f;
        int counter = 0; //the first 4 elements will be straight = 0 dec_cost
        
        //System.Random random = new System.Random(); 
        // Create all four neighbours
        foreach ((float x, float z) in n_index2)
        {
            if (counter >= 4)
            {
                deceleration_cost = 1f;
            }
            else
            {
                counter += 1; //inc by 1
            }

            int[] grid = map.GetGridIndex(x, z);
            float collision_cost = Math.Abs(map.nodes[grid[0], grid[1]])*0;//Add cost for walking on collision turrets
            float same_path_cost = 0; //add cost to avoid taking exactly same path as other cars

        
            

            // foreach (GameObject car in friends)
            // {
            //     if (car.transform.position == transform.position)//dont wanna check our own car
            //         continue;
            //     List<Vector3> other_traj = car.GetComponent<CarAI5>().trajectory;
            //     if (other_traj.Count>0)
            //     {
            //         foreach (Vector3 point in other_traj)
            //         {
            //             int[] other_grid = map.GetGridIndex(point.x, point.z);
            //             if (grid[0] == other_grid[0] & grid[1] == other_grid[1])
            //             {
            //                 Debug.Log("Added same path cost");
            //                 same_path_cost = 30;
            //             }
            //         }
            //     }
            
            // }


            Node nb = new Node(new int[] { terrain_manager.myInfo.get_i_index(x), terrain_manager.myInfo.get_j_index(z) }, new float[] { x, z });
            nb.parent = current;
            nb.dist_target = compute_dist(nb, goal);
            nb.path_cost = nb.parent.path_cost + UnityEngine.Random.Range(1,3) + deceleration_cost + collision_cost + same_path_cost; //path cost + potential deceleration cost+collision 
            nb.f = nb.path_cost + nb.dist_target;
            valid_neighbours.Add(nb);
        }

        foreach (Node nb in valid_neighbours)
        {

            // Check if node is not in closed set
            if (!closed.Exists(x => (x.pos_xz[0] == nb.pos_xz[0]) && (x.pos_xz[1] == nb.pos_xz[1])))
            {
                // If neighbour is obstacle add it to closed set
                if (terrain_manager.myInfo.traversability[nb.pos_ij[0], nb.pos_ij[1]] == 1) //this is still checked in ij space
                {
                    closed.Add(nb);
                }
                //DO WE NEED TO ADD SPACE FOR COLLISION HERE?
                else if (check_safety(nb, goal))
                {
                    closed.Add(nb);// too close to edge
                }

                // Check if it is in open set
                else if (open.Exists(x => (x.pos_xz[0] == nb.pos_xz[0]) && (x.pos_xz[1] == nb.pos_xz[1])))
                {
                    foreach (Node x in open)
                    {
                        if ((x.pos_xz[0] == nb.pos_xz[0]) && (x.pos_xz[1] == nb.pos_xz[1]))
                        {
                            if (nb.path_cost < x.path_cost)
                            {
                                x.parent = nb.parent;
                                x.path_cost = nb.path_cost; //add a heuristic that likes to go straight 
                                x.dist_target = nb.dist_target;
                                x.f = nb.f;
                                x.pos_xz = nb.pos_xz;
                            }
                        }
                    }
                }

                // Otherwise add node to open set
                else
                {
                    open.Add(nb);
                }
            }
        }
        return (open, closed);
    }

    private List<Vector3> ReconstructPath(Node goal)
    {
        List<Vector3> path = new List<Vector3>();
        Node current_node = goal;

        // Backtrack path until start node is reached
        while (current_node.parent != null)
        {
            path.Add(new Vector3(current_node.pos_xz[0], 0f, current_node.pos_xz[1]));
            current_node = current_node.parent;
        }

        // Add the start node to path
        path.Add(new Vector3(current_node.pos_xz[0], 0f, current_node.pos_xz[1]));

        path.Reverse();
        return path;
    }

    private float[] get_xz(int i, int j)
    {
        float x_pos = terrain_manager.myInfo.get_x_pos(i);
        float z_pos = terrain_manager.myInfo.get_z_pos(j);
        return new float[] { x_pos, z_pos };

    }
}
