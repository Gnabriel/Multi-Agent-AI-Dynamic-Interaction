using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Panda;
using UnityEditor;
using System.Linq;



[RequireComponent(typeof(DroneController))]
public class DroneAISoccer_red : MonoBehaviour
{
    // ----- Debugging -----
    bool TESTING = true;
    // ----- /Debugging -----

    // ----- Misc -----
    public int goal_resolution = 20;
    public Vector3[] other_goal_positions;
    public int agent_index;                             // Index of this agent in the friends array.
    public static int current_goalkeeper;               // The agent_index of the agent that is currently goalkeeper.
    Rect field;
    // ----- /Misc -----

    // ----- PD controller -----
    public Vector3 old_target_pos;
    Rigidbody my_rigidbody;
    public float k_p = 2f;
    public float k_d = 1f;
    // ----- /PD controller -----

    private DroneController m_Drone; // the drone controller we want to use

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    public static GameObject[] friends;                 // Made this static.
    public string friend_tag;
    public GameObject[] enemies;
    public string enemy_tag;

    public GameObject own_goal;
    public GameObject other_goal;
    public GameObject ball;

    PandaBehaviour myPandaBT;

    public float dist;
    public float maxKickSpeed = 40f;
    public float lastKickTime = 0f;

    public float max_speed = 15f;
    public float max_acceleration = 15f;

    private void Start()
    {
        my_rigidbody = GetComponent<Rigidbody>();
        myPandaBT = GetComponent<PandaBehaviour>();
        field = Rect.MinMaxRect(55f, 52.5f, 245f, 147.5f);

        //myPandaBT.Compile("SoccerRed.BT.txt");

        // get the car controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


        // note that both arrays will have holes when objects are destroyed
        // but for initial planning they should work
        friend_tag = gameObject.tag;
        if (friend_tag == "Blue")
            enemy_tag = "Red";
        else
            enemy_tag = "Blue";

        friends = GameObject.FindGameObjectsWithTag(friend_tag);
        enemies = GameObject.FindGameObjectsWithTag(enemy_tag);

        ball = GameObject.FindGameObjectWithTag("Ball");

        agent_index = Array.IndexOf(friends, gameObject);

        // ----- Discretize goal -----
        other_goal_positions = new Vector3[goal_resolution + 1];
        Vector3 left_goal_post;
        Vector3 right_goal_post;

        if (friend_tag == "Red")
        {
            left_goal_post = new Vector3(60, 0, 85);
            right_goal_post = new Vector3(60, 0, 115);
        }
        else
        {
            left_goal_post = new Vector3(240, 0, 85);
            right_goal_post = new Vector3(240, 0, 115);
        }

        for (int i = 0; i <= goal_resolution; i++)
        {
            other_goal_positions[i] = Vector3.Lerp(left_goal_post, right_goal_post, (float)i / goal_resolution);
        }
        // ----- Discretize goal -----
    }


    private bool CanKick()
    {
        dist = (transform.position - ball.transform.position).magnitude;
        return dist < 7f && (Time.time - lastKickTime) > 0.5f;
    }


    private void KickBall(Vector3 velocity)
    {
        // impulse to ball object in direction away from agent
        if (CanKick())
        {
            velocity.y = 0f;
            Rigidbody rb = ball.GetComponent<Rigidbody>();
            rb.AddForce(velocity, ForceMode.VelocityChange);
            lastKickTime = Time.time;
            //print("ball was kicked ");
        }
    }


    public void GoToPosition(Vector3 target_position)
    {
        // Moves agent toward desired position with the help of Petter's PD controller.
        Vector3 target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
        old_target_pos = target_position;
        Vector3 position_error = target_position - transform.position;
        Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
        Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;
        m_Drone.Move_vect(desired_acceleration);
    }


    // Returns a speed
    public Vector3 InterceptTarget(GameObject agent, GameObject target)
    {
        Vector3 target_position = target.transform.position;
        Vector3 target_velocity = target.GetComponent<Rigidbody>().velocity;

        Vector3 our_position = agent.transform.position;
        Vector3 our_velocity = agent.GetComponent<Rigidbody>().velocity;

        Vector3 bearing = (our_position - target_position).normalized;

        Vector3 new_point = (target_position + target_velocity);

        Vector3 delta = new_point - our_position;
        int i = 0;
        while (delta.magnitude > max_speed)
        { // or difference between delta and our velocity is too big ( should be checked alongside the magnitude tho)
            i = i + 1;
            new_point = new_point + bearing;
            delta = new_point - our_position;
            if (i == 15)
            {
                Vector3 right = new Vector3(bearing.z, 0.0f, -bearing.x);
                Vector3 left = new Vector3(-bearing.z, 0.0f, bearing.x);
                if ((target_position - (our_position + right)).magnitude > (target_position - (our_position + right)).magnitude)
                {
                    delta = left * max_speed;
                }
                else
                {
                    delta = right * max_speed;
                }
            }
        }
        return delta;
    }


    public Vector3 GetShootDirection()                                                                                  // TODO: Add margin to goal post?
    {
        // Get the best direction in which the ball should travel in order to score a goal.
        Vector3 goal_direction;
        Vector3 shoot_direction = new Vector3(-999, -999, -999);                                    // Returned to recognize if no shoot direction was found.
        List<float> enemy_intercept_distances = new List<float>();
        float current_closest_enemy = 0;
        float max_closest_enemy = 0;
        RaycastHit hit;

        foreach (Vector3 goal_position in other_goal_positions)
        {
            goal_direction = goal_position - ball.transform.position;                                                   // TODO: Include wall bouncing directions here.
            
            Physics.Raycast(ball.transform.position, goal_direction, out hit);

            //if (hit.transform.gameObject.name != "DroneSoccerCapsule(Clone)")
            if (hit.transform.gameObject.name == "Red_goal" || hit.transform.gameObject.name == "Blue_goal")
            {
                // Check distances from each enemy to the goal direction.
                foreach (GameObject enemy in enemies)
                {
                    // Add the length of the projection of ball-to-enemy-vector onto ball-to-goal-vector.
                    enemy_intercept_distances.Add(Vector3.Project(enemy.transform.position - ball.transform.position, goal_direction).magnitude);
                }
                // Choose the goal direction with maximum distance to the closest enemy.
                current_closest_enemy = enemy_intercept_distances.Min();
                if (current_closest_enemy > max_closest_enemy)
                {
                    shoot_direction = goal_direction;
                    max_closest_enemy = current_closest_enemy;
                }
            }
        }

        //Debug.DrawLine(ball.transform.position, ball.transform.position + shoot_direction, Color.red, 5);

        return shoot_direction;
    }


    public float GetKickSpeed(Vector3 goal_direction)
    {
        Vector3 ball_velocity = ball.GetComponent<Rigidbody>().velocity;
        float kick_speed;
        float min_kick_speed = 20;                          // TODO: Play with this value!
        float max_kick_speed = maxKickSpeed - 1;            // -1 for error margin in the GetKickVelocity() computations.
        float max_goal_distance = 185;                      // Distance from opposing corner to goal.

        //float distance_factor = Math.Min(2 * goal_direction.magnitude / max_goal_distance, 1);
        float distance_factor = Math.Min(goal_direction.magnitude / max_goal_distance, 1);                  // Factor from 0->1 on how much force should be added.

        Vector3 ball_velocity_to_goal = Vector3.Project(ball_velocity, goal_direction);

        // Check direction of ball_velocity_to_goal relative to goal_direction.
        float velocity_factor;
        if (Vector3.Dot(goal_direction, ball_velocity_to_goal) < 0)                                         // If ball_velocity_to_goal is at opposite direction from goal_direction.
        {
            velocity_factor = Math.Min(ball_velocity_to_goal.magnitude, maxKickSpeed) / maxKickSpeed;       // Factor from 0->1 on how much force should be added.
        }
        else                                                                                                // If ball_velocity_to_goal is at same direction as goal_direction.
        {
            velocity_factor = -Math.Min(ball_velocity_to_goal.magnitude, maxKickSpeed) / maxKickSpeed;      // Factor from -1->0 on how much force should be reduced.
        }

        // ############################# (distance_factor + velocity_factor) kan bli negativt....... gör om till faktorer istället och multiplicera dem?

        //kick_speed = (distance_factor + velocity_factor) * max_kick_speed;

        kick_speed = Math.Max(distance_factor * max_kick_speed, min_kick_speed);

        kick_speed = max_kick_speed;

        Debug.Log("Kick speed: " + kick_speed);

        return kick_speed;

    }


    public Vector3 GetKickVelocity(Vector3 goal_direction)
    {
        // Compensates the kick direction to the ball's current velocity.
        Vector3 kick_velocity;
        Vector3 ball_velocity = ball.GetComponent<Rigidbody>().velocity;
        goal_direction.y = 0;
        ball_velocity.y = 0;
        //Vector3 orthogonal_vector = ball_velocity - Vector3.Project(ball_velocity, goal_direction);
        Vector3 orthogonal_vector = Vector3.Project(ball_velocity, goal_direction) - ball_velocity;

        float kick_speed = GetKickSpeed(goal_direction);

        float x_v = goal_direction.x;
        float z_v = goal_direction.z;
        float x_red = orthogonal_vector.x;
        float z_red = orthogonal_vector.z;

        float m = z_v / x_v;
        float a = 1 + m * m;
        float b = x_red + z_red;
        float c = -(kick_speed * kick_speed + x_red * x_red + z_red * z_red);

        float x_1 = (-b + (float)Math.Sqrt(b * b - 4 * a * c)) / (2 * a);
        float x_2 = (-b - (float)Math.Sqrt(b * b - 4 * a * c)) / (2 * a);

        //Debug.DrawLine(ball.transform.position, ball.transform.position + goal_direction*40, Color.magenta, 10);

        float z_1 = m * x_1;
        float z_2 = m * x_2;

        Vector3 kick_velocity_1 = orthogonal_vector + new Vector3(x_1, 0, z_1);
        Vector3 kick_velocity_2 = orthogonal_vector + new Vector3(x_2, 0, z_2);

        if (Vector3.Dot(kick_velocity_1, goal_direction) >= 0)
        {
            kick_velocity = kick_velocity_1;
        }
        else
        {
            kick_velocity = kick_velocity_2;
        }

        if (kick_velocity.magnitude > maxKickSpeed)
        {
            kick_velocity = kick_velocity.normalized * maxKickSpeed;
        }

        //Debug.DrawLine(ball.transform.position, ball.transform.position + ball_velocity, Color.green, 10);
        //Debug.DrawLine(ball.transform.position + ball_velocity, ball.transform.position + ball_velocity + orthogonal_vector, Color.red, 10);
        //Debug.DrawLine(ball.transform.position, ball.transform.position + new Vector3(x_2, 0, z_2), Color.blue, 10);
        //Debug.DrawLine(ball.transform.position, ball.transform.position + kick_velocity, Color.black, 10);

        return kick_velocity;
        //return (goal_direction + orthogonal_vector).normalized;
    }


    public GameObject GetOtherForward()
    {
        int i = 0;
        foreach (GameObject friend in friends)
        {
            // Skip this agent's own gameobject.
            if (i != agent_index)
            {
                // Check if the friend is not goalkeeper, ie. the other forward.
                if (current_goalkeeper != i)
                {
                    return friend;
                }
            }
            i++;
        }
        return gameObject;
    }


    public float GoalkeeperScore(GameObject agent)                                                          // TODO: Add more advanced scoring.
    {
        // Returns score of how this agent is suited to be goalkeeper.
        Vector3 own_goal_to_agent = agent.transform.position - own_goal.transform.position;
        Vector3 ball_to_agent = agent.transform.position - ball.transform.position;
        return -own_goal_to_agent.magnitude + ball_to_agent.magnitude;
    }


    public float ForwardScore(GameObject agent)                                                             // TODO: Add more advanced scoring.
    {
        // Returns score of how this agent is suited to be forward.
        Vector3 ball_to_agent = agent.transform.position - ball.transform.position;
        return -ball_to_agent.magnitude;
    }


    void Move_with_speed(Vector3 speed)
    {
        Vector3 acceleration = speed - my_rigidbody.velocity;

        m_Drone.Move_vect(acceleration);
    }


    public bool CorrectSideOfBall(GameObject agent)
    {
        if (friend_tag == "Red" && ball.transform.position.x - agent.transform.position.x < 0)
        {
            return true;
        }
        else if (friend_tag == "Blue" && ball.transform.position.x - agent.transform.position.x > 0)
        {
            return true;
        }
        return false;
    }


    [Task]
    bool IsGoalkeeper()
    {
        // Checks if this agent is currently best suited to be goalkeeper or not.
        float my_goalkeeper_score = GoalkeeperScore(transform.gameObject);
        float best_goalkeeper_score = float.MinValue;
        float friend_goalkeeper_score;
        int best_goalkeeper_index = -1;
        foreach (GameObject friend in friends)
        {
            if (friend != transform.gameObject)
            {
                friend_goalkeeper_score = GoalkeeperScore(friend);
                if (friend_goalkeeper_score > best_goalkeeper_score)
                {
                    best_goalkeeper_score = friend_goalkeeper_score;
                    best_goalkeeper_index = Array.IndexOf(friends, friend);
                }
            }
        }

        //Check if this agent is the best goalkeeper.
        if (my_goalkeeper_score > best_goalkeeper_score)
        {
            current_goalkeeper = agent_index;
            return true;
        }
        else if (my_goalkeeper_score == best_goalkeeper_score)
        {
            if (Array.IndexOf(friends, transform.gameObject) < best_goalkeeper_index)                   // If they are equally suited for goalkeeper, pick first one in friends list.
            {
                current_goalkeeper = agent_index;
                return true;
            }
        }
        return false;
    }


    [Task]
    bool IsForward()
    {
        // Checks if this agent is currently best suited to be forward or not.
        // (but at the same time not goalkeeper, i.e goalkeeper is prioritized).
        float my_forward_score = ForwardScore(transform.gameObject);
        float my_goalkeeper_score = GoalkeeperScore(transform.gameObject);
        float best_forward_score = float.MinValue;
        float best_goalkeeper_score = float.MinValue;
        float friend_forward_score;
        float friend_goalkeeper_score;
        foreach (GameObject friend in friends)
        {
            if (friend != transform.gameObject)
            {
                friend_forward_score = ForwardScore(friend);
                friend_goalkeeper_score = GoalkeeperScore(friend);
                if (friend_forward_score > best_forward_score)
                {
                    best_forward_score = friend_forward_score;
                }
                if (friend_goalkeeper_score > best_goalkeeper_score)
                {
                    best_goalkeeper_score = friend_goalkeeper_score;
                }
            }
        }

        // Check if this agent is the best chaser but at the same time not the best goalkeeper.
        if (my_forward_score >= best_forward_score && my_goalkeeper_score <= best_goalkeeper_score)
        {
            return true;
        }
        return false;
    }


    [Task]
    bool IsBallOutOfBounds()
    {
        // Checks if the ball is outside of the soccer field.
        if (field.Contains(new Vector2(ball.transform.position.x, ball.transform.position.z)))
        {
            return false;
        }
        Debug.Log("Ball is out of bounds!");
        return true;
    }


    [Task]
    bool IsBallCloserThan(float distance)
    {
        // Checks if the ball is closer than a certain distance to the agent.
        if ((transform.position - ball.transform.position).magnitude < distance)
        {
            return true;
        }
        return false;
        //return ((ball.transform.position - transform.position).sqrMagnitude < (distance * distance));
    }


    [Task]
    bool IsOtherForwardCloserThan(float distance)
    {
        // Checks if the other forward is within a certain distance to the ball.
        GameObject other_forward = GetOtherForward();
        if ((other_forward.transform.position - ball.transform.position).magnitude < distance)
        {
            return true;
        }
        return false;
    }


    [Task]
    bool IsReadyToShoot()
    {
        // Checks if the agent is ready to shoot, i.e close enough to the ball and at the correct side of the ball.
        if (IsBallCloserThan(7))
        {
            if (CorrectSideOfBall(gameObject))
            {
                return true;
            }
        }
        return false;
    }


    [Task]
    bool IsOtherForwardReadyToShoot()
    {
        // Checks if the other forward is ready to shoot, i.e close enough to the ball and at the correct side of the ball.
        if (IsOtherForwardCloserThan(7))
        {
            if (CorrectSideOfBall(GetOtherForward()))
            {
                return true;
            }
        }
        return false;
    }


    [Task]
    bool TeammatesHaveBall()
    {
        // Checks if the ball is currently held by one of this agent's teammates.
        foreach (GameObject friend in friends)
        {
            if ((transform.position - ball.transform.position).magnitude < 7f)                              // 7 units seems to be the distance needed to kick the ball.
            {
                return true;
            }
        }
        return false;
    }


    [Task]
    void Defend()
    {
        if (friend_tag == "Red")
        {
            GoToPosition(own_goal.transform.position - new Vector3(10, 0, 0));
        }
        else
        {
            GoToPosition(own_goal.transform.position + new Vector3(10, 0, 0));
        }
    }


    [Task]
    void InterceptBall()
    {
        //Vector3 target_speed = InterceptTarget(transform.gameObject, ball);
        //Move_with_speed(target_speed);
        if (friend_tag == "Red")
        {
            GoToPosition(ball.transform.position + new Vector3(3, 0, 0));
        }
        else
        {
            GoToPosition(ball.transform.position - new Vector3(3, 0, 0));
        }
        //GoToPosition(ball.transform.position);
    }


    [Task]
    bool ShootBall()
    {
        Vector3 shoot_direction = GetShootDirection();
        if (CanKick())
        {
            KickBall(GetKickVelocity(shoot_direction));
            return true;
        }
        return false;
    }


    [Task]
    bool ShootBallSynchronised()
    {
        return false;
    }


    [Task]
    void GoCenter()
    {
        // Moves this agent toward the center of the field lengtwise.
        Vector3 center = Vector3.Lerp(own_goal.transform.position, other_goal.transform.position, 0.5f);
        if (friend_tag == "Red")
        {
            GoToPosition(center + new Vector3(3, 0, 0));
        }
        else
        {
            GoToPosition(center + new Vector3(3, 0, 0));
        }
    }


    [Task]
    void GoFishing()
    {
        // Moves this agent toward the opposing team's goal.
        GoToPosition(other_goal.transform.position);
    }


    private void FixedUpdate()
    {


        // Execute your path here
        // ...

        Vector3 avg_pos = Vector3.zero;

        foreach (GameObject friend in friends)
        {
            avg_pos += friend.transform.position;
        }
        avg_pos = avg_pos / friends.Length;
        //Vector3 direction = (avg_pos - transform.position).normalized;
        Vector3 direction = (ball.transform.position - transform.position).normalized;



        // this is how you access information about the terrain
        int i = terrain_manager.myInfo.get_i_index(transform.position.x);
        int j = terrain_manager.myInfo.get_j_index(transform.position.z);
        float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
        float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

        //Debug.DrawLine(transform.position, ball.transform.position, Color.black);
        //Debug.DrawLine(transform.position, own_goal.transform.position, Color.green);
        //Debug.DrawLine(transform.position, other_goal.transform.position, Color.yellow);
        //Debug.DrawLine(transform.position, friends[0].transform.position, Color.cyan);
        //Debug.DrawLine(transform.position, enemies[0].transform.position, Color.magenta);

        if (CanKick())
        {
            //Debug.DrawLine(transform.position, ball.transform.position, Color.red);
            //KickBall(maxKickSpeed * Vector3.forward);
            //ShootBall();
        }



        // this is how you control the agent
        m_Drone.Move_vect(direction);

        // this is how you kick the ball (if close enough)
        // Note that the kick speed is added to the current speed of the ball (which might be non-zero)
        Vector3 kickDirection = (other_goal.transform.position - transform.position).normalized;

        // replace the human input below with some AI stuff
        if (Input.GetKeyDown("space"))
        {
            KickBall(maxKickSpeed * kickDirection);
        }
    }


    private void Update()
    {
        myPandaBT.Reset();
        myPandaBT.Tick();
    }


    public void OnDrawGizmos()
    {
        if (TESTING)
        {
            if (current_goalkeeper == agent_index)
            {
                Handles.Label(transform.position, "Goalkeeper");
            }
            else
            {
                Handles.Label(transform.position, "Forward");
            }

            //foreach (Vector3 goal_pos in other_goal_positions)
            //{
            //    Handles.Label(goal_pos, "X");
            //}
        }
    }
}

