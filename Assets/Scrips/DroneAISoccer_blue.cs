using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Panda;
using UnityEditor;



[RequireComponent(typeof(DroneController))]
public class DroneAISoccer_blue : MonoBehaviour
{
    // ----- Debugging -----
    bool TESTING = true;
    bool currently_gk;
    bool currently_fw;
    // ----- /Debugging -----

    // ----- Misc -----
    int goal_resolution = 10;
    Vector3[] other_goal_positions;
    // ----- /Misc -----

    private DroneController m_Drone; // the drone controller we want to use

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    public GameObject[] friends;
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
        myPandaBT = GetComponent<PandaBehaviour>();

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
            other_goal_positions[i] = Vector3.Lerp(left_goal_post, right_goal_post, (float) i /goal_resolution);
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
            print("ball was kicked ");
        }
    }


    public void GoToPosition(Vector3 target_position)
    {
        // Moves agent toward desired position.
        m_Drone.Move_vect(target_position - transform.position);
    }


    // Returns a speed
    public Vector3 InterceptTarget(GameObject agent, GameObject target)
    {
        Vector3 target_position = target.transform.position;
        Vector3 target_velocity = target.GetComponent<Rigidbody>().velocity;

        Vector3 our_position = agent.transform.position;
        Vector3 our_velocity = agent.GetComponent<Rigidbody>().velocity;

        if (target_velocity.magnitude < 2 ) {
            return target_position - our_position ;
        }

        Vector3 bearing = (our_position - target_position).normalized;

        Vector3 new_point = (target_position + target_velocity);

        Vector3 delta = new_point - our_position;
        // int i = 0;
        // while (delta.magnitude > max_speed ) { // or difference between delta and our velocity is too big ( should be checked alongside the magnitude tho)
        //     i = i + 1;
        //     new_point = new_point + bearing;
        //     delta = new_point - our_position;
        //     if (i == 15) {
        //         Vector3 right = new Vector3(bearing.z, 0.0f, - bearing.x );
        //         Vector3 left  = new Vector3(- bearing.z, 0.0f, bearing.x);
        //         if ((target_position - (our_position + right)).magnitude > (target_position - (our_position + right)).magnitude) {
        //             delta = left * max_speed;
        //         } else {
        //             delta = right * max_speed;
        //         }
        //     }
        // }

        return delta;
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

    void Move_with_speed(Vector3 speed) {
        Rigidbody my_rigidbody = GetComponent<Rigidbody>();
        Vector3 acceleration = speed - my_rigidbody.velocity;

        m_Drone.Move_vect(acceleration);
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
            currently_gk = true;            // Used for debugging.
            return true;
        }
        else if (my_goalkeeper_score == best_goalkeeper_score)
        {
            if (Array.IndexOf(friends, transform.gameObject) < best_goalkeeper_index)                   // If they are equally suited for goalkeeper, pick first one in friends list.
            {
                currently_gk = true;        // Used for debugging.
                return true;
            }
        }
        currently_gk = false;               // Used for debugging.
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
            currently_fw = true;            // Used for debugging.
            return true;
        }
        currently_fw = false;               // Used for debugging.
        return false;
    }


    [Task]
    bool IsBallCloserThan(float distance)
    {
        // Checks if the ball is closer than a certain distance to the agent.
        if ((transform.position - ball.transform.position).magnitude < distance)
        {
            Debug.Log("Ball is within kicking distance.");
            return true;
        }
        return false;
        //return ((ball.transform.position - transform.position).sqrMagnitude < (distance * distance));
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


    //[Task]
    //bool ShootingOpportunity()
    //{
    //    // Checks if an agent that already has the ball also has the opportunity to score.
    //    RaycastHit hit;
    //    foreach (Vector3 goal_position in other_goal_positions)
    //    {
    //        Physics.Raycast(ball.transform.position, goal_position, out hit);
    //        if (hit.transform.gameObject.tag != "Drone")
    //        {
    //            return true;
    //        }
    //    }
    //    return false;
    //}


    [Task]
    void Defend()
    {
        GoToPosition(own_goal.transform.position);
    }


    [Task]
    void InterceptBall()
    {
        Vector3 target_speed = InterceptTarget( gameObject, ball);
        Move_with_speed(target_speed);
    }


    [Task]
    bool ShootBall()
    {
        Vector3 goal_direction;
        RaycastHit hit;
        foreach (Vector3 goal_position in other_goal_positions)
        {
            goal_direction = (goal_position - ball.transform.position).normalized;
            Physics.Raycast(ball.transform.position, goal_direction, out hit);

            //if (hit.transform.gameObject.name == "Red_goal" || hit.transform.gameObject.name == "Blue_goal")
            if (hit.transform.gameObject.name != "DroneSoccerCapsule(Clone)")
            {
                Debug.DrawLine(ball.transform.position, hit.transform.position, Color.green, 1f);
                if (CanKick())
                {
                    KickBall((maxKickSpeed / 2) * goal_direction);                                            // TODO: Change to not always using maxKickSpeed/2.
                    return true;
                }
            }
            Debug.DrawLine(ball.transform.position, hit.transform.position, Color.red, 1f);
        }
        return false;
    }


    [Task]
    void Dribble()
    {
    }


    [Task]
    void GoCenter()
    {
        // Moves this agent toward the center of the field lengtwise.
        Vector3 center = Vector3.Lerp(own_goal.transform.position, other_goal.transform.position, 0.5f);        // TODO: does this return the center point correctly?
        GoToPosition(center);                                                                   // TODO: Change this to add margin and/or make it dynamic somehow.
    }


    [Task]
    void GoFishing()
    {
        // Moves this agent toward the opposing team's goal.
        GoToPosition(other_goal.transform.position);                                            // TODO: Change this to add margin and/or make it dynamic somehow.
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
            ShootBall();
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
            if (currently_gk)
            {
                Handles.Label(transform.position, "Goalkeeper");
            }
            else if (currently_fw)
            {
                Handles.Label(transform.position, "Forward");
            }
            else
            {
                Handles.Label(transform.position, "Midfielder");
            }

            foreach (Vector3 goal_pos in other_goal_positions)
            {
                Handles.Label(goal_pos, "X");
            }
        }
    }
}

