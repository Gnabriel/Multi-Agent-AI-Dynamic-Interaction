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


        // Plan your path here
        // ...
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


    public Vector3 InterceptTarget(GameObject agent, GameObject target)
    {
        Vector3 target_velocity = target.GetComponent<Rigidbody>().velocity;
        Vector3 target_direction = (target.transform.position - agent.transform.position).normalized;
        return ball.transform.position;     // Temporary
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
            if (friend != transform.gameObject)                                                             // TODO: is this check valid? or check pos instead?
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
        // ----- Debugging -----
        if (TESTING)
        {
            if (my_forward_score >= best_forward_score && my_goalkeeper_score <= best_goalkeeper_score)
            {
                currently_fw = true;
            }
            else
            {
                currently_fw = false;
            }
        }
        // ----- /Debugging -----

        // Check if this agent is the best chaser but at the same time not the best goalkeeper.
        if (my_forward_score >= best_forward_score && my_goalkeeper_score <= best_goalkeeper_score)          // TODO: is this right?
        {
            return true;
        }
        return false;
    }


    [Task]
    bool IsBallCloserThan(float distance)
    {
        // Checks if the ball is closer than a certain distance to the agent.
        return ((ball.transform.position - transform.position).sqrMagnitude < (distance * distance));
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
    void Defend(float what_to_do_with_this)
    {
        GoToPosition(own_goal.transform.position);
    }


    [Task]
    void InterceptBall()
    {
        Vector3 target_position = InterceptTarget(ball, transform.gameObject);
        GoToPosition(target_position);
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

        Debug.DrawLine(transform.position, ball.transform.position, Color.black);
        Debug.DrawLine(transform.position, own_goal.transform.position, Color.green);
        Debug.DrawLine(transform.position, other_goal.transform.position, Color.yellow);
        Debug.DrawLine(transform.position, friends[0].transform.position, Color.cyan);
        Debug.DrawLine(transform.position, enemies[0].transform.position, Color.magenta);

        if (CanKick())
        {
            Debug.DrawLine(transform.position, ball.transform.position, Color.red);
            //KickBall(maxKickSpeed * Vector3.forward);
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
        }
    }
}

