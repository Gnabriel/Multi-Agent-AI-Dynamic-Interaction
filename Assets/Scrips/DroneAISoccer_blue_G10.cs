using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using Panda;

//[ExecuteInEditMode]
[ExecuteAlways]
[RequireComponent(typeof(DroneController))]
public class DroneAISoccer_blue_G10 : MonoBehaviour
{
    private string team = "Blue - ";
    private bool print = false;

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
    private Vector3 frontPost;
    private Vector3 backPost;
    private Vector3 oldTargetPos;

    PandaBehaviour myPandaBT;
    private Rigidbody my_rigidbody;

    // Fixe parameters
    private float ballDiameter = 4.3f;
    private float robotDiameter = 2.4f;
    private float GoalHeight = 30f;
    public float maxKickSpeed = 40f;
    private float MaxSpeedRobot = 18f; // ?
    private float lastKickTime = 0f;
    bool areBlue = false;

    // Tunable parameters
    private float distanceBackup = 40f;
    private float distanceSolution = 30f;
    private float distanceEnemyArriving = 15f;
    private float distanceDribbling = 15f;
    private float ballInAirLimit = 5f;
    private float kP = 2f;
    private float kD = 1f;
    private float offsetFromPost = 3f;

    Vector3 EnemyGoalPos;
    Vector3 EnemyGoalTop;
    Vector3 EnemyGoalBottom;
    Vector3 OurGoalPos;
    Vector3 OurGoalTop;
    Vector3 OurGoalBottom;
    

    private bool challenger = false;
    private bool backup = false;
    private bool solution = false;

    private Vector3 angleToShoot;

    // Start is called before the first frame update
    private void Start()
    {
        // get the car controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

        myPandaBT = GetComponent<PandaBehaviour>();

        ball = GameObject.FindGameObjectWithTag("Ball");
        my_rigidbody = GetComponent<Rigidbody>();

        // note that both arrays will have holes when objects are destroyed
        // but for initial planning they should work
        friend_tag = gameObject.tag;
        if (friend_tag == "Blue")
            enemy_tag = "Red";
        else
            enemy_tag = "Blue";

        friends = GameObject.FindGameObjectsWithTag(friend_tag);
        enemies = GameObject.FindGameObjectsWithTag(enemy_tag);
        
        // TODO: Maybe change this. GoalHeight? 
        frontPost = own_goal.transform.position - new Vector3(0f, 0f, GoalHeight / 2 - offsetFromPost);
        backPost = own_goal.transform.position + new Vector3(0f, 0f, GoalHeight / 2 - offsetFromPost);

        if (AreWeBlue())
            areBlue = true;

        OurGoalPos = terrain_manager.myInfo.start_pos;
        OurGoalTop = OurGoalPos;
        OurGoalBottom = OurGoalPos;
        OurGoalTop.z = OurGoalTop.z + GoalHeight/2;
        OurGoalBottom.z = OurGoalBottom.z - GoalHeight/2;

        EnemyGoalPos = terrain_manager.myInfo.goal_pos;
        EnemyGoalTop = EnemyGoalPos;
        EnemyGoalBottom = EnemyGoalPos;
        EnemyGoalTop.z = EnemyGoalTop.z + GoalHeight/2;
        EnemyGoalBottom.z = EnemyGoalBottom.z - GoalHeight/2;
    }

    // Update is called once per frame
    private void Update()
    {
        myPandaBT.Reset();
        myPandaBT.Tick();

        Vector3 ballPos = ball.transform.position;
        Vector3 ballTop = ballPos;
        ballTop.z = ballTop.z + ballDiameter/2;
        Vector3 ballBottom = ballPos;
        ballBottom.z = ballBottom.z - ballDiameter/2;

        //if (areBlue)
        //{
        //    Debug.DrawLine(ballTop, OurGoalTop, Color.blue);
        //    Debug.DrawLine(ballBottom, OurGoalBottom, Color.blue);
        //    Debug.DrawLine(ballTop, EnemyGoalTop, Color.red);
        //    Debug.DrawLine(ballBottom, EnemyGoalBottom, Color.red);
        //}
        //else
        //{
        //    Debug.DrawLine(ballTop, OurGoalTop, Color.red);
        //    Debug.DrawLine(ballBottom, OurGoalBottom, Color.red);
        //    Debug.DrawLine(ballTop, EnemyGoalTop, Color.blue);
        //    Debug.DrawLine(ballBottom, EnemyGoalBottom, Color.blue);
        //}

        //DrawVelocity();
    }


    // ###############################
    // FUNCTIONS
    // ###############################

    private float distance(Vector3 pA, Vector3 pB)
    {
        return (pA - pB).magnitude;
    }

    private float mindistanceFromRobotsToPoint(Vector3 position)
    {
        float min_dist = 1000000f;
        foreach (GameObject friend in friends) // look for smallest distance to the point
        {
            float dist = distance(friend.transform.position, position);
            if (dist < min_dist)
                min_dist = dist;
        }

        return min_dist;
    }

    private float mindistanceFromRobotsOnGoodSideOfBall(Vector3 position)
    {
        float min_dist = 1000000f;
        foreach (GameObject friend in friends) // look for smallest distance to the point
        {
            if ((friend.transform.position.x - own_goal.transform.position.x)/(ball.transform.position.x - own_goal.transform.position.x) < 1)  // good side of the ball
            {
                float dist = distance(friend.transform.position, position);
                if (dist < min_dist)
                    min_dist = dist;
            }
        }

        if (min_dist == 1000000f)  // all robots on the wrong side
        {
            foreach (GameObject friend in friends) // look for smallest distance to the point
            {
                float dist = distance(friend.transform.position, position);
                if (dist < min_dist)
                    min_dist = dist;
            }
        }

        return min_dist;
    }

    private float distanceEnemyToBall()
    {
        float min_dist_enemy = 1000000f;
        foreach (GameObject enemy in enemies) // look for smallest enemy distance to ball
        {
            float dist = distance(enemy.transform.position, ball.transform.position);
            if (dist < min_dist_enemy)
                min_dist_enemy = dist;
        }

        return min_dist_enemy;
    }

    private void DrawVelocity()
    {
        Vector3 ballPos = ball.transform.position;
        Rigidbody rbBall = ball.GetComponent<Rigidbody>();
        Vector3 ballSpeed = rbBall.velocity;

        Vector3 robotPos = transform.position;
        Rigidbody rbCurrentRobot = GetComponent<Rigidbody>();
        Vector3 currentRobotSpeed = rbCurrentRobot.velocity;

        Vector3 RobotWithMaxSpeed = robotPos;
        RobotWithMaxSpeed.x = RobotWithMaxSpeed.x + MaxSpeedRobot;

        Debug.DrawLine(ballPos, ballPos+ballSpeed, Color.black);
        Debug.DrawLine(robotPos, robotPos+currentRobotSpeed, Color.black);
        //Debug.DrawLine(robotPos, RobotWithMaxSpeed, Color.green);
    }

    private void AttackRoleAssignment()
    {
        // initialize
        challenger = false;
        backup = false;
        solution = false;

        Vector3 pos = transform.position;
        Vector3 ball_pos = ball.transform.position;
        Vector3 backupPosition = ball.transform.position;
        backupPosition.x = backupPosition.x - distanceBackup;
        Vector3 solutionPosition = ball.transform.position;
        if (solutionPosition.z > 100)  // top of the field
            solutionPosition.z = solutionPosition.z - distanceSolution;
        else  // bottom of the field
            solutionPosition.z = solutionPosition.z + distanceSolution;


        // challenger
        float dist = distance(pos, ball_pos);
        float min_dist = mindistanceFromRobotsOnGoodSideOfBall(ball_pos);  // takes into account the side where the robots are (/ball)
        if (dist == min_dist)  // between our_goal and ball
            challenger = true;

        // backup
        if (!challenger)  // no role assigned yet
        {
            // find other free robot for backup/solution
            Vector3 otherPos = pos;
            foreach (GameObject friend in friends)  // look for smallest distance to the point
            {
                float otherDist = distance(friend.transform.position, ball_pos); // distance to ball
                if ((otherDist != min_dist) && (otherDist != dist))
                    otherPos = friend.transform.position;
            }

            // Calculate cost: -> backup / other -> solution
            float dist1 = Mathf.Max(distance(pos, backupPosition), distance(otherPos, solutionPosition));
            // Calculate cost: -> solution / other -> backup
            float dist2 = Mathf.Max(distance(pos, solutionPosition), distance(otherPos, backupPosition));

            if (dist1 < dist2)
                backup = true;
            else
                solution = true;
        }

        if (print)
            Debug.Log(team + "AttackRoleAssignment : position = " + transform.position + "; challenger = " + challenger + "; backup = " + backup + "; solution = " + solution);
    }

    private bool CanKick()
    {
        float dist = distance(transform.position, ball.transform.position);
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

    private bool AreWeBlue()
    {
        return own_goal.transform.position == terrain_manager.myInfo.start_pos;
    }

    private bool DirectedToEnemyGoal(Vector3 dir)
    {
        Vector3 ballPos = ball.transform.position;
        Vector3 enemyGoalPos; // center of the enemy goal
        if (areBlue)
            enemyGoalPos = terrain_manager.myInfo.goal_pos;
        else // we are red
            enemyGoalPos = terrain_manager.myInfo.start_pos;

        if ((enemyGoalPos.x - ballPos.x) / dir.x > 0) // shoot in direction of the enemy goal
        {
            float zAtGoal = ballPos.z + (enemyGoalPos.x - ballPos.x) / dir.x * dir.z; // z coordinate of the ball when at the x-level of the enemy goal
            Debug.Log(team + "dteg: zAtGoal = " + zAtGoal);
            Debug.DrawLine(ballPos, ballPos+4*dir, Color.white);
            if (enemyGoalPos.z - GoalHeight / 2 <= zAtGoal && zAtGoal <= enemyGoalPos.z + GoalHeight / 2)
                return true;
        }

        return false;
    }

    private bool NoObstacleForShooting(Vector3 dir)
    {
        Vector3 dirOrthogonal = new Vector3(-dir.z, 0, dir.x).normalized;

        Vector3 ballPos = ball.transform.position;
        Vector3 ballPosUp = ballPos + dirOrthogonal * ballDiameter;
        Vector3 ballPosDown = ballPos - dirOrthogonal * ballDiameter;

        Vector3 enemyGoalPos = other_goal.transform.position;

        float xAfterGoal = enemyGoalPos.x + 25f;
        float zAfterGoal = ballPos.z + (xAfterGoal - ballPos.x) / dir.x * dir.z;

        Vector3 trajectory = new Vector3(xAfterGoal - ballPos.x, 0, zAfterGoal - ballPos.z);
        float dist = trajectory.magnitude;
        float timeBallArrive = dist / maxKickSpeed;

        /*Vector3 dirUp = new Vector3(xAfterGoal - ballPosUp.x + dirOrthogonal.x * timeBallArrive * MaxSpeedRobot, 0,
            zAfterGoal - ballPosDown.x + dirOrthogonal.z * timeBallArrive * MaxSpeedRobot); // Velocity Obstacle, up
        Vector3 dirDown = new Vector3(xAfterGoal - ballPosDown.x + dirOrthogonal.x * timeBallArrive * MaxSpeedRobot, 0,
            zAfterGoal - ballPosDown.z - dirOrthogonal.z * timeBallArrive * MaxSpeedRobot); // Velocity Obstacle, down*/
        
        Vector3 afterGoalUp = new Vector3(xAfterGoal + dirOrthogonal.x * timeBallArrive * MaxSpeedRobot, 0, zAfterGoal + dirOrthogonal.z * timeBallArrive * MaxSpeedRobot);
        Vector3 afterGoalDown = new Vector3(xAfterGoal - dirOrthogonal.x * timeBallArrive * MaxSpeedRobot, 0, zAfterGoal - dirOrthogonal.z * timeBallArrive * MaxSpeedRobot);
        Vector3 dirUp = (afterGoalUp - ballPosUp).normalized;
        Vector3 dirDown = (afterGoalDown - ballPosDown).normalized;

        Debug.Log(team + "nofs: zAtGoal = " + zAfterGoal + "; " + afterGoalUp + "; " + afterGoalDown + "; " + dirUp + "; " + dirDown);
        Debug.DrawLine(ballPos, ballPos+dirUp, Color.white);
        Debug.DrawLine(ballPos, ballPos+dirDown, Color.white);

        // Look if friends or enemies are in the VO
        foreach (GameObject friend in friends) // look for smallest distance to ball
        {
            Vector3 pos = friend.transform.position;
            if (pos.z < ballPosUp.z + dirUp.z * (pos.x - ballPosUp.x) / dirUp.x) // below dirUp
            {
                if (pos.z > ballPosDown.z + dirDown.z * (pos.x - ballPosDown.x) / dirDown.x) // above dirDown
                    return false;
            }
        }

        foreach (GameObject enemy in enemies) // look for smallest enemy distance to ball
        {
            Vector3 pos = enemy.transform.position;
            if (pos.z < ballPosUp.z + dirUp.z * (pos.x - ballPosUp.x) / dirUp.x) // below dirUp
            {
                if (pos.z > ballPosDown.z + dirDown.z * (pos.x - ballPosDown.x) / dirDown.x) // above dirDown
                    return false;
            }
        }

        Vector3 pointAfterGoal = new Vector3(xAfterGoal, 0, zAfterGoal);
        Debug.DrawLine(ballPos, pointAfterGoal, Color.white);

        return true;
    }

    private bool FreeAngleOnGoal()
    {
        Vector3 ballPos = ball.transform.position;
        Vector3 direction = ballPos - transform.position;  // direction : robot -> ball

        if (DirectedToEnemyGoal(direction))
        {
            /*Debug.Log(team + "FreeAngleOnGoal : DTEG");*/
            if (NoObstacleForShooting(direction))
            {
                /*Debug.Log(team + "FreeAngleOnGoal : NOFS");*/
                return true;
            }
        }
        return false;
    }

    private float convertToRadian(float angle)
    {
        return angle * (float) Math.PI / 180;
    }

    private bool PossibilityShootOnGoal()
    {
        Vector3 ballPos = ball.transform.position;
        Vector3 enemyGoalPos = other_goal.transform.position;
        Vector3 direction = enemyGoalPos - ballPos; 

        List<(Vector3, float)> friendsList = new List<(Vector3, float)>();
        List<(Vector3, float)> enemiesList = new List<(Vector3, float)>();

        foreach (GameObject friend in friends)  // look for smallest distance to the point
        {
            if ((friend.transform.position.x - enemyGoalPos.x)/(ball.transform.position.x - enemyGoalPos.x) < 1)  // good side of the ball
                friendsList.Add((friend.transform.position, robotDiameter + ballDiameter/2));
        }

        foreach (GameObject enemy in enemies)  // look for smallest distance to the point
        {
            if ((enemy.transform.position.x - enemyGoalPos.x)/(ball.transform.position.x - enemyGoalPos.x) < 1)  // good side of the ball
            {
                float angleBetween = (float) Vector3.Angle(enemyGoalPos-ballPos, enemy.transform.position-ballPos);
                angleBetween = convertToRadian(angleBetween);
                float xDistToBall = (float) distance(ballPos, enemy.transform.position) * Mathf.Cos(angleBetween);
                float radius = robotDiameter + xDistToBall * MaxSpeedRobot/400;

                Vector3 newPoint = enemy.transform.position;
                newPoint.z = newPoint.z + radius;
                //Debug.DrawLine(enemy.transform.position, newPoint, Color.green);
                newPoint.z = newPoint.z - 2* radius;
                //Debug.DrawLine(enemy.transform.position, newPoint, Color.green);

                enemiesList.Add((enemy.transform.position, radius));
            }
        }

        int numberAngles = 100;
        Vector3 enemyGoalObj = enemyGoalPos;
        enemyGoalObj.z = enemyGoalObj.z + GoalHeight/2;  // up
        for (int i = 0; i < numberAngles; i++)  // up to down
        {
            float step = (float) 1 / numberAngles;
            enemyGoalObj.z = enemyGoalObj.z - step * GoalHeight;
            Vector3 directionObj = enemyGoalObj - ballPos;

            bool cond = true;
            foreach ((Vector3 p, float r) obj in friendsList)  // look for smallest distance to the point
            {
                float angleBetween = (float) Vector3.Angle(directionObj-ballPos, obj.p-ballPos);
                angleBetween = convertToRadian(angleBetween);
                float zDistToBall = (float) distance(ballPos, obj.p) * Mathf.Sin(angleBetween);
                if (zDistToBall < obj.r)
                    cond = false;
            }
            foreach ((Vector3 p, float r) obj in enemiesList)  // look for smallest distance to the point
            {
                float angleBetween = (float) Vector3.Angle(directionObj-ballPos, obj.p-ballPos);
                angleBetween = convertToRadian(angleBetween);
                float zDistToBall = (float) distance(ballPos, obj.p) * Mathf.Sin(angleBetween);
                if (zDistToBall < obj.r)
                    cond = false;
            }

            if (cond)
            {
                angleToShoot = directionObj;
                return true;
            }
        }

        return false;
    }

    private Vector3 findBallVelocityToAdd(Vector3 dirObj)
    {
        /*Debug.Log(team + dirObj);*/

        Vector3 ball_pos = ball.transform.position;
        Rigidbody rbBall = ball.GetComponent<Rigidbody>();
        Vector3 ballSpeed = rbBall.velocity;
        dirObj = dirObj.normalized;
        Vector3 dirObjOrtho = new Vector3(-dirObj.z, 0, dirObj.x);

        float angleBetween = (float) Vector3.Angle(ballSpeed, dirObj);
        angleBetween = convertToRadian(angleBetween);
        float zDistToObj = (float) ballSpeed.magnitude * Mathf.Sin(angleBetween);

        int sign = Math.Sign(dirObjOrtho.x * ballSpeed.x + dirObjOrtho.z * ballSpeed.z);
        Vector3 closestVelocity = ball_pos - sign * zDistToObj * dirObjOrtho;

        if (distance(ball_pos, closestVelocity) > maxKickSpeed)  // maxKickSpeed not enough power
            return closestVelocity;

        float angleCos = distance(ball_pos, closestVelocity) / maxKickSpeed;
        float angleNewObj = Mathf.Acos(angleCos);
        float distNew = distance(ball_pos, closestVelocity) * Mathf.Tan(angleNewObj);

        Vector3 newObj = closestVelocity + distNew * dirObj;

        return newObj - ball_pos;
    }

    private bool IsBallInAir()
    {
        Vector3 ballPos = ball.transform.position;
        return ballPos.y > ballInAirLimit;
    }

    //private float 

    private void PDcontroller(Vector3 targetPos)
    {
        targetPos = transform.position + targetPos;
        Vector3 targetVelocity = (targetPos - oldTargetPos) / Time.fixedDeltaTime;
        oldTargetPos = targetPos;
        // a PD-controller to get desired velocity
        Vector3 positionError = targetPos - transform.position;
        Vector3 velocityError = targetVelocity - my_rigidbody.velocity;
        Vector3 desiredAcceleration = kP * positionError + kD * velocityError;
        // Debug.DrawLine(transform.position, transform.position + desiredAcceleration, Color.red);
        // Debug.DrawLine(transform.position, targetPos, Color.yellow);
        m_Drone.Move_vect(desiredAcceleration);
    }


    // ###############################
    // TASKS
    // ###############################

    /*[Task]
    bool IsBallCloserThan(float distance)
    {
        if (distance > 0)
            Debug.Log(team + "IsBallCloserThan here we are");
        return (ball.transform.position - transform.position).sqrMagnitude < distance * distance;
    }*/

    [Task]
    bool IsCloserToBallThanEnemy()
    {
        Vector3 ball_pos = ball.transform.position;
        float min_dist = mindistanceFromRobotsToPoint(ball_pos);
        float min_dist_enemy = distanceEnemyToBall();

        bool res = (min_dist <= min_dist_enemy);
        if (res)
        {
            if (print)
                Debug.Log(team + "IsCloserToBallThanEnemy : true");
            AttackRoleAssignment();  // find role for current robot
        }
        return res;
    }

    [Task]
    void PrintIfCloserToBallThanEnemy(bool cond)
    {
        if (cond)
            Debug.Log(team + "closest to the ball");
        else
            Debug.Log(team + "not closer to the ball");
    }

    [Task]
    bool CurrentRobotclosest()
    {
        if (challenger && print)
            Debug.Log(team + "IsCloserToBallThanEnemy : true");
        return challenger;
    }

    [Task]
    bool AngleOnFreeGoal() // TODO : can we kick on the side?
    {
        if (CanKick())
        {
            /*if (FreeAngleOnGoal())
                return true;*/
            if (PossibilityShootOnGoal())
                return true;
        }

        return false;
    }

    [Task]
    void Shoot()
    {
        if (print)
            Debug.Log(team + "Shoot : true");
        //Vector3 direction = ball.transform.position - transform.position;
        //Vector3 velocity = maxKickSpeed * direction;
        Vector3 velocity = findBallVelocityToAdd(angleToShoot);
        Debug.DrawLine(ball.transform.position, ball.transform.position+velocity, Color.cyan);
        KickBall(velocity);
        Task.current.Succeed();
    }

    // TODO: Pass and Challenge
    /*[Task]
    bool MakePass()
    {}*/

    [Task]
    void EnemyArriving()
    {
        float min_dist_enemy = distanceEnemyToBall();
        if (min_dist_enemy < distanceEnemyArriving)
            Task.current.Succeed();
        //return true;
        //return false;
        Task.current.Fail();
    }


    [Task]
    void Challenge()
    {
        if (print)
            Debug.Log(team + "Challenge : true");
        Vector3 objective = ball.transform.position;

        Vector3 ourGoalPos; // center of our goal
        if (areBlue)
            ourGoalPos = terrain_manager.myInfo.start_pos;
        else // we are red
            ourGoalPos = terrain_manager.myInfo.goal_pos;

        // Go against ball, on axis own_goal/ball
        Vector3 axis = ourGoalPos - objective;
        axis = axis.normalized;
        objective.x = objective.x + ballDiameter / 2 * axis.x;
        objective.z = objective.z + ballDiameter / 2 * axis.z;

        Vector3 direction = objective - transform.position;
        // m_Drone.Move_vect(direction); // move the robot
        PDcontroller(direction); // move the robot
        Task.current.Succeed();
    }

    [Task]
    void Dribble() // TODO: direction of dribbling, avoid enemies, etc
    {
        Vector3 objective = ball.transform.position;
        Vector3 EnemyGoalPos; // center of the enemy goal
        if (areBlue)
            EnemyGoalPos = terrain_manager.myInfo.goal_pos;
        else // we are red
            EnemyGoalPos = terrain_manager.myInfo.start_pos;
        
        if (distance(transform.position, objective) > distanceDribbling)  // far away from ball, take into account velocity
        {
            Rigidbody rbBall = ball.GetComponent<Rigidbody>();
            Vector3 ballSpeed = rbBall.velocity;
            objective = objective + ballSpeed;
        }

        // Go against ball, on axis own_goal/ball
        Vector3 axis = objective - EnemyGoalPos;
        axis = axis.normalized;
        objective.x = objective.x + ballDiameter / 2 * axis.x; // take into account size of drone ?!
        objective.z = objective.z + ballDiameter / 2 * axis.z;

        if (print)
            Debug.Log(team + "Dribble : true " + transform.position + "; " + objective);

        Vector3 direction = objective - transform.position;
        m_Drone.Move_vect(direction); // move the robot
        // PDcontroller(direction); // move the robot
        Task.current.Succeed();
    }

    [Task]
    bool Backup()
    {
        return backup;
    }

    [Task]
    void GoBackup()
    {
        if (print)
            Debug.Log(team + "GoBackup : true");
        Vector3 backupPosition = ball.transform.position;
        if (areBlue)
            backupPosition.x = backupPosition.x - distanceBackup;  // between our_goal and the ball
        else
            backupPosition.x = backupPosition.x + distanceBackup;  // between our_goal and the ball
        Vector3 direction = backupPosition - transform.position;
        // m_Drone.Move_vect(direction); // move the robot
        PDcontroller(direction); // move the robot
        Task.current.Succeed();
    }

    [Task]
    void GoSolution()
    {
        if (print)
            Debug.Log(team + "GoSolution : true");
        Vector3 solutionPosition = ball.transform.position;
        if (solutionPosition.z > 100) // top of the field
            solutionPosition.z = solutionPosition.z - distanceSolution;
        else // bottom of the field
            solutionPosition.z = solutionPosition.z + distanceSolution;
        Vector3 direction = solutionPosition - transform.position;
        // m_Drone.Move_vect(direction); // move the robot
        PDcontroller(direction); // move the robot
        Task.current.Succeed();
    }

    [Task]
    bool IsLastDefender()
    {
        float xPos = transform.position.x;
        Vector3 ball_pos = ball.transform.position;

        Vector3 ourGoalPos; // center of our goal
        if (areBlue)
            ourGoalPos = terrain_manager.myInfo.start_pos;
        else // we are red
            ourGoalPos = terrain_manager.myInfo.goal_pos;

        float dist = distance(transform.position, ball_pos);
        float min_dist = mindistanceFromRobotsToPoint(ball_pos);
        if (dist == min_dist) // closest defender
        {
            foreach (GameObject friend in friends) // look for smallest distance to the point
            {
                float xPosFriend = friend.transform.position.x;
                if (areBlue)
                {
                    if (xPosFriend < xPos) // left of the closest robot to the ball
                        return false;
                }
                else
                {
                    if (xPosFriend > xPos) // right of the closest robot to the ball
                        return false;
                }
            }

            return true; // last defender
        }

        return false; // not closest defender
    }

    [Task]
    void ShadowDefense()
    {
        if (print)
            Debug.Log(team + "ShadowDefense : true");
        Vector3 objective = ball.transform.position;

        Vector3 ourGoalPos; // center of our goal
        if (areBlue)
            ourGoalPos = terrain_manager.myInfo.start_pos;
        else // we are red
            ourGoalPos = terrain_manager.myInfo.goal_pos;

        // Go between ball and own_goal, on axis own_goal/ball
        float distBallToOwnGoal = distance(ourGoalPos, objective);
        Vector3 axis = ourGoalPos - objective;
        axis = axis.normalized;
        objective.x = objective.x + distBallToOwnGoal/3 * axis.x;  // 1/3 from ball to our_goal
        objective.z = objective.z + distBallToOwnGoal/3 * axis.z;

        Vector3 direction = objective - transform.position;
        // m_Drone.Move_vect(direction); // move the robot
        PDcontroller(direction); // move the robot
        Task.current.Succeed();
    }

    [Task]
    bool InOurHalfField()
        // Return True if ball is in our half-field
    {
        float distOurGoal = (ball.transform.position - own_goal.transform.position).magnitude;
        float distOtherGoal = (ball.transform.position - other_goal.transform.position).magnitude;
        return distOurGoal < distOtherGoal;
    }

    [Task]
    bool FrontPost()
        // Return true if current agent should be at fronpost
    {
        float minDist = mindistanceFromRobotsToPoint(ball.transform.position);
        GameObject other = transform.gameObject;  // GameObject other = null;  //  transform.position;
        foreach (GameObject friend in friends)
        {
            if (friend.transform == transform)
            {
                continue;
            }
            else if (Math.Abs((friend.transform.position - ball.transform.position).magnitude - minDist) < 0.1)
            {
                continue;
            }
            other = friend;
        }
        // Calculate cost: -> backup / other -> solution
        float dist1 = Mathf.Max(distance(transform.position, frontPost), distance(other.transform.position, backPost));
        // Calculate cost: -> solution / other -> backup
        float dist2 = Mathf.Max(distance(transform.position, backPost), distance(other.transform.position, frontPost));

        if (dist1 < dist2)
            return true;
        else
            return false;
    }

    [Task]
    void GoFrontPost()
    {
        Vector3 position = frontPost + (ball.transform.position - frontPost) * 0.1f;
        Vector3 direction = position - transform.position;
        // m_Drone.Move_vect(direction); // move the robot
        PDcontroller(direction); // move the robot
        Task.current.Succeed();
    }

    [Task]
    void RotateBackPost()
    {
        Vector3 position = backPost + (ball.transform.position - backPost) * 0.1f;
        Vector3 direction = position - transform.position;
        // m_Drone.Move_vect(direction); // move the robot
        PDcontroller(direction); // move the robot
        Task.current.Succeed();
    }
}