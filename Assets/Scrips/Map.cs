using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;




public class Map
{


    // Discretization of a single grid
    private float single_grid_size;
    private int x_n;
    private int z_n;


    // Keeps track of the obstacles in a grip: 1 = obstacle, 0 = free
    public float[,] obstacles;
    private int safety_zone = 0;


    // Keeps track of how many nodes already exists within a grid cell
    public int[,] nodes;


    // Terrain manager to access environment information
    TerrainManager terrain_manager;





    public Map(TerrainManager tm, float gz)
    {
        single_grid_size = gz;
        terrain_manager = tm;
        InitGrid();
    }


    private void InitGrid()
    {

        x_n = (int)Math.Floor((terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / single_grid_size);
        z_n = (int)Math.Floor((terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / single_grid_size);


        obstacles = new float[x_n, z_n];


        nodes = new int[x_n, z_n];




        for (int i = 0; i < x_n; i++)
        {
            for (int j = 0; j < z_n; j++)
            {
                nodes[i, j] = -1;
                //shifted to bottom left
                float x_terrain = terrain_manager.myInfo.x_low + single_grid_size * i;
                float z_terrain = terrain_manager.myInfo.z_low + single_grid_size * j;
                Vector3 s = new Vector3(x_terrain, 0, z_terrain);
                Vector3 g1 = new Vector3(x_terrain + single_grid_size, 0, z_terrain);
                Vector3 g2 = new Vector3(x_terrain, 0, z_terrain + single_grid_size);

                if (CheckObstacle(i, j))
                {
                    obstacles[i, j] = 1;
                    Debug.DrawLine(s, g1, Color.red, 1000f);
                    Debug.DrawLine(s, g2, Color.red, 1000f);
                }
                else
                {
                    Debug.DrawLine(s, g1, Color.green, 1000f);
                    Debug.DrawLine(s, g2, Color.green, 1000f);
                }



            }
        }

    }


    public bool CheckObstacle(int i, int j)
    {


        List<(int, int)> safe_box = new List<(int, int)>();
        safe_box.Add((i - safety_zone, j - safety_zone));
        safe_box.Add((i - safety_zone, j));
        safe_box.Add((i - safety_zone, j + safety_zone));
        safe_box.Add((i, j - safety_zone));
        safe_box.Add((i, j));
        safe_box.Add((i, j + safety_zone));
        safe_box.Add((i + safety_zone, j - safety_zone));
        safe_box.Add((i + safety_zone, j));
        safe_box.Add((i + safety_zone, j + safety_zone));




        foreach ((int k, int l) in safe_box)
        {
            float x_terrain = terrain_manager.myInfo.x_low + single_grid_size / 2 + single_grid_size * k;
            float z_terrain = terrain_manager.myInfo.z_low + single_grid_size / 2 + single_grid_size * l;
            int ii = terrain_manager.myInfo.get_i_index(x_terrain);
            int jj = terrain_manager.myInfo.get_j_index(z_terrain);


            // Todo add border margin
            if (terrain_manager.myInfo.traversability[ii, jj] == 1)
            {
                return true;
            }
        }
        return false;
    }
    //public bool wall_check(int[] cell)
    //{
    //    Vector3 pp = get_xyz(cell[0], cell[1]);
    //    int i = terrain_manager.myInfo.get_i_index(pp.x);
    //    int j = terrain_manager.myInfo.get_j_index(pp.z);
    //    if (terrain_manager.myInfo.traversability[i, j] == 1)
    //    {
    //        return true;
    //    }
    //    else
    //        return false;
    //}


    public Vector3 get_xyz(int i, int j)
    {
        //get corresponding centerpoint of the grid in xyz cordinates
        float step_x = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / x_n;
        float x = terrain_manager.myInfo.x_low + step_x / 2 + step_x * i;


        float step_z = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / z_n;
        float z = terrain_manager.myInfo.z_low + step_z / 2 + step_z * j;
        return new Vector3(x, 0, z);
    }
    public int[] GetGridIndex(float x, float z)
    {


        int i = (int)Mathf.Floor(x_n * (x - terrain_manager.myInfo.x_low) / (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low));
        int j = (int)Mathf.Floor(z_n * (z - terrain_manager.myInfo.z_low) / (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low));

        if (i < 0)
        {
            i = 0;
        }
        else if (i > x_n - 1)
        {
            i = x_n - 1;
        }


        if (j < 0)
        {
            j = 0;
        }
        else if (j > z_n - 1)
        {
            j = z_n - 1;
        }




        return new int[] { i, j };
    }












    public float CheckNodeCount(int i, int j)
    {
        return nodes[i, j];
    }


    public void SetNodeCount(int i, int j, int c)
    {
        nodes[i, j] = c;
    }


    public void IncreaseNodeCount(int i, int j, int amount)
    {
        if (nodes[i, j] >= 9)
        {
            Debug.Log("Invalid Node count: " + nodes[i, j] + 1);
        }
        nodes[i, j] = amount;
    }
    public void ResetNodeCount()
    {
        Array.Clear(nodes, 0, nodes.Length);//reset the node array
        //one could perhaps just clear the turret we took out instead?
    }




}