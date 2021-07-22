#include "System/MovingBox.h"
namespace rabbit
{
namespace system
{
    int MovingBox::GetCubeID(int x_id, int y_id, int z_id)
    {
        if(x_id >= cube_size_x || x_id < 0) return -1;
        if(y_id >= cube_size_y || y_id < 0) return -1;
        if(z_id >= cube_size_z || z_id < 0) return -1;
        return x_id * cube_size_y * cube_size_z
            + y_id * cube_size_z + z_id;
    }
    int MovingBox::GetCubeID(const util::Vec3 &pos)
    {
        int x_id = std::round(pos(0) / cube_len)
            - cube_origin_x;
        int y_id = std::round(pos(1) / cube_len)
            - cube_origin_y;
        int z_id = std::round(pos(2) / cube_len)
            - cube_origin_z;

        return GetCubeID(x_id, y_id, z_id);
    }
    void MovingBox::CenterCubesAt(const util::Vec3 &pos)
    {
        int tmp_origin_x = std::round(pos(0) / cube_len)
            - std::floor(cube_size_x / 2);
        int tmp_origin_y = std::round(pos(1) / cube_len)
            - std::floor(cube_size_y / 2);
        int tmp_origin_z = std::round(pos(2) / cube_len)
            - std::floor(cube_size_z / 2);
        int x_offset = tmp_origin_x - cube_origin_x;
        int y_offset = tmp_origin_y - cube_origin_y;
        int z_offset = tmp_origin_z - cube_origin_z;
        std::cout<<"Box Moved to ("<<tmp_origin_x<<", "<<tmp_origin_y<<
            ", "<<tmp_origin_z<<")."<<std::endl;
        if(x_offset >= 0) 
        {
            for(int i = 0; i < cube_size_x; ++i)
            {
                if(y_offset >= 0)
                {
                    for(int j = 0; j < cube_size_y; ++j)
                    {
                        if(z_offset >= 0)
                        {
                            for(int k = 0; k < cube_size_z; ++k)
                            {
                                int curr_id = GetCubeID(i, j, k);
                                int old_id = GetCubeID(i + x_offset, j + y_offset, k + z_offset);
                                if(old_id != -1)// get current cube the value of old cube
                                cubes[curr_id] = cubes[old_id];
                                else
                                cubes[curr_id].clear();
                            }
                        }
                        else
                        {
                            for(int k = cube_size_z - 1; k >= 0; --k)
                            {
                                int curr_id = GetCubeID(i, j, k);
                                int old_id = GetCubeID(i + x_offset, j + y_offset, k + z_offset);
                                if(old_id != -1)// get current cube the value of old cube
                                cubes[curr_id] = cubes[old_id];
                                else
                                cubes[curr_id].clear();
                            }
                        }
                    }
                }
                else
                {
                    for(int j = cube_size_y - 1; j >= 0; --j)
                    {
                        if(z_offset >= 0)
                        {
                            for(int k = 0; k < cube_size_z; ++k)
                            {
                                int curr_id = GetCubeID(i, j, k);
                                int old_id = GetCubeID(i + x_offset, j + y_offset, k + z_offset);
                                if(old_id != -1)// get current cube the value of old cube
                                cubes[curr_id] = cubes[old_id];
                                else
                                cubes[curr_id].clear();
                            }
                        }
                        else
                        {
                            for(int k = cube_size_z - 1; k >= 0; --k)
                            {
                                int curr_id = GetCubeID(i, j, k);
                                int old_id = GetCubeID(i + x_offset, j + y_offset, k + z_offset);
                                if(old_id != -1)// get current cube the value of old cube
                                cubes[curr_id] = cubes[old_id];
                                else
                                cubes[curr_id].clear();
                            }
                        }
                    }
                }
            }
        }
        else
        {
            for(int i = cube_size_x - 1; i >= 0; --i)
            {
                if(y_offset >= 0)
                {
                    for(int j = 0; j < cube_size_y; ++j)
                    {
                        if(z_offset >= 0)
                        {
                            for(int k = 0; k < cube_size_z; ++k)
                            {
                                int curr_id = GetCubeID(i, j, k);
                                int old_id = GetCubeID(i + x_offset, j + y_offset, k + z_offset);
                                if(old_id != -1)// get current cube the value of old cube
                                cubes[curr_id] = cubes[old_id];
                                else
                                cubes[curr_id].clear();
                            }
                        }
                        else
                        {
                            for(int k = cube_size_z - 1; k >= 0; --k)
                            {
                                int curr_id = GetCubeID(i, j, k);
                                int old_id = GetCubeID(i + x_offset, j + y_offset, k + z_offset);
                                if(old_id != -1)// get current cube the value of old cube
                                cubes[curr_id] = cubes[old_id];
                                else
                                cubes[curr_id].clear();
                            }
                        }
                    }
                }
                else
                {
                    for(int j = cube_size_y - 1; j >= 0; --j)
                    {
                        if(z_offset >= 0)
                        {
                            for(int k = 0; k < cube_size_z; ++k)
                            {
                                int curr_id = GetCubeID(i, j, k);
                                int old_id = GetCubeID(i + x_offset, j + y_offset, k + z_offset);
                                if(old_id != -1)// get current cube the value of old cube
                                cubes[curr_id] = cubes[old_id];
                                else
                                cubes[curr_id].clear();
                            }
                        }
                        else
                        {
                            for(int k = cube_size_z - 1; k >= 0; --k)
                            {
                                int curr_id = GetCubeID(i, j, k);
                                int old_id = GetCubeID(i + x_offset, j + y_offset, k + z_offset);
                                if(old_id != -1)// get current cube the value of old cube
                                cubes[curr_id] = cubes[old_id];
                                else
                                cubes[curr_id].clear();
                            }
                        }
                    }
                }
            }
        }
        cube_origin_x = tmp_origin_x;
        cube_origin_y = tmp_origin_y;
        cube_origin_z = tmp_origin_z;
    }
}
}