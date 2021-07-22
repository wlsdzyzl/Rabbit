#include "Utils/Utils.h"
namespace rabbit
{
namespace system
{
    class MovingBox
    {
        public:
        MovingBox() = default;
        MovingBox(int x, int y, int z, double l)
        {
            if(!(x&1)) x+=1;
            if(!(y&1)) y+=1;
            if(!(z&1)) z+=1;
            cube_size_x = x;
            cube_size_y = y;
            cube_size_z = z;
            cube_origin_x = -std::floor(cube_size_x / 2);
            cube_origin_y = -std::floor(cube_size_y / 2);
            cube_origin_z = -std::floor(cube_size_z / 2);
            cubes.resize(cube_size_x * cube_size_y * cube_size_z,
                std::vector<int>());
            cube_len = l;
        }
        int GetCubeID(const util::Vec3 &pos);
        int GetCubeID(int x_id, int y_id, int z_id);
        void CenterCubesAt(const util::Vec3 &pos);
        int cube_size_x = 21;
        int cube_size_y = 21;
        int cube_size_z = 11;
        int cube_origin_x = -10;
        int cube_origin_y = -10;
        int cube_origin_z = -5; 
        double cube_len = 50.0;    
        std::vector<std::vector<int>> cubes;   
    };
}
}