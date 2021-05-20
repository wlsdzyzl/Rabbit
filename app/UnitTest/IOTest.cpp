#include "Utils/IO.h"
using namespace rabbit;
using namespace util;
int main(int argc, char **argv)
{
    if(argc > 1)
    {
        if(DirExists(argv[1]))
        {
            std::vector<std::string> seq; 
            GetPCDSequence(argv[1], seq);
            std::cout<<"Find "<< seq.size()<<" point clouds."<<std::endl;
            std::vector<int> has_pcd(seq.size(), -1);
            int max_id = -1;
            for(size_t i = 0; i != seq.size(); ++i)
            {
                int id = ExtractIDFromPath(seq[i]);
                if(has_pcd.size() <= id) has_pcd.resize(id + 1, -1);
                has_pcd[id] = 1;
                if(id > max_id) max_id = id;
            }
            std::cout<<"max_id: "<<max_id<<std::endl;
            for(size_t i = 0; i != has_pcd.size(); ++i)
            if(has_pcd[i] == -1) std::cout<<"lack of pcd "<<i<<std::endl;
        }
    }
    return 0;
}