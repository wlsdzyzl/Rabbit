#include "Utils/Utils.h"
#include <boost/filesystem.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
namespace rabbit
{
namespace util
{
    std::vector<std::string> Split(const std::string &str, const std::string &delim, int split_times)
    {
        size_t start = 0;
        std::vector<std::string> results;
        int current_split_time = 0;
        if(split_times == 0)
        {
            results.push_back(str);
            return results;
        }
        while(start < str.size()) 
        {
            size_t new_start = str.find(delim, start);
            
            if(new_start != std::string::npos)
            {
                current_split_time += 1;
                if(new_start - start > 0)
                    results.push_back(str.substr(start, new_start));
                start = new_start + delim.size();
                
            }
            else break; 
            if(split_times > 0 && current_split_time >= split_times )
            break;
        }
        if( start < str.size())
        results.push_back(str.substr(start, str.size()));
        return results;
    }
    std::vector<std::string> RSplit(const std::string &str, const std::string &delim, int split_times)
    {
        size_t end = str.size();
        std::vector<std::string> results;
        int current_split_time = 0;
        if(split_times == 0)
        {
            results.push_back(str);
            return results;
        }
        while(end > 0)
        {
            
            size_t new_end = str.rfind(delim, end);
            if(new_end != std::string::npos)
            {
                current_split_time += 1;
                if(end - new_end - delim.size() > 0)
                    results.push_back(str.substr(new_end + delim.size(), end));
                end = new_end;
            }
            else break; 
            if(split_times > 0 && current_split_time >= split_times)
            break;
        }
        if( end > 0)
        results.push_back(str.substr(0, end));

        std::reverse(results.begin(), results.end());
        return results;
    }
    bool DirExists(const std::string & folder_name)
    {
         boost::filesystem::path p(folder_name.c_str());
        if(boost::filesystem::exists(p) && boost::filesystem::is_directory(p))
        return true;
        return false;
    }
    bool FileExists(const std::string & folder_name)
    {
         boost::filesystem::path p(folder_name.c_str());
        if(boost::filesystem::exists(p) &&!boost::filesystem::is_directory(p))
        return true;
        return false;
    }
    bool Exists(const std::string & folder_name)
    {
         boost::filesystem::path p(folder_name.c_str());
        return boost::filesystem::exists(p);
    }
    bool MakeDir(const std::string & folder_name)
    {
        if(DirExists(folder_name))
        {
            std::cout<<YELLOW<<"[WARNNING]::[MakeDir]::Directory exsits!"<<RESET<<std::endl;
            return true;            
        }
        const int dir_err = mkdir(folder_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (-1 == dir_err)
        {
            std::cout<<RED<<"[ERROR]::[MakeDir]::Error creating directory!"<<RESET<<std::endl;
            return false;
        }
        return true;
    }
    void ListFileNames(const std::string &folder_name, std::vector<std::string> &filenames)
    {
        filenames.clear();
        if(!DirExists(folder_name)) return;
        boost::filesystem::path p(folder_name.c_str());
        for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
        {
            if (!boost::filesystem::is_directory(i->path())) //we eliminate directories
            {
                filenames.push_back(i->path().filename().string());
            }
        }     
    }
    void ListFilePaths(const std::string &folder_name, std::vector<std::string> &filenames)
    {
        filenames.clear();
        if(!DirExists(folder_name)) return;
        boost::filesystem::path p(folder_name.c_str());
        for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
        {
            if (!boost::filesystem::is_directory(i->path())) //we eliminate directories
            {
                filenames.push_back(i->path().string());
            }
        }     
    }
    std::string AbsolutePath(const std::string &f)
    {
        if(Exists(f))
        {
            boost::filesystem::path p(f.c_str());
            return boost::filesystem::canonical(p).string();
        }
        std::cout<<YELLOW<<"[WARNING]::[Util]::File or folder doesn't exist."<<RESET<<std::endl;
        return "";
    }
    int ExtractIDFromPath(const std::string &path)
    {
        //the path should be like /path/to/pcd/123.pcd
        std::vector<std::string> strs = RSplit(path, "." , 1);
        strs = RSplit(strs[0], "/", 1);
        return std::stoi(strs[1]);        
    }
    void ColorRemapping(const std::vector<float> &values, Vec3fList &mapped_color)
    {
        mapped_color.clear();
        float min_value = std::numeric_limits<float>::max();
        float max_value = std::numeric_limits<float>::lowest();
        for(size_t i = 0; i != values.size(); ++i)
        {
            if(values[i] < min_value) min_value = values[i];
            if(values[i] > max_value) max_value = values[i];
        }
        std::cout<<BLUE<<"[INFO]::[ColorRemapping]::Min value: "<<min_value<<", Max value:"<<max_value<<RESET<<std::endl;
        std::vector<int> histogram(COLOR_RANGE, 0);
        std::vector<int> color_id;
        for(size_t i = 0; i != values.size(); ++i)
        {
            size_t index = (values[i] - min_value) / (max_value - min_value) * (COLOR_RANGE - 1);
            color_id.push_back(index);
            histogram[index] ++;
        }
        std::vector<int> map(COLOR_RANGE, 0);
        size_t current_n = 0;
        Vec3fList color_table;
        for(size_t i = 0; i != COLOR_RANGE; ++i)
        {
            if(i < COLOR_RANGE / 2)
            {
                color_table.push_back( Vec3f((i*2 + 0.0) / COLOR_RANGE, (i * 2 +0.0)/ COLOR_RANGE, 1));
                            // std::cout<<i<<" "<<color_table.back().transpose()<<std::endl;
            }
            else
            {
                color_table.push_back( Vec3f(1, 2.0 * (COLOR_RANGE - i ) / COLOR_RANGE, 2.0 * (COLOR_RANGE - i) / COLOR_RANGE));
            }

        }
        for(size_t i = 0; i != COLOR_RANGE; ++i)
        {
            
            current_n += histogram[i];
            map[i] = (COLOR_RANGE - 1) * (current_n + 0.0) / values.size();
            // std::cout<<i<<" "<<histogram[i] <<" "<<map[i]<<std::endl;
            
        }
        for(size_t i = 0; i != values.size(); ++i)
        {
            mapped_color.push_back(color_table[map[color_id[i]]]);
        }
    }
    void ColorizePointCloud(const PointCloud &pcdi, PointCloudRGB &mapped_pcd)
    {
        mapped_pcd.points.resize(pcdi.points.size());
        std::vector<float> intensity_list;
        for (size_t i = 0; i < pcdi.points.size(); i++) 
        {
            mapped_pcd.points[i].x = pcdi.points[i].x;
            mapped_pcd.points[i].y = pcdi.points[i].y;
            mapped_pcd.points[i].z = pcdi.points[i].z;
            intensity_list.push_back( pcdi.points[i].intensity);
        }
        Vec3fList mapped_color;
        ColorRemapping(intensity_list, mapped_color);
        for(size_t i = 0; i != mapped_color.size(); ++i)
        {
            mapped_pcd.points[i].r = 255 * mapped_color[i](0);
            mapped_pcd.points[i].g= 255 * mapped_color[i](1);
            mapped_pcd.points[i].b = 255 * mapped_color[i](2);
        }       
    }
    void ColorizePointCloud(const PointCloud &pcdi, const Vec3f &c, PointCloudRGB &mapped_pcd)
    {
        mapped_pcd.points.resize(pcdi.points.size());
        std::vector<float> intensity_list;
        for (size_t i = 0; i < pcdi.points.size(); i++) 
        {
            mapped_pcd.points[i].x = pcdi.points[i].x;
            mapped_pcd.points[i].y = pcdi.points[i].y;
            mapped_pcd.points[i].z = pcdi.points[i].z;
            mapped_pcd.points[i].r = c[0] * 255;
            mapped_pcd.points[i].g = c[1] * 255;
            mapped_pcd.points[i].b = c[2] * 255;
        }
    }
    void EstimateNormal(const PointCloud &pcd, PCDNormal &n, float search_radius)
    {
        pcl::NormalEstimation<PointType, pcl::Normal> ne;
        ne.setInputCloud (PointCloudPtr(new PointCloud (pcd)));
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (search_radius);
        ne.compute (n);
    }
    void RemoveClosedPointCloud(const PointCloud&cloud_in,
                                PointCloud &cloud_out, float thres)
    {
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }
    void FilterPCD(const PointCloud &cloud_in, PointCloud &cloud_out, float x_leaf_size, float y_leaf_size, float z_leaf_size)
    {
        PointCloudPtr in_ptr(new PointCloud(cloud_in));
        pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize (x_leaf_size, y_leaf_size, z_leaf_size);
        approximate_voxel_filter.setInputCloud (in_ptr);
        approximate_voxel_filter.filter (cloud_out);
    }
}
}