#include "utility.h"

namespace utility
{

    bool read_FakeGPS(std::ifstream& is, NewPosesGPS& gps)
    {
        std::string line;
        Vector6d np;
        int seq;

        while(std::getline(is, line))
        {
            std::stringstream ss(line);
            ss >> seq;
            ss >> np[0] >> np[1] >> np[2] >> np[3] >> np[4] >> np[5];
            gps.push_back(std::make_pair(seq,np));
        }
            std::cout << gps.size() << std::endl;
        is.close();
        return true;
    }

//    Eigen::Vector3d t2v_2d(const Eigen::Isometry2d& iso)
//    {
//        Eigen::Vector3d t;
//        t.x() = iso.translation().x();
//        t.y() = iso.translation().y();
//        Eigen::Rotation2Dd r(0);
//        r.fromRotationMatrix(iso.linear());
//        t.z() = r.angle();
//        return t;
//    }


//    Eigen::Isometry2d v2t_2d(const Eigen::Vector3d& v)
//    {
//        Eigen::Isometry2d iso;
//        iso.translation() = v.head<2>();
//        iso.linear() = Eigen::Rotation2Dd(v.z()).toRotationMatrix();
//        return iso;
//    }
}
