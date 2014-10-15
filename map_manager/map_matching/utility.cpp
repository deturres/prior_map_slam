#include "utility.h"



namespace utility
{

//typedef std::pair< int,Eigen::Vector2d > NewPoseSeq;
//typedef std::vector<NewPoseSeq, Eigen::aligned_allocator<Eigen::Vector2d> > NewPoses;

//    bool read_fileFakeGPS(std::ifstream& is, NewPoses& k)
//    {
//        //todo aggiungi pair con seq (check the struct)

//        std::string line;
//        Eigen::Vector2d np;
//        NewPoseSeq fakeGPS;
//        int seq;
//        while(std::getline(is, line))
//        {
//            std::stringstream ss(line);
//            ss >> seq;
//            ss >> np.x >> np.y;
//            //TODO!!!
////            std::vector<Vector4f>& rest:

////            ss >> rest[0] >> rest[1] >> rest[2] >> rest[3];
////            k.push_back(make_pair(seq,point));
////            r.push_back(rest);

//        }
//        //    cout << k.size() << endl;
//        is.close();
//        return true;
//    }
    Eigen::Vector3d t2v(const Eigen::Isometry2d& iso)
    {
        Eigen::Vector3d t;
        t.x() = iso.translation().x();
        t.y() = iso.translation().y();
        Eigen::Rotation2Dd r(0);
        r.fromRotationMatrix(iso.linear());
        t.z() = r.angle();
        return t;
    }


    Eigen::Isometry2d v2t(const Eigen::Vector3d& v)
    {
        Eigen::Isometry2d iso;
        iso.translation() = v.head<2>();
        iso.linear() = Eigen::Rotation2Dd(v.z()).toRotationMatrix();
        return iso;
    }
}
