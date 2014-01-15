#include <kdl/chain.hpp>
#include <kdl_format_io/urdf_import.hpp>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <kdl/frames_io.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>

const double deg2rad = M_PI/180.0;


double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}


KDL::Chain getDHComanLeftArmChain()
{
    KDL::Chain left_arm;
    
    /*
     link_0 (A 0.0)    (D 0.112)   (alpha 110.0)  (offset  90.0)  (min -180.0)  (max 180.0)
     link_1 (A 0.0)    (D 0.022)   (alpha -90.0)  (offset  90.0)  (min -180.0)  (max 180.0)
     link_2 (A 0.0)    (D 0.0)     (alpha  90.0)  (offset -70.0)  (min -180.0)  (max 180.0)
     link_3 (A 0.015)  (D -0.203)  (alpha -90.0)  (offset   0.0)  (min -180.0)  (max 180.0)
     link_4 (A 0.0)    (D 0.153)   (alpha -90.0)  (offset  90.0)  (min -180.0)  (max 180.0)
     link_5 (A 0.0)    (D 0.0)     (alpha 90.0)   (offset -90.0)  (min -180.0)  (max 180.0)
     link_6 (A 0.015)  (D 0.180)   (alpha -90.0)  (offset  90.0)  (min -180.0)  (max 180.0)
     link_7 (A 0.210)  (D 0.0)     (alpha 180.0)  (offset   0.0)  (min -180.0)  (max 180.0)
     */
    
    left_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH(0,deg2rad*110,0.112,deg2rad*90)));
    left_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,deg2rad*-90,0.022,deg2rad*90)));
    left_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,deg2rad*90,0.0,deg2rad*-70)));
    left_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.015,deg2rad*-90,-0.203,deg2rad*0)));
    left_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,deg2rad*-90,0.153,deg2rad*90)));
    left_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,deg2rad*90,0.0,deg2rad*-90)));
    left_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.015,deg2rad*-90,0.0180,deg2rad*90)));
    left_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.210,deg2rad*180,0.0,deg2rad*0.0)));
    return left_arm;
}

KDL::Chain getURDFComanLeftArmChain(std::string urdf_file_name)
{
    KDL::Tree coman_tree;
    KDL::Chain coman_chain;
    bool ret = kdl_format_io::treeFromUrdfFile(urdf_file_name,coman_tree);
    if( !ret ) { std::cout << "Problem on treeFromUrdfFile " << std::endl; }
    ret = coman_tree.getChain("base_link","LForearm",coman_chain);
    if( !ret ) { std::cout << "Problem on getChain " << std::endl; }
    return coman_chain;
}


KDL::Chain getDHComanRightArmChain()
{
    KDL::Chain right_arm;
    
    /**
    link_0 (A 0.0)    (D 0.112)   (alpha 110.0)  (offset  90.0)  (min -180.0)  (max 180.0)
    link_1 (A 0.0)    (D 0.022)   (alpha -90.0)  (offset  90.0)  (min -180.0)  (max 180.0)
    link_2 (A 0.0)    (D 0.0)     (alpha  90.0)  (offset -70.0)  (min -180.0)  (max 180.0)
    link_3 (A 0.015)  (D -0.203)  (alpha -90.0)  (offset   0.0)  (min -180.0)  (max 180.0)
    link_4 (A 0.0)    (D -0.153)  (alpha  90.0)  (offset  90.0)  (min -180.0)  (max 180.0)
    link_5 (A 0.0)    (D 0.0)     (alpha -90.0)  (offset -90.0)  (min -180.0)  (max 180.0)
    link_6 (A 0.015)  (D -0.180)  (alpha  90.0)  (offset  90.0)  (min -180.0)  (max 180.0)
    link_7 (A 0.210)  (D 0.0)     (alpha 180.0)  (offset   0.0)  (min -180.0)  (max 180.0)
    */
    
    right_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH(0,deg2rad*110,0.112,deg2rad*90)));
    right_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,deg2rad*-90,0.022,deg2rad*90)));
    right_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,deg2rad*90,0.0,deg2rad*-70)));
    right_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.015,deg2rad*-90,-0.203,deg2rad*0)));
    right_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,deg2rad*90,-0.153,deg2rad*90)));
    right_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,deg2rad*-90,0.0,deg2rad*-90)));
    right_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.015,deg2rad*90,-0.0180,deg2rad*90)));
    right_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.210,deg2rad*180,0.0,deg2rad*0.0)));
    return right_arm; 
}

KDL::Chain getURDFComanRightArmChain(std::string urdf_file_name)
{
    KDL::Tree coman_tree;
    KDL::Chain coman_chain;
    bool ret = kdl_format_io::treeFromUrdfFile(urdf_file_name,coman_tree);
    if( !ret ) { std::cout << "Problem on treeFromUrdfFile " << std::endl; }
    ret = coman_tree.getChain("base_link","r_wrist",coman_chain);
    if( !ret ) { std::cout << "Problem on getChain " << std::endl; }
    return coman_chain;
}

int main(int argc, char ** argv)
{
    if( argc != 2 ) {
        std::cout << "Usage dh_coman coman.urdf" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string coman_urdf_filename = argv[1];
    
    KDL::Chain left_arm_urdf = getURDFComanLeftArmChain(coman_urdf_filename);
    KDL::Chain left_arm_dh = getDHComanLeftArmChain();
    
    if( left_arm_dh.getNrOfJoints() != left_arm_urdf.getNrOfJoints() ) {
        std::cout << "Test failed: number of joints different" << std::endl;
        std::cout << "DH:   " << left_arm_dh.getNrOfJoints() << std::endl;
        std::cout << "URDF: " << left_arm_urdf.getNrOfJoints() << std::endl;
        return EXIT_FAILURE;
    }
    
    KDL::ChainFkSolverPos_recursive left_arm_urdf_slv(left_arm_urdf);
    KDL::ChainFkSolverPos_recursive left_arm_dh_slv(left_arm_dh);
    
    KDL::JntArray q(left_arm_dh.getNrOfJoints());
    
    double gain = 0;
    for(int i = 0; i < q.rows(); i++ ) { q(i) = gain*random_double(); } 
    
    KDL::Frame left_frame_dh, left_frame_urdf;
    left_arm_dh_slv.JntToCart(q,left_frame_dh);
    left_arm_urdf_slv.JntToCart(q,left_frame_urdf);
    
    std::cout << "Left frame calculated with DH parameters: " << std::endl;
    std::cout << left_frame_dh << std::endl;
    
    std::cout << "Left frame calculated with URDF model " << std::endl;
    std::cout << left_frame_urdf << std::endl;
    
    
    KDL::Chain right_arm_urdf = getURDFComanRightArmChain(coman_urdf_filename);
    KDL::Chain right_arm_dh = getDHComanRightArmChain();
    
    if( right_arm_dh.getNrOfJoints() != right_arm_urdf.getNrOfJoints() ) {
        std::cout << "Test failed: number of joints different" << std::endl;
    }
    
    KDL::ChainFkSolverPos_recursive right_arm_urdf_slv(right_arm_urdf);
    KDL::ChainFkSolverPos_recursive right_arm_dh_slv(right_arm_dh);
    
    //KDL::JntArray q(left_arm_dh.getNrOfJoints());
    
    //for(int i = 0; i < q.rows(); i++ ) { q(i) = random_double(); } 
    
    KDL::Frame right_frame_dh, right_frame_urdf;
    right_arm_dh_slv.JntToCart(q,right_frame_dh);
    right_arm_urdf_slv.JntToCart(q,right_frame_urdf);
    
    std::cout << "Right frame calculated with DH parameters: " << std::endl;
    std::cout << right_frame_dh << std::endl;
    
    std::cout << "Right frame calculated with URDF model " << std::endl;
    std::cout << right_frame_urdf << std::endl;
    
    
    return EXIT_SUCCESS;
    
}
