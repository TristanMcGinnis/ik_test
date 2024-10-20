#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

class KDLTestNode : public rclcpp::Node {
public:
    KDLTestNode() : Node("kdl_test_node") {
        // Load URDF
        urdf::Model urdf_model;
        std::string urdf_file = "/home/tristan/Downloads/arm06.urdf";
        if (!urdf_model.initFile(urdf_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF");
            return;
        }
        
        // Parse URDF and create KDL tree
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create KDL tree");
            return;
        }

        // Extract chain from base to end-effector
        KDL::Chain kdl_chain;
        if (!kdl_tree.getChain("base_link", "continuous", kdl_chain)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "KDL chain created with %d segments.", kdl_chain.getNrOfSegments());

        // Forward kinematics example
        testForwardKinematics(kdl_chain);
    }

private:
    void testForwardKinematics(const KDL::Chain& chain) {
        // Define solvers
        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::JntArray joint_positions(chain.getNrOfJoints());

        // Define some joint positions (replace with actual joint values)
        for (unsigned int i = 0; i < joint_positions.rows(); ++i) {
            joint_positions(i) = 0.0; // Example joint angles, replace as needed
        }

        // Output for the FK solver
        KDL::Frame end_effector_pose;

        // Compute forward kinematics
        if (fk_solver.JntToCart(joint_positions, end_effector_pose) >= 0) {
            RCLCPP_INFO(this->get_logger(), "End effector position: (%f, %f, %f)", 
                        end_effector_pose.p.x(), 
                        end_effector_pose.p.y(), 
                        end_effector_pose.p.z());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute forward kinematics");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KDLTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
