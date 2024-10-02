#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include  "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;
using std::vector;
using std::cout;
////1. defined a function to create matrix - as per DH conversion 
vector<vector<double>> matrix(double a,double l,double d ,double q){
////1.a - DH matrix
 vector<vector<double>> vec{{cos(q),-sin(q)*cos(l),sin(q)*sin(l),a*cos(q)},{sin(q) , cos(q)* cos(l) , -cos(q)*sin(l),a* sin(q)},{0 , sin(l),cos(l),d},{0 , 0 , 0 ,1}};
 return vec;

}
////cross product of two vectors as needed in jacobian matrix
vector<double> crossproduct(const vector<double>&a , const vector<double>&b){
 double x = (a[1]*b[2] )- (a[2]*b[1]);
 double y = (a[2]*b[0] )- (a[0]*b[2]);
 double z = (a[0]*b[1] )- (a[1]*b[0]);
 vector<double> result={x,y,z};
 return result;
 }


////calclucation of Zaxis vector for jacobian matrix
vector<double> zvalue(const vector<vector<double>> & matrix){
return {matrix[0][2],matrix[1][2],matrix[2][2]};
}
/////calculation of origin of a frame vector for jacobian matrix
vector<double> ovalue(const vector<vector<double>> & matrix){
return {matrix[0][3],matrix[1][3],matrix[2][3]};
}
////susbtraction in two vectors
vector<double> subs(const vector<double>&a ,const vector<double>&b){
 double x = a[0]-b[0];
 double y = a[1]-b[1];
 double z = a[2]-b[2];
 vector<double> result={x,y,z};
 return result;
 }
 
 
// Function to calculate the determinant of a 3x3 matrix
double determinant(const vector<vector<double>>& mat) {
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) -
           mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
           mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
}

// Function to calculate the inverse of a 3x3 matrix
void inv(const vector<vector<double>>& mat, vector<vector<double>>& inv) {
       
    double det = determinant(mat);  

    // Calculate the inverse using the formula for the inverse of a 3x3 matrix
    inv[0][0] = (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) / det;
    inv[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) / det;
    inv[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) / det;

    inv[1][0] = (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]) / det;
    inv[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) / det;
    inv[1][2] = (mat[0][2] * mat[1][0] - mat[0][0] * mat[1][2]) / det;

    inv[2][0] = (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]) / det;
    inv[2][1] = (mat[0][1] * mat[2][0] - mat[0][0] * mat[2][1]) / det;
    inv[2][2] = (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) / det;
    return ;
    
}





/////2. Defined a function to multiply two matrices 
vector<vector<double>> mat_multi(const vector<vector<double>>&a , const vector<vector<double>>&b){
int m = a.size();
int n=a[0].size();
int p=b[0].size();

////2.a initiate all the elements of new martix
vector<std::vector<double>> c(m, vector<double>(p, 0.0));
/////use of loop to put the values as per multiplication formula
for(auto i=0;i<m;i++){
   for(auto j=0 ;j<p;j++){
   	for (auto h=0;h<n;h++){
   	   c[i][j]+=a[i][h]*b[h][j];
   	}}}
   
return c;
}
vector<vector<double>> assigner()
    {
        // Assuming values for q 30, 0 and 90
        double q1 = 30 * (M_PI / 180);
        double q2 = 0 * (M_PI / 180);
        double q3 = 90 * (M_PI / 180);

        // Initializing three DH matrices A1, A2 & A3
        vector<vector<double>> A1 = matrix(0, M_PI / 2, 1, q1);
        vector<vector<double>> A2 = matrix(1, 0, 0, q2);
        vector<vector<double>> A3 = matrix(1, 0, 0, q3);
        vector<vector<double>> B = mat_multi(A1, A2);
        vector<vector<double>> T = mat_multi(B, A3);
        vector<vector<double>> final_pos = mat_multi(T, {{0}, {0}, {0}, {1}});

        // Assigning values of origins and z-axis vectors
        vector<double> o4 = {final_pos[0][0], final_pos[1][0], final_pos[2][0]};
        vector<double> o3 = ovalue(B);
        vector<double> o2 = ovalue(A1);
        vector<double> o1 = {0, 0, 0};
        vector<double> z0 = {0, 0, 1};
        vector<double> z1 = zvalue(A1);
        vector<double> z2 = zvalue(B);

        // Calculating Jacobian matrix
        vector<vector<double>>jacobian = {
            {crossproduct(z0, subs(o4, o1))[0], crossproduct(z1, subs(o4, o2))[0], crossproduct(z2, subs(o4, o3))[0]},
            {crossproduct(z0, subs(o4, o1))[1], crossproduct(z1, subs(o4, o2))[1], crossproduct(z2, subs(o4, o3))[1]},
            {crossproduct(z0, subs(o4, o1))[2], crossproduct(z1, subs(o4, o2))[2], crossproduct(z2, subs(o4, o3))[2]},
            {z0[0], z1[0], z2[0]},
            {z0[1], z1[1], z2[1]},
            {z0[2], z1[2], z2[2]}
        };
        return jacobian;
        }
        
        
  vector<vector<double>> invassigner(const vector<vector<double>> & jacobian){
        // Considering only velocity Jacobian to check the inverse
        vector<vector<double>> jacob = {{jacobian[0][0], jacobian[0][1], jacobian[0][2]},
                                        {jacobian[1][0], jacobian[1][1], jacobian[1][2]},
                                        {jacobian[2][0], jacobian[2][1], jacobian[2][2]}};
        vector<vector<double>> jacobinv(3, vector<double>(3, 0));
        
       
       
      inv(jacob, jacobinv);
      return jacobinv;
        
        
     
    }


////3. defining a new class inherited from class Node



class vel_node : public rclcpp::Node
{
public:
    vel_node()
        : Node("vel_node")
    {
        subscription_1 = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "topic1", 10, std::bind(&vel_node::topic_callback, this, _1));
        
        subscription_2 = this->create_subscription<geometry_msgs::msg::Point>(
            "topic2", 10, std::bind(&vel_node::topic_geo, this, _1));
    }

    

private:
       void topic_callback(const std_msgs::msg::Float32MultiArray & msg) const
    {
   /////5.Input value in form of joint velocities
   ///// let these velocities be v1 ,v2,v3 
      double v1= msg.data[0];
      double v2 =msg.data[1];
      double v3 = msg.data[2];
      vector<vector<double>>v = {{v1},{v2},{v3}};
      
      
      vector<vector<double>>jacobian=assigner();
      
      vector<vector<double>>speed=mat_multi(jacobian,v);
      
     ////8. display of final results.
    RCLCPP_INFO(this->get_logger(), "Final twist values for end effector :'%f %f %f %f %f %f'",speed[0][0],speed[1][0],speed[2][0],speed[3][0],speed[4][0],speed[5][0]);
    
    
    }
     //////new function for subscriber_2
     
     
     void topic_geo(const geometry_msgs::msg::Point & msg) const
    {
   /////get the position from the msg
      double vx=msg.x;
      double vy=msg.y;
      double vz=msg.z;
      vector<vector<double> >v = {{vx},{vy},{vz}};
      vector<vector<double>> jacobian=assigner();
       vector<vector<double>> jacob = {{jacobian[0][0], jacobian[0][1], jacobian[0][2]},
                                        {jacobian[1][0], jacobian[1][1], jacobian[1][2]},
                                        {jacobian[2][0], jacobian[2][1], jacobian[2][2]}};
      
      double det = determinant(jacob);                                
      
      
      
       if(det==0){
      RCLCPP_INFO(this->get_logger(), "Singularity in system , values cannot be calculated");
    
      }
      else{
      vector<vector<double>> jacobinv=invassigner(jacobian);
     vector<vector<double>>jointw=mat_multi(jacobinv,v);
    //RCLCPP_INFO(this->get_logger(), "ok till here ");
    RCLCPP_INFO(this->get_logger(), "Value of velocities :'%f %f %f'",jointw[0][0],jointw[1][0],jointw[2][0]);
    }}


    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_1;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_2;

    vector<vector<double>> jacobian;
    vector<vector<double>> jacobinv;
    
    vector<double> o1, o2, o3, o4; // Origins
    vector<double> z0, z1, z2;     // Z-axis vectors
};

    

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vel_node>());
  rclcpp::shutdown();
  return 0;
}
