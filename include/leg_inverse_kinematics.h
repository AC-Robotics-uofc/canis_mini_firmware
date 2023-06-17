class LegInverseKinematicsProcessor
{
    public:
        static bool calculate_superior_right_leg_angles();
        static bool calculate_superior_left_leg_angles(); 
        static bool calculate_inferior_right_leg_angles(); 
        static bool calculate_inferior_left_leg_angles(); 


    private:
        double shoulder_length;
        double arm_length;
        double forearm_length;
};
