function [q_dot] = inv_diff_kin(R, ve, q_pose)
      %R - SerialLink Object
      %ve - specifc end-effector velocity
      %q_pose - specific pose the robot is in
      
      %Outputs:
      %q_dot - time derivative to get ve at a specific pose

       J = R.jacob0(q_pose);
       q_dot = pinv(J) * ve;
       
end

