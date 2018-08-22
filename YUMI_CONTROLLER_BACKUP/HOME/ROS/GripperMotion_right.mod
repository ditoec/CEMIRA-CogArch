MODULE GripperMotion_right

! Software License Agreement (BSD License)
!
! Copyright (c) 2012, Edward Venator, Case Western Reserve University
! Copyright (c) 2012, Jeremy Zoss, Southwest Research Institute
! All rights reserved.
!
! Redistribution and use in source and binary forms, with or without modification,
! are permitted provided that the following conditions are met:
!
!   Redistributions of source code must retain the above copyright notice, this
!       list of conditions and the following disclaimer.
!   Redistributions in binary form must reproduce the above copyright notice, this
!       list of conditions and the following disclaimer in the documentation
!       and/or other materials provided with the distribution.
!   Neither the name of the Case Western Reserve University nor the names of its contributors
!       may be used to endorse or promote products derived from this software without
!       specific prior written permission.
!
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
! EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
! OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
! SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
! INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
! TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
! BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
! WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


PROC main()
    VAR num grasp_force;
    VAR num prev_grasp_force;
    
    g_JogIn;
    WaitTime 3;
    g_Calibrate\Jog;
    g_Init \maxSpd:=20, \holdForce:=15;
    g_GripOut;
    grasp_force:=0;
    prev_grasp_force:=0;
    
    WHILE true DO
        ! Check for an updated setpoint. 
        grasp_force := next_grasp_target.right;
        current_gripper_right := g_GetPos();
        
        IF(grasp_force <> prev_grasp_force) THEN
            IF(grasp_force > 0 ) THEN
                ! TPWrite "Grasping with right";
                g_GripIn \holdForce:=grasp_force;
            ELSEIF (grasp_force < 0 ) THEN
                g_GripOut \holdForce:=-grasp_force;
            ELSE
                !do nothing
            ENDIF
            prev_grasp_force:=grasp_force;
        ENDIF
        
        WaitTime 0.1;
    ENDWHILE
ERROR
    ErrWrite \W, "Gripper Motion Error", "Error executing motion.  Aborting trajectory.";
ENDPROC


ENDMODULE
